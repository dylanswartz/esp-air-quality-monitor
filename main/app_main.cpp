/*
 * Copyright (c) 2022 Golioth, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "WiFi.h"
#include "../credentials.h"
#include "golioth.h"

#include "Arduino.h"
#include "Wire.h"
#include "SeeedOLED.h"
#include "Air_Quality_Sensor.h"
#include "Seeed_BME280.h"


#define TAG "esp-air-quality-monitor"

// LED connection: LED_PIN--LED--Resistor--Ground
#define LED_PIN 13
#define OLED_SCL 23
#define OLED_SDA 22
#define DUST_SENSOR_PIN 15
#define AQS_PIN 34
#define SENSOR_READING_INTERVAL 30000

// I2C device found at address 0x3C
// I2C device found at address 0x76
AirQualitySensor aqSensor(AQS_PIN);
BME280 bme;

unsigned long lastInterval;
unsigned long lowpulseoccupancy = 0;
unsigned long last_lpo = 0;
unsigned long duration;


float ratio = 0;
float concentration = 0;

int getBMEValues(int &temp, int &humidity, int &pressure);
void getDustSensorReadings();
String getAirQualityString();
int getAirQualityInt();
void createEventPayload(golioth_client_t client, int temp, int humidity, int pressure, String airQualityReadable);
void updateDisplay(int temp, int humidity, int pressure, String airQuality);

// Current firmware version
const char* _current_version = "1.1.0";

// Configurable via Settings service, key = "LOOP_DELAY_S"
int32_t _loop_delay_s = 10;

// Given if/when the we have a connection to Golioth
static golioth_sys_sem_t _connected_sem;

static golioth_rpc_status_t on_multiply(
        const char* method,
        const cJSON* params,
        uint8_t* detail,
        size_t detail_size,
        void* callback_arg) {
    if (cJSON_GetArraySize(params) != 1) {
        return RPC_INVALID_ARGUMENT;
    }
    int num_to_double = cJSON_GetArrayItem(params, 0)->valueint;
    snprintf((char*)detail, detail_size, "{ \"value\": %d }", 2 * num_to_double);
    return RPC_OK;
}
// Callback for Golioth connect/disconnect events
void on_client_event(golioth_client_t client, golioth_client_event_t event, void* arg) {
    bool is_connected = (event == GOLIOTH_CLIENT_EVENT_CONNECTED);
    if (is_connected) {
        golioth_sys_sem_give(_connected_sem);
    }
    GLTH_LOGI(TAG, "Golioth client %s", is_connected ? "connected" : "disconnected");

  int err = golioth_rpc_register(client, "multiply", on_multiply, NULL);
	if (err) {
		GLTH_LOGI(TAG, "Failed to observe RPC: %d", err);
	}
}

// Callback for Golioth Settings Service updates
golioth_settings_status_t on_loop_delay_setting(int32_t new_value, void* arg) {
    GLTH_LOGI(TAG, "Setting loop delay to %" PRId32 " s", new_value);
    _loop_delay_s = new_value;
    return GOLIOTH_SETTINGS_SUCCESS;
}


// Main function
extern "C" void app_main(void) {
    initArduino();
    // This is where the Arduino setup() code would be found

    pinMode(LED_PIN, OUTPUT);

    // Arduino-like setup()
    Serial.begin(115200);
    delay(50);
    while(!Serial){
      ; // wait for serial port to connect
    }
    Serial.println("This is an Arduino function.");

    WiFi.begin(WIFI_SSID, WIFI_PASS);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());

    // Configure the dust sensor pin as an input
    pinMode(DUST_SENSOR_PIN, INPUT);

    Serial.println("Waiting sensor to init...");
    delay(1000);

    if (aqSensor.init())
    {
        Serial.println("Air Quality Sensor ready.");
    }
    else
    {
        Serial.println("Air Quality Sensor ERROR!");
    }

    Wire.begin(OLED_SCL, OLED_SDA);
    SeeedOled.init();

    SeeedOled.clearDisplay();
    SeeedOled.setNormalDisplay();
    SeeedOled.setPageMode();

    SeeedOled.setTextXY(2, 0);
    SeeedOled.putString("Golioth");
    SeeedOled.setTextXY(3, 0);
    SeeedOled.putString("Air Quality");
    SeeedOled.setTextXY(4, 0);
    SeeedOled.putString("Monitor");

    if (bme.init())
    {
        Serial.println("BME280 Sensor ready.");
    }
    else
    {
        Serial.println("BME280 Sensor ERROR!");
    }

    lastInterval = millis();

    // Now we are ready to connect to the Golioth cloud.
    //
    // To start, we need to create a client. The function golioth_client_create will
    // dynamically create a client and return a handle to it.
    //
    // The client itself runs in a separate task, so once this function returns,
    // there will be a new task running in the background.
    //
    // As soon as the task starts, it will try to connect to Golioth using the
    // CoAP protocol over DTLS, with the PSK ID and PSK for authentication.

    golioth_client_config_t config = {
            .credentials = {
                    .auth_type = GOLIOTH_TLS_AUTH_TYPE_PSK,
                    .psk = {
                            .psk_id = PSK_ID,
                            .psk_id_len = strlen(PSK_ID),
                            .psk = PSK,
                            .psk_len = strlen(PSK),
                    }}};
    golioth_client_t client = golioth_client_create(&config);
    assert(client);


    uint16_t counter = 0;
    Serial.println("This is where Arduino loop() functions happen");
    
    delay(1000); // if I remove this it bootloops and I don't know why

    // Demonstrate Golioth Features

    // Register a callback function that will be called by the client task when
    // connect and disconnect events happen.
    //
    // This is optional, but can be useful for synchronizing operations on connect/disconnect
    // events. For this example, the on_client_event callback will simply log a message.
    golioth_sys_sem_t _connected_sem = golioth_sys_sem_create(1, 0);
    golioth_client_register_event_callback(client, on_client_event, NULL);

    // Check for a Golioth connect and wait if one is not present
    if (!golioth_client_is_connected(client)) {
        GLTH_LOGI(TAG, "Waiting for connection to Golioth...");
        golioth_sys_sem_take(_connected_sem, GOLIOTH_SYS_WAIT_FOREVER);
    }

    // For OTA, we will spawn a background task that will listen for firmware
    // updates from Golioth and automatically update firmware on the device
    golioth_fw_update_init(client, _current_version);

    // We can register a callback for persistent settings. The Settings service
    // allows remote users to manage and push settings to devices.
    golioth_settings_register_int(client, "LOOP_DELAY_S", on_loop_delay_setting, NULL);

    int err = golioth_rpc_register(client, "multiply", on_multiply, NULL);
    if (err) {
           GLTH_LOGI(TAG,"Failed to register RPC: %d", err);
    }

    while(1) {
        // Send a log message to the server with the counter value
        GLTH_LOGI(TAG, "Counter value: %d", counter);

        digitalWrite(LED_PIN, counter%2);

        int temp, pressure, humidity;

        duration = pulseIn(DUST_SENSOR_PIN, LOW);
        lowpulseoccupancy = lowpulseoccupancy + duration;

        if ((millis() - lastInterval) > SENSOR_READING_INTERVAL)
        {
            String qualityReadable = getAirQualityString();
            // int qualityValue = getAirQualityInt();
            Serial.print("Air Quality: ");
            Serial.println(qualityReadable.c_str());

            getBMEValues(temp, pressure, humidity);
            Serial.print("Temp: ");
            Serial.println(temp);

            Serial.print("Pressure: ");
            Serial.println(pressure);

            Serial.print("Humidity: ");
            Serial.println(humidity);

            getDustSensorReadings();

            updateDisplay(temp, humidity, pressure, qualityReadable);

            createEventPayload(client, temp, humidity, pressure, qualityReadable);

            lowpulseoccupancy = 0;
            lastInterval = millis();
        }

        // Send the most recent counter value to Golioth LightDB State
        golioth_lightdb_set_int_async(client, "counter", counter, NULL, NULL);

        ++counter;

        // Call the Golioth SDK sleep (delay) function. The _loop_delay_s value
        // is remotely configurable using the Golioth Settings Service.
        golioth_sys_msleep(_loop_delay_s*1000);
    }
}

//helpers
String getAirQualityString()
{
  int quality = aqSensor.slope();
  String qual = "None";

  if (quality == AirQualitySensor::FORCE_SIGNAL)
  {
    qual = "Danger";
  }
  else if (quality == AirQualitySensor::HIGH_POLLUTION)
  {
    qual = "High Pollution";
  }
  else if (quality == AirQualitySensor::LOW_POLLUTION)
  {
    qual = "Low Pollution";
  }
  else if (quality == AirQualitySensor::FRESH_AIR)
  {
    qual = "Fresh Air";
  }

  return qual;
}

int getAirQualityInt()
{
  return aqSensor.slope();
}

int getBMEValues(int &temp, int &pressure, int &humidity)
{
  temp = (int)bme.getTemperature();
  pressure = (int)(bme.getPressure() / 100.0F);
  humidity = (int)bme.getHumidity();

  return 1;
}

void getDustSensorReadings()
{
  // This particular dust sensor returns 0s often, so let's filter them out by making sure we only
  // capture and use non-zero LPO values for our calculations once we get a good reading.
  if (lowpulseoccupancy == 0)
  {
    lowpulseoccupancy = last_lpo;
  }
  else
  {
    last_lpo = lowpulseoccupancy;
  }

  ratio = lowpulseoccupancy / (SENSOR_READING_INTERVAL * 10.0);                   // Integer percentage 0=>100
  concentration = 1.1 * pow(ratio, 3) - 3.8 * pow(ratio, 2) + 520 * ratio + 0.62; // using spec sheet curve


  Serial.print("LPO: ");
  Serial.println(lowpulseoccupancy);

  Serial.print("Ratio: ");
  Serial.println(ratio);

  Serial.print("Concentration: ");
  Serial.println(concentration);
}

void createEventPayload(golioth_client_t client, int temp, int humidity, int pressure, String airQualityReadable)
{

    Serial.println("Creating payload");
    golioth_lightdb_set_int_async(client, "temperature", temp, NULL, NULL);
    golioth_lightdb_set_int_async(client, "humidity", humidity, NULL, NULL);
    golioth_lightdb_set_int_async(client, "pressure", pressure, NULL, NULL);

    golioth_lightdb_stream_set_int_async(client, "temperature", temp, NULL, NULL);
    golioth_lightdb_stream_set_int_async(client, "humidity", humidity, NULL, NULL);
    golioth_lightdb_stream_set_int_async(client, "pressure", pressure, NULL, NULL);

}

void updateDisplay(int temp, int humidity, int pressure, String airQuality)
{
  SeeedOled.clearDisplay();

  SeeedOled.setTextXY(0, 3);
  SeeedOled.putString(airQuality.c_str());

  SeeedOled.setTextXY(2, 0);
  SeeedOled.putString("Temp: ");
  SeeedOled.putNumber(temp);
  SeeedOled.putString("C");

  SeeedOled.setTextXY(3, 0);
  SeeedOled.putString("Humidity: ");
  SeeedOled.putNumber(humidity);
  SeeedOled.putString("%");

  SeeedOled.setTextXY(4, 0);
  SeeedOled.putString("Press: ");
  SeeedOled.putNumber(pressure);
  SeeedOled.putString(" hPa");

  if (concentration > 1)
  {
    SeeedOled.setTextXY(5, 0);
    SeeedOled.putString("Dust: ");
    SeeedOled.putNumber(concentration); // Will cast our float to an int to make it more compact
    SeeedOled.putString(" pcs/L");
  }
}