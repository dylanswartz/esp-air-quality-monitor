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


#define TAG "golioth_espidf_arduino"

// LED connection: LED_PIN--LED--Resistor--Ground
#define LED_PIN 13
#define OLED_SCL 23
#define OLED_SDA 22

// Current firmware version
const char* _current_version = "1.0.4";

// Configurable via Settings service, key = "LOOP_DELAY_S"
int32_t _loop_delay_s = 10;

// Given if/when the we have a connection to Golioth
static golioth_sys_sem_t _connected_sem;

// Callback for Golioth connect/disconnect events
void on_client_event(golioth_client_t client, golioth_client_event_t event, void* arg) {
    bool is_connected = (event == GOLIOTH_CLIENT_EVENT_CONNECTED);
    if (is_connected) {
        golioth_sys_sem_give(_connected_sem);
    }
    GLTH_LOGI(TAG, "Golioth client %s", is_connected ? "connected" : "disconnected");
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

    // Lets make the OLED Display something: 
    Wire.begin(OLED_SCL, OLED_SDA);
    SeeedOled.init();  //initialze SEEED OLED display
    SeeedOled.clearDisplay();          //clear the screen and set start position to top left corner
    SeeedOled.setNormalDisplay();      //Set display to normal mode (i.e non-inverse mode)
    SeeedOled.setPageMode();           //Set addressing mode to Page Mode
    SeeedOled.setTextXY(0, 0);         //Set the cursor to Xth Page, Yth Column
    SeeedOled.putString("Booting..."); //Print the String

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
    delay(1000); /// If I comment this out it bootloops 
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

    SeeedOled.clearDisplay();
    SeeedOled.setTextXY(0, 0); 
    SeeedOled.putString("Hello Golioth!");

    while(1) {
        // Send a log message to the server with the counter value
        GLTH_LOGI(TAG, "Counter value: %d", counter);

        digitalWrite(LED_PIN, counter%2);

        Serial.print("Hello Golioth! #");
        Serial.println(counter);

        SeeedOled.setTextXY(2, 0);
        SeeedOled.putString("Count: ");
        SeeedOled.putNumber(counter);

        // Send the most recent counter value to Golioth LightDB State
        golioth_lightdb_set_int_async(client, "counter", counter, NULL, NULL);
        ++counter;

        // Call the Golioth SDK sleep (delay) function. The _loop_delay_s value
        // is remotely configurable using the Golioth Settings Service.
        golioth_sys_msleep(_loop_delay_s*1000);
    }
}
