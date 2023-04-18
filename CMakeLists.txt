# Define the environment variables
set(WIFI_SSID "$ENV{WIFI_SSID}")
set(WIFI_PASS "$ENV{WIFI_PASS}")
set(PSK_ID "$ENV{PSK_ID}")
set(PSK "$ENV{PSK}")

cmake_minimum_required(VERSION 3.5)
set(EXTRA_COMPONENT_DIRS ./third_party/golioth-firmware-sdk/port/esp_idf/components)
list(APPEND EXTRA_COMPONENT_DIRS ./third_party/arduino)
list(APPEND EXTRA_COMPONENT_DIRS ./third_party/esp32-arduino-lib-builder/components/arduino_tinyusb)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp-air-quality-monitor)
configure_file(
  ${CMAKE_SOURCE_DIR}/credentials_template.h.in
  ${CMAKE_BINARY_DIR}/credentials.h
  @ONLY
)