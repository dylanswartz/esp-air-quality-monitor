set(srcs
    "app_main.cpp"
    "SeeedOLED.cpp"
    "Seeed_BME280.cpp"
    "Air_Quality_Sensor.cpp"
)

set(deps
    "golioth_sdk"
    "arduino"
    "nvs_flash"
    "json"
)

idf_component_register(
    SRCS "${srcs}"
    PRIV_REQUIRES "${deps}"
)
list(APPEND EXTRA_C_FLAGS_LIST
    -Werror
)
component_compile_options(${EXTRA_C_FLAGS_LIST})
