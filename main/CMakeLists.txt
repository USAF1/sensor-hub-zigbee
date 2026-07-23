idf_component_register(
    SRCS
        "main.c"
        "uart_master.c"
        "uart_hooks.c"
    PRIV_INCLUDE_DIRS "."
    PRIV_REQUIRES
        nvs_flash
        esp_wifi
        esp-zigbee-lib
        esp_timer
        driver
)