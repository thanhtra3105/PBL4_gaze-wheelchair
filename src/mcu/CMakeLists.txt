idf_component_register(
    SRCS
        "main.c"
        "joystick_motor.c"
        "mode_manager.c"
       
        "uart_control.c"
       
        "hall_rpm.c"
        "rpm_pid.c"
    INCLUDE_DIRS "."
    REQUIRES
        driver
        esp_adc
        esp_wifi
        mqtt
        nvs_flash
        esp_netif
        esp_event
)