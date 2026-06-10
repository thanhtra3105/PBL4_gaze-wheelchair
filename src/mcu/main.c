
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "joystick_motor.h"
#include "mode_manager.h"
// #include "mqtt_control.h"
#include "uart_control.h"
// #include "sensor_monitor.h"
#include "hall_rpm.h"
#include "rpm_pid.h"

static const char *TAG = "MAIN";

void app_main(void)
{
    ESP_LOGI(TAG, "=== Robot Multi-Mode Boot ===");

    adc_init();
    pwm_init();

    mode_manager_init();

    uart_control_init();

    hall_rpm_init();
    rpm_pid_init();
    ESP_LOGI(TAG, "Tat ca peripheral da san sang. Khoi tao task...");

    xTaskCreate(joystick_task, "task_joystick", 4096, NULL, 5, NULL);
    xTaskCreate(uart_control_task, "task_uart", 4096, NULL, 5, NULL);
    xTaskCreate(hall_rpm_task, "task_hall_rpm", 4096, NULL, 6, NULL);
    xTaskCreate(rpm_pid_task, "task_rpm_pid", 4096, NULL, 7, NULL);
    
    ESP_LOGI(TAG, "He thong khoi dong thanh cong.");
    ESP_LOGI(TAG, "Mode mac dinh: JOYSTICK ");
}