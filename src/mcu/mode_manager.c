#include "mode_manager.h"

#include "esp_log.h"
#include "joystick_motor.h"
#include "rpm_pid.h"

static const char *TAG = "MODE_MGR";

static volatile robot_mode_t s_current_mode = MODE_UART; // khởi động vào UART để dễ test với Jetson trước, đổi lại MODE_JOYSTICK nếu muốn dùng joystick ngay từ đầu
static volatile sensor_data_t s_sensor = {0};

EventGroupHandle_t g_mode_event_group = NULL;

static const char *mode_name(robot_mode_t mode)
{
    switch (mode) {
        case MODE_JOYSTICK: return "JOYSTICK";
        case MODE_UART:     return "UART";
        default:            return "UNKNOWN";
    }
}

static EventBits_t mode_to_bit(robot_mode_t mode)
{
    switch (mode) {
        case MODE_JOYSTICK: return BIT_MODE_JOYSTICK;
        case MODE_UART:     return BIT_MODE_UART;
        default:            return BIT_MODE_JOYSTICK;
    }
}

static void apply_mode(robot_mode_t new_mode)
{
    if (new_mode == s_current_mode) {
        return;
    }

    ESP_LOGI(TAG, "Chuyen mode: %s -> %s",
             mode_name(s_current_mode),
             mode_name(new_mode));

    /* Dừng motor ngay lúc đổi mode để tránh task cũ còn giữ lệnh */
    rpm_pid_stop();
    motor_set_lr(0, 0);

    s_current_mode = new_mode;

    xEventGroupClearBits(g_mode_event_group,
                         BIT_MODE_JOYSTICK | BIT_MODE_UART);

    xEventGroupSetBits(g_mode_event_group, mode_to_bit(new_mode));
}

static void toggle_mode(void)
{
    if (s_current_mode == MODE_JOYSTICK) {
        apply_mode(MODE_UART);
    } else {
        apply_mode(MODE_JOYSTICK);
    }
}

static void mode_button_task(void *arg)
{
    int last_level = gpio_get_level(MODE_BUTTON_GPIO);
    TickType_t last_press_tick = 0;

    ESP_LOGI(TAG, "Button mode task started. GPIO%d, default mode=%s",
             MODE_BUTTON_GPIO,
             mode_name(s_current_mode));

    for (;;) {
        int level = gpio_get_level(MODE_BUTTON_GPIO);

        /* Bắt cạnh nhấn: từ 1 xuống 0 */
        if (last_level != MODE_BUTTON_PRESSED_LEVEL &&
            level == MODE_BUTTON_PRESSED_LEVEL) {

            TickType_t now = xTaskGetTickCount();

            if ((now - last_press_tick) >= pdMS_TO_TICKS(MODE_BUTTON_DEBOUNCE_MS)) {
                last_press_tick = now;
                toggle_mode();
            }
        }

        last_level = level;

        vTaskDelay(pdMS_TO_TICKS(MODE_BUTTON_POLL_MS));
    }
}

void mode_manager_init(void)
{
    g_mode_event_group = xEventGroupCreate();
    configASSERT(g_mode_event_group);

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << MODE_BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    s_current_mode = MODE_UART;

    xEventGroupClearBits(g_mode_event_group,
                        BIT_MODE_JOYSTICK | BIT_MODE_UART);

    xEventGroupSetBits(g_mode_event_group, BIT_MODE_UART);

    ESP_LOGI(TAG, "Mode manager init: default UART, button GPIO%d",
            MODE_BUTTON_GPIO);

    xTaskCreate(mode_button_task,
                "mode_button",
                2048,
                NULL,
                10,
                NULL);
}

robot_mode_t mode_manager_get(void)
{
    return s_current_mode;
}

sensor_data_t mode_manager_get_sensor(void)
{
    return s_sensor;
}