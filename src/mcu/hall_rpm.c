#include "hall_rpm.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_err.h"

static const char *TAG = "HALL_RPM";

static volatile uint32_t left_total_pulse = 0;
static volatile uint32_t right_total_pulse = 0;

static volatile int64_t left_last_isr_us = 0;
static volatile int64_t right_last_isr_us = 0;

static float left_rpm = 0.0f;
static float right_rpm = 0.0f;

static volatile int64_t left_period_us = 0;
static volatile int64_t right_period_us = 0;

static portMUX_TYPE hall_mux = portMUX_INITIALIZER_UNLOCKED;

static volatile uint32_t right_sample_count = 0;
static volatile int64_t right_sample_start_us = 0;
static volatile int64_t right_sample_dt_us = 0;
static volatile bool right_rpm_ready = false;

static volatile uint32_t left_sample_count = 0;
static volatile int64_t left_sample_start_us = 0;
static volatile int64_t left_sample_dt_us = 0;
static volatile bool left_rpm_ready = false;

/* ================= ISR ================= */
static void IRAM_ATTR hall_right_isr(void *arg)
{
    int64_t now_us = esp_timer_get_time();

    portENTER_CRITICAL_ISR(&hall_mux);

    int64_t interval_us = now_us - right_last_isr_us;

    if (interval_us >= HALL_MIN_PULSE_INTERVAL_US) {
        right_total_pulse++;
        right_last_isr_us = now_us;

        if (right_sample_count == 0) {
            right_sample_start_us = now_us;
        }

        right_sample_count++;

        if (right_sample_count >= HALL_SAMPLE_PULSE) {
            right_sample_dt_us = now_us - right_sample_start_us;
            right_sample_count = 0;
            right_sample_start_us = now_us;
            right_rpm_ready = true;
        }
    }

    portEXIT_CRITICAL_ISR(&hall_mux);
}

static void IRAM_ATTR hall_left_isr(void *arg)
{
    int64_t now_us = esp_timer_get_time();

    portENTER_CRITICAL_ISR(&hall_mux);

    int64_t interval_us = now_us - left_last_isr_us;

    if (interval_us >= HALL_MIN_PULSE_INTERVAL_US) {
        left_total_pulse++;
        left_last_isr_us = now_us;

        if (left_sample_count == 0) {
            left_sample_start_us = now_us;
        }

        left_sample_count++;

        if (left_sample_count >= HALL_SAMPLE_PULSE) {
            left_sample_dt_us = now_us - left_sample_start_us;
            left_sample_count = 0;
            left_sample_start_us = now_us;
            left_rpm_ready = true;
        }
    }

    portEXIT_CRITICAL_ISR(&hall_mux);
}
/* ================= INIT ================= */

void hall_rpm_init(void)
{
    left_total_pulse = 0;
    right_total_pulse = 0;

    left_rpm = 0.0f;
    right_rpm = 0.0f;

    left_last_isr_us = esp_timer_get_time();
    right_last_isr_us = esp_timer_get_time();

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << HALL_LEFT_PIN) | (1ULL << HALL_RIGHT_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE
    };

    ESP_ERROR_CHECK(gpio_config(&io_conf));

    esp_err_t err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(err);
    }

    ESP_ERROR_CHECK(gpio_isr_handler_add(HALL_LEFT_PIN, hall_left_isr, NULL));
    ESP_ERROR_CHECK(gpio_isr_handler_add(HALL_RIGHT_PIN, hall_right_isr, NULL));

    ESP_LOGI(TAG, "Hall RPM init done. LEFT GPIO=%d, RIGHT GPIO=%d",
             HALL_LEFT_PIN, HALL_RIGHT_PIN);
}

/* ================= TASK ================= */
void hall_rpm_task(void *pvParameters)
{
    const int64_t timeout_us = HALL_RPM_TIMEOUT_MS * 1000LL;

    while (1) {
        int64_t now_us = esp_timer_get_time();

        bool left_ready = false;
        bool right_ready = false;

        int64_t left_dt = 0;
        int64_t right_dt = 0;

        int64_t left_last_time = 0;
        int64_t right_last_time = 0;

        portENTER_CRITICAL(&hall_mux);

        if (left_rpm_ready) {
            left_ready = true;
            left_dt = left_sample_dt_us;
            left_rpm_ready = false;
        }

        if (right_rpm_ready) {
            right_ready = true;
            right_dt = right_sample_dt_us;
            right_rpm_ready = false;
        }

        left_last_time = left_last_isr_us;
        right_last_time = right_last_isr_us;

        portEXIT_CRITICAL(&hall_mux);

        if (left_ready && left_dt > 0) {
            float rpm = (60.0f * 1000000.0f * HALL_SAMPLE_PULSE) /
                        ((float)left_dt * HALL_MAGNET_COUNT);

            portENTER_CRITICAL(&hall_mux);
            left_rpm = rpm;
            portEXIT_CRITICAL(&hall_mux);

            ESP_LOGI(TAG, "LEFT dt=%lld us, RPM=%.2f", left_dt, rpm);
        }

        if (right_ready && right_dt > 0) {
            float rpm = (60.0f * 1000000.0f * HALL_SAMPLE_PULSE) /
                        ((float)right_dt * HALL_MAGNET_COUNT);

            portENTER_CRITICAL(&hall_mux);
            right_rpm = rpm;
            portEXIT_CRITICAL(&hall_mux);

            ESP_LOGI(TAG, "RIGHT dt=%lld us, RPM=%.2f", right_dt, rpm);
        }

        if ((now_us - left_last_time) > timeout_us) {
            portENTER_CRITICAL(&hall_mux);
            left_rpm = 0.0f;
            portEXIT_CRITICAL(&hall_mux);
        }

        if ((now_us - right_last_time) > timeout_us) {
            portENTER_CRITICAL(&hall_mux);
            right_rpm = 0.0f;
            portEXIT_CRITICAL(&hall_mux);
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
/* ================= GETTER ================= */

float hall_get_left_rpm(void)
{
    float rpm;

    portENTER_CRITICAL(&hall_mux);
    rpm = left_rpm;
    portEXIT_CRITICAL(&hall_mux);

    return rpm;
}

float hall_get_right_rpm(void)
{
    float rpm;

    portENTER_CRITICAL(&hall_mux);
    rpm = right_rpm;
    portEXIT_CRITICAL(&hall_mux);

    return rpm;
}