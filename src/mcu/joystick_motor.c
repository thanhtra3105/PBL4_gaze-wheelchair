#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "joystick_motor.h"
#include "mode_manager.h"    /* BIT_MODE_JOYSTICK, g_mode_event_group */

static const char *TAG = "JOYSTICK";

static const char *dir_name[] = {
    "DUNG", "TIEN", "LUI", "TRAI", "PHAI"
};

/* ═══════════════════════════════════════════════
 *  ADC
 * ═══════════════════════════════════════════════ */
static adc_oneshot_unit_handle_t adc_handle;

static int adc_read_avg(adc_channel_t channel)
{
    int sum = 0;
    for (int i = 0; i < SAMPLE_COUNT; i++) {
        int raw = 0;
        adc_oneshot_read(adc_handle, channel, &raw);
        sum += raw;
    }
    return sum / SAMPLE_COUNT;
}

void adc_init(void)
{
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id  = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &adc_handle));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten    = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, VRX_CHANNEL, &chan_cfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, VRY_CHANNEL, &chan_cfg));
}

/* ═══════════════════════════════════════════════
 *  PWM
 * ═══════════════════════════════════════════════ */
static void ledc_ch_init(ledc_channel_t ch, gpio_num_t gpio)
{
    ledc_channel_config_t cfg = {
        .speed_mode = LEDC_MODE,
        .channel    = ch,
        .timer_sel  = LEDC_TIMER,
        .intr_type  = LEDC_INTR_DISABLE,
        .gpio_num   = gpio,
        .duty       = 0,
        .hpoint     = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&cfg));
}

void pwm_init(void)
{
    ledc_timer_config_t timer_cfg = {
        .speed_mode      = LEDC_MODE,
        .duty_resolution = PWM_RESOLUTION,
        .timer_num       = LEDC_TIMER,
        .freq_hz         = PWM_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));

    ledc_ch_init(LEDC_CH_L_FWD, MOTOR_L_RPWM);
    ledc_ch_init(LEDC_CH_L_REV, MOTOR_L_LPWM);
    ledc_ch_init(LEDC_CH_R_FWD, MOTOR_R_RPWM);
    ledc_ch_init(LEDC_CH_R_REV, MOTOR_R_LPWM);
}

/* ═══════════════════════════════════════════════
 *  MOTOR SET — private (1 channel)
 * ═══════════════════════════════════════════════ */
static void motor_set(ledc_channel_t ch_fwd, ledc_channel_t ch_rev, int speed)
{
    if (speed >  PWM_MAX) speed =  PWM_MAX;
    if (speed < -PWM_MAX) speed = -PWM_MAX;

    uint32_t duty = (uint32_t)abs(speed);
    if (duty > 0 && duty < PWM_MIN_MOVE) duty = PWM_MIN_MOVE;

    if (speed > 0) {
        ledc_set_duty(LEDC_MODE, ch_fwd, duty);
        ledc_set_duty(LEDC_MODE, ch_rev, 0);
    } else if (speed < 0) {
        ledc_set_duty(LEDC_MODE, ch_fwd, 0);
        ledc_set_duty(LEDC_MODE, ch_rev, duty);
    } else {
        ledc_set_duty(LEDC_MODE, ch_fwd, 0);
        ledc_set_duty(LEDC_MODE, ch_rev, 0);
    }

    ledc_update_duty(LEDC_MODE, ch_fwd);
    ledc_update_duty(LEDC_MODE, ch_rev);
}

/* ═══════════════════════════════════════════════
 *  MOTOR SET LR — PUBLIC, dùng chung 3 mode
 * ═══════════════════════════════════════════════ */
void motor_set_lr(int left_speed, int right_speed)
{
    motor_set(LEDC_CH_L_FWD, LEDC_CH_L_REV, left_speed);
    motor_set(LEDC_CH_R_FWD, LEDC_CH_R_REV, right_speed);
}

#define MOTOR_LEFT(spd)  motor_set(LEDC_CH_L_FWD, LEDC_CH_L_REV, (spd))
#define MOTOR_RIGHT(spd) motor_set(LEDC_CH_R_FWD, LEDC_CH_R_REV, (spd))

/* ═══════════════════════════════════════════════
 *  MAP CHUẨN HÓA
 * ═══════════════════════════════════════════════ */
static int map_normalized(int deflection, int max_pos, int max_neg)
{
    if (deflection > 0 && max_pos > 0) {
        int val = (deflection * PWM_MAX) / max_pos;
        return val > PWM_MAX ? PWM_MAX : val;
    } else if (deflection < 0 && max_neg > 0) {
        int val = (deflection * PWM_MAX) / max_neg;
        return val < -PWM_MAX ? -PWM_MAX : val;
    }
    return 0;
}

/* ═══════════════════════════════════════════════
 *  CALC SPEEDS
 * ═══════════════════════════════════════════════ */
static void calc_speeds(int dx, int dy,
                        int max_dx_pos, int max_dx_neg,
                        int max_dy_pos, int max_dy_neg,
                        int *left_out, int *right_out)
{
    int base = map_normalized(dx, max_dx_pos, max_dx_neg);
    int turn = map_normalized(dy, max_dy_pos, max_dy_neg);

    int left  = base + turn;
    int right = base - turn;

    if (left  >  PWM_MAX) left  =  PWM_MAX;
    if (left  < -PWM_MAX) left  = -PWM_MAX;
    if (right >  PWM_MAX) right =  PWM_MAX;
    if (right < -PWM_MAX) right = -PWM_MAX;

    if (base >= 0 && turn != 0) {
        if (left  < 0) left  = PWM_TURN_MIN;
        if (right < 0) right = PWM_TURN_MIN;
    } else if (base < 0 && turn != 0) {
        if (left  > 0) left  = -PWM_TURN_MIN;
        if (right > 0) right = -PWM_TURN_MIN;
    }

    *left_out  = left;
    *right_out = right;
}

/* ═══════════════════════════════════════════════
 *  EXECUTE DIRECTION
 * ═══════════════════════════════════════════════ */
static void execute_direction(int dx, int dy,
                               int max_dx_pos, int max_dx_neg,
                               int max_dy_pos, int max_dy_neg)
{
    int left_speed, right_speed;
    calc_speeds(dx, dy,
                max_dx_pos, max_dx_neg,
                max_dy_pos, max_dy_neg,
                &left_speed, &right_speed);

    motor_set_lr(left_speed, right_speed);

    direction_t dir;
    if (abs(dx) < DEAD_ZONE && abs(dy) < DEAD_ZONE) {
        dir = DIR_CENTER;
    } else if (abs(dy) >= abs(dx)) {
        dir = (dy < 0) ? DIR_LEFT : DIR_RIGHT;
    } else {
        dir = (dx > 0) ? DIR_FORWARD : DIR_BACKWARD;
    }

    ESP_LOGI(TAG, "Huong: %-6s | L=%4d R=%4d",
             dir_name[dir], left_speed, right_speed);
}

/* ═══════════════════════════════════════════════
 *  JOYSTICK TASK — block khi không ở MODE_JOYSTICK
 * ═══════════════════════════════════════════════ */
void joystick_task(void *arg)
{
    direction_t prev_dir  = DIR_CENTER;
    int         prev_left = 0, prev_right = 0;

    /* Đo tâm thực tế — chỉ làm 1 lần khi khởi động */
    int center_x = adc_read_avg(VRX_CHANNEL);
    int center_y = adc_read_avg(VRY_CHANNEL);
    ESP_LOGI(TAG, "Tam thuc te: VRX=%d VRY=%d", center_x, center_y);

    int max_dx_pos = 4095 - center_x;
    int max_dx_neg = center_x;
    int max_dy_pos = 4095 - center_y;
    int max_dy_neg = center_y;

    ESP_LOGI(TAG, "Max deflect: dx+=%d dx-=%d dy+=%d dy-=%d",
             max_dx_pos, max_dx_neg, max_dy_pos, max_dy_neg);

    for (;;) {
        /* ── BLOCK nếu không phải mode Joystick ── */
        EventBits_t bits = xEventGroupWaitBits(
            g_mode_event_group,
            BIT_MODE_JOYSTICK,
            pdFALSE,        /* không clear bit */
            pdTRUE,
            portMAX_DELAY);

        if (!(bits & BIT_MODE_JOYSTICK)) {
            /* Không phải mode của mình → dừng motor, vòng lại chờ */
            motor_set_lr(0, 0);
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        /* ── Đọc joystick và điều khiển ── */
        int vrx = adc_read_avg(VRX_CHANNEL);
        int vry = adc_read_avg(VRY_CHANNEL);

        int dx = vrx - center_x;
        int dy = vry - center_y;

        if (abs(dx) < DEAD_ZONE) dx = 0;
        if (abs(dy) < DEAD_ZONE) dy = 0;

        int left_speed, right_speed;
        calc_speeds(dx, dy,
                    max_dx_pos, max_dx_neg,
                    max_dy_pos, max_dy_neg,
                    &left_speed, &right_speed);

        direction_t dir;
        if (dx == 0 && dy == 0) {
            dir = DIR_CENTER;
        } else if (abs(dy) >= abs(dx)) {
            dir = (dy < 0) ? DIR_LEFT : DIR_RIGHT;
        } else {
            dir = (dx > 0) ? DIR_FORWARD : DIR_BACKWARD;
        }

        bool changed = (dir != prev_dir) ||
                       (abs(left_speed  - prev_left)  > 20) ||
                       (abs(right_speed - prev_right) > 20);

        if (changed) {
            execute_direction(dx, dy,
                              max_dx_pos, max_dx_neg,
                              max_dy_pos, max_dy_neg);
            prev_dir   = dir;
            prev_left  = left_speed;
            prev_right = right_speed;
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}