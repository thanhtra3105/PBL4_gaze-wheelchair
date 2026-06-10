#pragma once

#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"

/* ═══════════════════════════════════════════════
 *  ADC CHANNELS — joystick VRX / VRY
 * ═══════════════════════════════════════════════ */
#define VRX_CHANNEL     ADC_CHANNEL_2   /* GPIO34 */
#define VRY_CHANNEL     ADC_CHANNEL_3   /* GPIO35 */
#define SAMPLE_COUNT    5

/* ═══════════════════════════════════════════════
 *  LEDC (PWM)
 * ═══════════════════════════════════════════════ */
#define LEDC_MODE           LEDC_LOW_SPEED_MODE
#define LEDC_TIMER          LEDC_TIMER_0
#define PWM_RESOLUTION      LEDC_TIMER_12_BIT
#define PWM_FREQ_HZ         1000
#define PWM_MAX             2047
#define PWM_MIN_MOVE        150
#define PWM_TURN_MIN        0

/* ═══════════════════════════════════════════════
 *  MOTOR GPIO — L298N / BTS7960 hoặc tương đương
 * ═══════════════════════════════════════════════ */
#define MOTOR_L_RPWM    GPIO_NUM_7
#define MOTOR_L_LPWM    GPIO_NUM_6
#define MOTOR_R_RPWM    GPIO_NUM_1
#define MOTOR_R_LPWM    GPIO_NUM_0

/* LEDC channels */
#define LEDC_CH_L_FWD   LEDC_CHANNEL_0
#define LEDC_CH_L_REV   LEDC_CHANNEL_1
#define LEDC_CH_R_FWD   LEDC_CHANNEL_2
#define LEDC_CH_R_REV   LEDC_CHANNEL_3

/* ═══════════════════════════════════════════════
    *  DEAD ZONE
 * ═══════════════════════════════════════════════ */
#define DEAD_ZONE       300

/* ═══════════════════════════════════════════════
 *  DIRECTION ENUM
 * ═══════════════════════════════════════════════ */
typedef enum {
    DIR_CENTER = 0,
    DIR_FORWARD,
    DIR_BACKWARD,
    DIR_LEFT,
    DIR_RIGHT,
} direction_t;

/* ═══════════════════════════════════════════════
 *  PUBLIC API
 * ═══════════════════════════════════════════════ */
void adc_init(void);
void pwm_init(void);

/**
 * @brief Đặt tốc độ 2 bánh trực tiếp.
 *        Dùng chung cho cả 3 mode (MQTT, UART, Joystick).
 *        speed: -PWM_MAX … +PWM_MAX; dương = tiến, âm = lùi.
 */
void motor_set_lr(int left_speed, int right_speed);

/**
 * @brief Task điều khiển joystick — tự block khi không ở MODE_JOYSTICK.
 */
void joystick_task(void *arg);