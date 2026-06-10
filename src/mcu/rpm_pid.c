#include "rpm_pid.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_timer.h"

#include "hall_rpm.h"
#include "joystick_motor.h"

static const char *TAG = "RPM_PID";

#define PID_TASK_PERIOD_MS      50

#define TARGET_FORWARD_RPM      200.0f

#define PID_PWM_MIN             0
#define PID_PWM_MAX             PWM_MAX

#define PID_PWM_BASE            1000

#define PID_KP                  2.0f
#define PID_KI                  0.0f
#define PID_KD                  0.0f

#define PID_INTEGRAL_LIMIT      300.0f

typedef struct {
    float kp;
    float ki;
    float kd;

    float integral;
    float prev_error;

    int pwm;
} pid_ctrl_t;

static pid_ctrl_t pid_left;
static pid_ctrl_t pid_right;

static volatile float target_left_rpm = 0.0f;
static volatile float target_right_rpm = 0.0f;
static volatile int pid_enable = 0;

static int clamp_int(int value, int min_value, int max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

static float clamp_float(float value, float min_value, float max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

static void pid_reset(pid_ctrl_t *pid)
{
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->pwm = 0;
}

static void pid_setup(pid_ctrl_t *pid)
{
    pid->kp = PID_KP;
    pid->ki = PID_KI;
    pid->kd = PID_KD;
    pid_reset(pid);
}

static int pid_compute(pid_ctrl_t *pid, float target_rpm, float current_rpm, float dt)
{
    if (target_rpm <= 0.0f) {
        pid_reset(pid);
        return 0;
    }

    float error = target_rpm - current_rpm;

    pid->integral += error * dt;
    pid->integral = clamp_float(pid->integral,
                                -PID_INTEGRAL_LIMIT,
                                 PID_INTEGRAL_LIMIT);

    float derivative = 0.0f;
    if (dt > 0.0f) {
        derivative = (error - pid->prev_error) / dt;
    }

    float output = pid->kp * error
                 + pid->ki * pid->integral
                 + pid->kd * derivative;

    pid->prev_error = error;

    
    int pwm = PID_PWM_BASE + (int)output;

    pwm = clamp_int(pwm, PID_PWM_MIN, PID_PWM_MAX);
    pid->pwm = pwm;

    return pwm;
}

void rpm_pid_init(void)
{
    pid_setup(&pid_left);
    pid_setup(&pid_right);

    target_left_rpm = 0.0f;
    target_right_rpm = 0.0f;
    pid_enable = 0;

    ESP_LOGI(TAG, "RPM PID init done");
}

void rpm_pid_set_target(float left_rpm, float right_rpm)
{
    target_left_rpm = left_rpm;
    target_right_rpm = right_rpm;
    pid_enable = 1;

    pid_reset(&pid_left);
    pid_reset(&pid_right);

    ESP_LOGI(TAG, "PID target set: L=%.2f RPM, R=%.2f RPM",
             left_rpm, right_rpm);
}

void rpm_pid_stop(void)
{
    pid_enable = 0;

    target_left_rpm = 0.0f;
    target_right_rpm = 0.0f;

    pid_reset(&pid_left);
    pid_reset(&pid_right);

    motor_set_lr(0, 0);

    ESP_LOGI(TAG, "PID stop");
}

float rpm_pid_get_target_left(void)
{
    return target_left_rpm;
}

float rpm_pid_get_target_right(void)
{
    return target_right_rpm;
}

void rpm_pid_task(void *arg)
{
    int64_t prev_time_us = esp_timer_get_time();

    while (1) {
        int64_t now_us = esp_timer_get_time();
        float dt = (now_us - prev_time_us) / 1000000.0f;
        prev_time_us = now_us;

        if (!pid_enable) {
            // motor_set_lr(0, 0);
            vTaskDelay(pdMS_TO_TICKS(PID_TASK_PERIOD_MS));
            continue;
        }

        float left_rpm = hall_get_left_rpm();
        float right_rpm = hall_get_right_rpm();

        float left_target = target_left_rpm;
        float right_target = target_right_rpm;

        int pwm_left = pid_compute(&pid_left, left_target, left_rpm, dt);
        int pwm_right = pid_compute(&pid_right, right_target, right_rpm, dt);

        motor_set_lr(pwm_left, pwm_right);

        ESP_LOGI(TAG,
                 "PID | Tar L=%.1f R=%.1f | RPM L=%.1f R=%.1f | PWM L=%d R=%d",
                 left_target, right_target,
                 left_rpm, right_rpm,
                 pwm_left, pwm_right);

        vTaskDelay(pdMS_TO_TICKS(PID_TASK_PERIOD_MS));
    }
}