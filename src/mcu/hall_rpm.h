#ifndef HALL_RPM_H
#define HALL_RPM_H

#include "driver/gpio.h"

/* Chân cảm biến Hall */
#define HALL_LEFT_PIN       GPIO_NUM_5
#define HALL_RIGHT_PIN      GPIO_NUM_19

#define HALL_MAGNET_COUNT          1
#define HALL_SAMPLE_PULSE          5
#define HALL_TASK_PERIOD_MS        10
#define HALL_RPM_TIMEOUT_MS        1000
#define HALL_MIN_PULSE_INTERVAL_US 30000

void hall_rpm_init(void);
void hall_rpm_task(void *pvParameters);

float hall_get_left_rpm(void);
float hall_get_right_rpm(void);

#endif