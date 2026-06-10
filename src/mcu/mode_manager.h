#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"

#define MODE_BUTTON_GPIO              GPIO_NUM_4

/* Button kéo xuống GND */
#define MODE_BUTTON_PRESSED_LEVEL     0

#define MODE_BUTTON_POLL_MS           20
#define MODE_BUTTON_DEBOUNCE_MS       60

typedef enum {
    MODE_JOYSTICK = 0,
    MODE_UART     = 1,
} robot_mode_t;

#define BIT_MODE_JOYSTICK   (1 << 0)
#define BIT_MODE_UART       (1 << 1)

/* Giữ lại để sensor_monitor.c không lỗi compile */
typedef struct {
    uint32_t red;
    uint32_t ir;
    uint16_t dist_cm;
} sensor_data_t;

void mode_manager_init(void);
robot_mode_t mode_manager_get(void);
sensor_data_t mode_manager_get_sensor(void);

extern EventGroupHandle_t g_mode_event_group;