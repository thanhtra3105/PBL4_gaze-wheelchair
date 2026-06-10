#pragma once

#include "driver/uart.h"
#include "driver/gpio.h"
#include "rpm_pid.h"

#define JETSON_UART_PORT    UART_NUM_1      
// #define JETSON_UART_TX      GPIO_NUM_21    
#define JETSON_UART_RX      GPIO_NUM_8      
#define JETSON_UART_BAUD    115200

/**
 * @brief
 */
void uart_control_init(void);

/**
 * @brief 
 *       
 */
void uart_control_task(void *arg);
   