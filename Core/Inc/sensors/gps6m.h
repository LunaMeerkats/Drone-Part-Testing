/*
 * gps6m.h
 *
 *  Created on: Sep 3, 2025
 *      Author: drrsm
 */

#ifndef INC_SENSORS_GPS6M_H_
#define INC_SENSORS_GPS6M_H_


#pragma once
#include "sensor_if.h"
#include "stm32h7xx_hal.h"   // or your device header

typedef struct {
    UART_HandleTypeDef *uart;
} gps6m_t;

void gps6m_bind(sensor_if_t *iface, gps6m_t *inst, UART_HandleTypeDef *uart);



#endif /* INC_SENSORS_GPS6M_H_ */
