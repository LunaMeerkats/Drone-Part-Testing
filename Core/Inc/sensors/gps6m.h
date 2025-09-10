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

// gps6m.h
typedef struct {
    UART_HandleTypeDef *uart;

    // parsed fields
    uint8_t  has_fix;
    double   lat_deg, lon_deg;
    float    alt_m, speed_mps, course_deg;
    float    hdop;
    uint8_t  sats_used;
    uint32_t utc_hms;

    // housekeeping
    uint32_t last_update_ms;  // last time we read ANY NMEA line
    uint32_t last_fix_ms;     // last time we confirmed fix via GGA/RMC
} gps6m_t;



void gps6m_bind(sensor_if_t *iface, gps6m_t *inst, UART_HandleTypeDef *uart);



#endif /* INC_SENSORS_GPS6M_H_ */
