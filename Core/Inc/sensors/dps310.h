/*
 * dps310.h
 *
 *  Created on: Sep 3, 2025
 *      Author: drrsm
 */

#ifndef INC_SENSORS_DPS310_H_
#define INC_SENSORS_DPS310_H_

#pragma once
#include "sensor_if.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DPS310_ADDR_A      0x77
#define DPS310_ADDR_B      0x76
#define DPS310_REG_PROD_ID 0x0D   // expect 0x10

typedef struct {
    I2C_HandleTypeDef *i2c;
    uint8_t addr7;
    // add calibration coeffs, config cache, etc.
} dps310_t;

void dps310_bind(sensor_if_t *iface, dps310_t *inst, I2C_HandleTypeDef *i2c, uint8_t addr7);

#ifdef __cplusplus
}
#endif

#endif /* INC_SENSORS_DPS310_H_ */
