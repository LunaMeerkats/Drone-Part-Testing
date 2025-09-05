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

#define DPS_I2C_ADDRESS 0x77
#define DPS_ID_REG 0x0D
#define DPS_WHOAMI_EXPECT 0x10

//Stored in twos compliment.
#define DPS_PRESSURE_BYTE3_Reg 0x00
#define DPS_PRESSURE_BYTE2_Reg 0x01
#define DPS_PRESSURE_BYTE1_Reg 0x02

//Stored in twos compliment.
#define DPS_TEMPERATURE_BYTE3_Reg 0x03
#define DPS_TEMPERATURE_BYTE2_Reg 0x04
#define DPS_TEMPERATURE_BYTE1_Reg 0x05

#define DPS_PRESSURE_CONFIGURATION_Reg 0x06
#define DPS_PRESSURE_CONFIGURATION_POR_Pos 0u
#define DPS_PRESSURE_CONFIGURATION_POR_Msk (7u << DPS_PRESSURE_CONFIGURATION_POR_Pos)
#define DPS_PRESSURE_CONFIGURATION_MR_Pos 4u
#define DPS_PRESSURE_CONFIGURATION_MR_Msk (3u << DPS_PRESSURE_CONFIGURATION_MR_Pos)
#define DPS_PRESSURE_CONFIGURATION_Msk (DPS_PRESSURE_CONFIGURATION_POR_Msk | DPS_PRESSURE_CONFIGURATION_MR_Msk)

#define DPS_TEMPERATURE_CONFIGURATION_Reg 0x07
#define DPS_TEMPERATURE_CONFIGURATION_TOS_Pos 0u
#define DPS_TEMPERATURE_CONFIGURATION_TOS_Msk (7u << DPS_TEMPERATURE_CONFIGURATION_TOS_Pos)
#define DPS_TEMPERATURE_CONFIGURATION_MR_Pos 4u
#define DPS_TEMPERATURE_CONFIGURATION_MR_Msk (7u << DPS_TEMPERATURE_CONFIGURATION_MR_Pos)
#define DPS_TEMPERATURE_CONFIGURATION_SENSE_Pos 7u
#define DPS_TEMPERATURE_CONFIGURATION_SENSE_Msk (1u << DPS_TEMPERATURE_CONFIGURATION_SENSE_Pos)
#define DPS_TEMPERATURE_CONFIGURATION_Msk (	DPS_TEMPERATURE_CONFIGURATION_SENSE_Msk | \
											DPS_PRESSURE_CONFIGURATION_POR_Msk | \
											DPS_PRESSURE_CONFIGURATION_MR_Msk)

#define DPS_STATUS_Reg 0x08
#define DPS_STATUS_MEAS_CTRL_Pos 0u
#define DPS_STATUS_MEAS_CTRL_Msk (3u << DPS_STATUS_MEAS_CTRL_Pos)
#define DPS_STATUS_PRS_RDY_Pos 4u
#define DPS_STATUS_PRS_RDY_Msk (1u << DPS_STATUS_PRS_RDY_Pos)
#define DPS_STATUS_TMP_RDY_Pos 5u
#define DPS_STATUS_TMP_RDY_Msk (1u << DPS_STATUS_TMP_RDY_Pos)
#define DPS_STATUS_SENSOR_RDY_Pos 6u
#define DPS_STATUS_SENSOR_RDY_Msk (1u << DPS_STATUS_SENSOR_RDY_Pos)
#define DPS_STATUS_COEF_RDY_Pos 7u
#define DPS_STATUS_COEF_RDY_Msk (1u << DPS_STATUS_COEF_RDY_Pos)
#define DPS_STATUS_READY_Msk 	(DPS_STATUS_PRS_RDY_Msk | \
								DPS_STATUS_TMP_RDY_Msk | \
								DPS_STATUS_SENSOR_RDY_Msk)

#define DPS_SOFT_RESET_Reg 0x0C
#define DPS_SOFT_RESET_Pos 0u
#define DPS_SOFT_RESET_Msk (15u << DPS_SOFT_RESET_Pos)

typedef enum {
	DPS_OK = 0,
	DPS_ERR_BAD_SELF,
	DPS_ERR_I2C,
	DPS_ERR_TIMEOUT,
	DPS_ERR_WHOAMI_MISMATCH,
	DPS_ERR_NOT_INITIALIZED,
	DPS_ERR_INCORRECT_BUFFER_SIZE
} dps_err_t;

static inline const char* DPS_err_str(dps_err_t e) {
	switch (e) {
	case DPS_OK:                 		return "OK";
	case DPS_ERR_BAD_SELF:       		return "Bad self/impl pointer";
	case DPS_ERR_I2C:            		return "I2C transfer failed";
	case DPS_ERR_TIMEOUT:        		return "I2C timeout";
	case DPS_ERR_WHOAMI_MISMATCH:		return "WHO_AM_I mismatch";
	case DPS_ERR_NOT_INITIALIZED:		return "Driver not initialized";
	case DPS_ERR_INCORRECT_BUFFER_SIZE: return "Incorrect buffer size used to store data";
	default:                     		return "Unknown";
	}
}

typedef struct {
	I2C_HandleTypeDef *i2c;
	uint8_t addr7;

	// diagnostics
	dps_err_t  last_err;
	uint32_t   last_hal_err;
	uint8_t    last_whoami;

	// latest measurements (raw register values)
	float pressure_raw;
	float temperature_raw;
	float pressure;
	float temperature;

} dps310_t;

void dps310_bind(sensor_if_t *iface, dps310_t *inst, I2C_HandleTypeDef *i2c, uint8_t addr7);

#ifdef __cplusplus
}
#endif

#endif /* INC_SENSORS_DPS310_H_ */
