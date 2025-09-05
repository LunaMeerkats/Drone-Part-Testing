/*
 * icm42688.h
 *
 *  Created on: Sep 3, 2025
 *      Author: drrsm
 */

#ifndef INC_SENSORS_ICM42688_H_
#define INC_SENSORS_ICM42688_H_


#pragma once
#include "sensor_if.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ICM_ADDR_7B      0x68  // or 0x69 depending
#define ICM_WHO_AM_I_REG 0x75
#define ICM_WHO_AM_I_REG_SIZE 8u
#define ICM_WHO_AM_I_DEFAULT 0x47

#define ICM_SOFT_RESET_REG 0x11
#define ICM_SOFT_RESET_POS 0
#define ICM_SOFT_RESET_Msk (1u << ICM_SOFT_RESET_POS)


#define ICM_POWER_MGMT0_REG 0x4E

#define ICM_POWER_MGMT0_TEMP_DISABLE_POS 5u
#define ICM_POWER_MGMT0_TEMP_DISABLE_Msk (1u << ICM_POWER_MGMT0_TEMP_DISABLE_POS)

#define ICM_POWER_MGMT0_IDLE_POS 4u
#define ICM_POWER_MGMT0_IDLE_Msk (1u << ICM_POWER_MGMT0_IDLE_POS)
#define ICM_POWER_MGMT0_GYRO_MODE_POS 2u
#define ICM_POWER_MGMT0_GYRO_MODE_Msk (3u << ICM_POWER_MGMT0_GYRO_MODE_POS)
#define ICM_POWER_MGMT0_GYRO_MODE_OFF            (0u   << ICM_POWER_MGMT0_GYRO_MODE_POS)
#define ICM_POWER_MGMT0_GYRO_MODE_STANDBY        (1u   << ICM_POWER_MGMT0_GYRO_MODE_POS)
#define ICM_POWER_MGMT0_GYRO_MODE_LN             (2u   << ICM_POWER_MGMT0_GYRO_MODE_POS)
#define ICM_POWER_MGMT0_GYRO_MODE_LP             (3u   << ICM_POWER_MGMT0_GYRO_MODE_POS)

#define ICM_POWER_MGMT0_ACCEL_MODE_POS 0u
#define ICM_POWER_MGMT0_ACCEL_MODE_Msk (3u << ICM_POWER_MGMT0_ACCEL_MODE_POS)
#define ICM_POWER_MGMT0_ACCEL_MODE_OFF           (0u   << ICM_POWER_MGMT0_ACCEL_MODE_POS)
#define ICM_POWER_MGMT0_ACCEL_MODE_STANDBY       (1u   << ICM_POWER_MGMT0_ACCEL_MODE_POS)
#define ICM_POWER_MGMT0_ACCEL_MODE_LN            (2u   << ICM_POWER_MGMT0_ACCEL_MODE_POS)
#define ICM_POWER_MGMT0_ACCEL_MODE_LP            (3u   << ICM_POWER_MGMT0_ACCEL_MODE_POS)

// Only bits [5:0] are writable; [7:6] are reserved → preserve them
#define ICM_POWER_MGMT0_WRITABLE_Msk ( \
		ICM_POWER_MGMT0_TEMP_DISABLE_Msk | \
		ICM_POWER_MGMT0_IDLE_Msk     | \
		ICM_POWER_MGMT0_GYRO_MODE_Msk| \
		ICM_POWER_MGMT0_ACCEL_MODE_Msk )


#define ICM_GYRO_CONFIG0_REG 0x4F

#define ICM_GYRO_CONFIG0_ODR_POS 0u
#define ICM_GYRO_CONFIG0_ODR_Msk (15u << ICM_GYRO_CONFIG0_ODR_POS)

#define ICM_GYRO_CONFIG0_FS_POS 5u
#define ICM_GYRO_CONFIG0_FS_Msk (7u << ICM_GYRO_CONFIG0_FS_POS)

#define ICM_GYRO_CONFIG0_WRITABLE_Msk (ICM_GYRO_CONFIG0_ODR_Msk | ICM_GYRO_CONFIG0_FS_Msk)


#define ICM_ACCEL_CONFIG0_REG 0x50

#define ICM_ACCEL_CONFIG0_ODR_POS 0u
#define ICM_ACCEL_CONFIG0_ODR_Msk (15u << ICM_ACCEL_CONFIG0_ODR_POS)

#define ICM_ACCEL_CONFIG0_FS_POS 5u
#define ICM_ACCEL_CONFIG0_FS_Msk (7u << ICM_ACCEL_CONFIG0_FS_POS)

#define ICM_ACCEL_CONFIG0_WRITABLE_Msk (ICM_ACCEL_CONFIG0_ODR_Msk | ICM_ACCEL_CONFIG0_FS_Msk)


#define ICM_TEMP_MSB                    0x1D
#define ICM_TEMP_LSB                    0x1E
// Convert to degrees C (typical scale/offset)
#define ICM_TEMP_SENS_LSB_PER_C  132.48f
#define ICM_TEMP_OFFSET_C        25.0f


#define ICM_ACCEL_X_MSB                 0x1F
#define ICM_ACCEL_X_LSB                 0x20
#define ICM_ACCEL_Y_MSB                 0x21
#define ICM_ACCEL_Y_LSB                 0x22
#define ICM_ACCEL_Z_MSB                 0x23
#define ICM_ACCEL_Z_LSB                 0x24


#define ICM_GYRO_X_MSB                  0x25
#define ICM_GYRO_X_LSB                  0x26
#define ICM_GYRO_Y_MSB                  0x27
#define ICM_GYRO_Y_LSB                  0x28
#define ICM_GYRO_Z_MSB                  0x29
#define ICM_GYRO_Z_LSB                  0x2A

typedef enum {
	ICM_OK = 0,
	ICM_ERR_BAD_SELF,
	ICM_ERR_I2C,
	ICM_ERR_TIMEOUT,
	ICM_ERR_WHOAMI_MISMATCH,
	ICM_ERR_NOT_INITIALIZED,
	ICM_ERR_INCORRECT_BUFFER_SIZE
} icm_err_t;

static inline const char* icm_err_str(icm_err_t e) {
	switch (e) {
	case ICM_OK:                 		return "OK";
	case ICM_ERR_BAD_SELF:       		return "Bad self/impl pointer";
	case ICM_ERR_I2C:            		return "I2C transfer failed";
	case ICM_ERR_TIMEOUT:        		return "I2C timeout";
	case ICM_ERR_WHOAMI_MISMATCH:		return "WHO_AM_I mismatch";
	case ICM_ERR_NOT_INITIALIZED:		return "Driver not initialized";
	case ICM_ERR_INCORRECT_BUFFER_SIZE: return "Incorrect buffer size used to store data";
	default:                     		return "Unknown";
	}
}

typedef struct {
	I2C_HandleTypeDef *i2c;
	uint8_t addr7;

	// diagnostics
	icm_err_t  last_err;
	uint32_t   last_hal_err;
	uint8_t    last_whoami;

	// latest measurements (raw register values)
	int16_t    temp_raw;      // signed
	int16_t    accel_raw[3];  // X,Y,Z
	int16_t    gyro_raw[3];   // X,Y,Z

	// latest measurements (engineering units)
	float      last_temp_c;
	float      accel_g[3];    // g
	float      gyro_dps[3];   // deg/s

	// scaling (set these in init based on FS config)
	float      accel_lsb_per_g;
	float      gyro_lsb_per_dps;

	// data rate
	float 	   gyro_odr;
	float 	   accel_odr;
} icm42688_t;


void icm42688_bind(sensor_if_t *iface, icm42688_t *inst, I2C_HandleTypeDef *i2c, uint8_t addr7);

#ifdef __cplusplus
}
#endif



#endif /* INC_SENSORS_ICM42688_H_ */
