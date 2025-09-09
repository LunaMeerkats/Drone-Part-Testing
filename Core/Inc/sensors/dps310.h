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

//Stored in twos compliment.
/* ===== DPS310 Coefficient registers (0x10–0x21) ===== */

/* --- c0 (12-bit, signed) --- */
#define DPS_COEF_C0_MSB_Reg              0x10    /* c0[11:4]   (8 bits) */
#define DPS_COEF_C0_LSN_Reg              0x11    /* c0[3:0] in bits[7:4] */
#define DPS_COEF_C0_LSN_Pos              4u
#define DPS_COEF_C0_LSN_Msk              (0xFu << DPS_COEF_C0_LSN_Pos)
#define DPS_COEF_C0_BITS                 12u

/* --- c1 (12-bit, signed) --- */
#define DPS_COEF_C1_MSN_Reg              0x11    /* c1[11:8] in bits[3:0] */
#define DPS_COEF_C1_MSN_Pos              0u
#define DPS_COEF_C1_MSN_Msk              (0xFu << DPS_COEF_C1_MSN_Pos)
#define DPS_COEF_C1_LSB_Reg              0x12    /* c1[7:0] */
#define DPS_COEF_C1_BITS                 12u

/* --- c00 (20-bit, signed) --- */
#define DPS_COEF_C00_B19_B12_Reg         0x13    /* c00[19:12] */
#define DPS_COEF_C00_B11_B4_Reg          0x14    /* c00[11:4]  */
#define DPS_COEF_C00_B3_B0_Reg           0x15    /* c00[3:0] in bits[7:4] */
#define DPS_COEF_C00_B3_B0_Pos           4u
#define DPS_COEF_C00_B3_B0_Msk           (0xFu << DPS_COEF_C00_B3_B0_Pos)
#define DPS_COEF_C00_BITS                20u

/* --- c10 (20-bit, signed) --- */
#define DPS_COEF_C10_B19_B16_Reg         0x15    /* c10[19:16] in bits[3:0] */
#define DPS_COEF_C10_B19_B16_Pos         0u
#define DPS_COEF_C10_B19_B16_Msk         (0xFu << DPS_COEF_C10_B19_B16_Pos)
#define DPS_COEF_C10_B15_B8_Reg          0x16    /* c10[15:8]  */
#define DPS_COEF_C10_B7_B0_Reg           0x17    /* c10[7:0]   */
#define DPS_COEF_C10_BITS                20u

/* --- c01 (16-bit, signed) --- */
#define DPS_COEF_C01_MSB_Reg             0x18    /* c01[15:8] */
#define DPS_COEF_C01_LSB_Reg             0x19    /* c01[7:0]  */
#define DPS_COEF_C01_BITS                16u

/* --- c11 (16-bit, signed) --- */
#define DPS_COEF_C11_MSB_Reg             0x1A
#define DPS_COEF_C11_LSB_Reg             0x1B
#define DPS_COEF_C11_BITS                16u

/* --- c20 (16-bit, signed) --- */
#define DPS_COEF_C20_MSB_Reg             0x1C
#define DPS_COEF_C20_LSB_Reg             0x1D
#define DPS_COEF_C20_BITS                16u

/* --- c21 (16-bit, signed) --- */
#define DPS_COEF_C21_MSB_Reg             0x1E
#define DPS_COEF_C21_LSB_Reg             0x1F
#define DPS_COEF_C21_BITS                16u

/* --- c30 (16-bit, signed) --- */
#define DPS_COEF_C30_MSB_Reg             0x20
#define DPS_COEF_C30_LSB_Reg             0x21
#define DPS_COEF_C30_BITS                16u

/* Block range */
#define DPS_COEF_BLOCK_START             0x10
#define DPS_COEF_BLOCK_END               0x21
#define DPS_COEF_BLOCK_LEN               (DPS_COEF_BLOCK_END - DPS_COEF_BLOCK_START + 1)


#define DPS_CFG_REG                0x09
#define DPS_CFG_SPI_MODE_Pos       0u
#define DPS_CFG_FIFO_EN_Pos        1u
#define DPS_CFG_P_SHIFT_Pos        2u
#define DPS_CFG_T_SHIFT_Pos        3u
#define DPS_CFG_INT_PRS_Pos        4u
#define DPS_CFG_INT_TMP_Pos        5u
#define DPS_CFG_INT_FIFO_Pos       6u
#define DPS_CFG_INT_HL_Pos         7u

#define DPS_CFG_SPI_MODE_Msk       (1u << DPS_CFG_SPI_MODE_Pos)
#define DPS_CFG_FIFO_EN_Msk        (1u << DPS_CFG_FIFO_EN_Pos)
#define DPS_CFG_P_SHIFT_Msk        (1u << DPS_CFG_P_SHIFT_Pos)
#define DPS_CFG_T_SHIFT_Msk        (1u << DPS_CFG_T_SHIFT_Pos)
#define DPS_CFG_INT_PRS_Msk        (1u << DPS_CFG_INT_PRS_Pos)
#define DPS_CFG_INT_TMP_Msk        (1u << DPS_CFG_INT_TMP_Pos)
#define DPS_CFG_INT_FIFO_Msk       (1u << DPS_CFG_INT_FIFO_Pos)
#define DPS_CFG_INT_HL_Msk         (1u << DPS_CFG_INT_HL_Pos)

/* Helper: OSR codes 0..3 = ≤8×, 4..7 = >8× (needs right-shift in data regs) */
#define DPS_OSR_NEEDS_SHIFT(osr_code)   (((osr_code) & 0x0Fu) >= 4u)


#define DPS_PRESSURE_CONFIGURATION_Reg 0x06
#define DPS_PRESSURE_CONFIGURATION_POR_Pos 0u
#define DPS_PRESSURE_CONFIGURATION_POR_Msk (15u << DPS_PRESSURE_CONFIGURATION_POR_Pos)
#define DPS_PRESSURE_CONFIGURATION_MR_Pos 4u
#define DPS_PRESSURE_CONFIGURATION_MR_Msk (3u << DPS_PRESSURE_CONFIGURATION_MR_Pos)
#define DPS_PRESSURE_CONFIGURATION_Msk (DPS_PRESSURE_CONFIGURATION_POR_Msk | DPS_PRESSURE_CONFIGURATION_MR_Msk)

#define DPS_TEMPERATURE_CONFIGURATION_Reg 0x07
#define DPS_TEMPERATURE_CONFIGURATION_TOS_Pos 0u
#define DPS_TEMPERATURE_CONFIGURATION_TOS_Msk (15u << DPS_TEMPERATURE_CONFIGURATION_TOS_Pos)
#define DPS_TEMPERATURE_CONFIGURATION_MR_Pos 4u
#define DPS_TEMPERATURE_CONFIGURATION_MR_Msk (7u << DPS_TEMPERATURE_CONFIGURATION_MR_Pos)
#define DPS_TEMPERATURE_CONFIGURATION_SENSE_Pos 7u
#define DPS_TEMPERATURE_CONFIGURATION_SENSE_Msk (1u << DPS_TEMPERATURE_CONFIGURATION_SENSE_Pos)
#define DPS_TEMPERATURE_CONFIGURATION_Msk ( \
    DPS_TEMPERATURE_CONFIGURATION_SENSE_Msk | \
    DPS_TEMPERATURE_CONFIGURATION_TOS_Msk   | \
    DPS_TEMPERATURE_CONFIGURATION_MR_Msk )

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
#define DPS_STATUS_READY_Msk 	(DPS_STATUS_SENSOR_RDY_Msk | \
								DPS_STATUS_COEF_RDY_Msk)

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

	/* --- calibration coefficients --- */
	int16_t  c0, c1;
	int32_t  c00, c10;
	int16_t  c01, c11, c20, c21, c30;

	/* --- cached config/scales --- */
	uint8_t  p_osr_code;   // pressure OSR code (0..7)
	uint8_t  t_osr_code;   // temperature OSR code (0..7)
	float    kP;           // pressure scale factor from OSR
	float    kT;           // temperature scale factor from OSR

	// latest measurements (raw register values)
	float pressure_raw;
	float temperature_raw;
	float pressure;
	float temperature;
	float altitude;

} dps310_t;

void dps310_bind(sensor_if_t *iface, dps310_t *inst, I2C_HandleTypeDef *i2c, uint8_t addr7);

#ifdef __cplusplus
}
#endif

#endif /* INC_SENSORS_DPS310_H_ */
