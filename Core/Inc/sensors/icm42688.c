/*
 * sensors_if.h
 *
 *  Created on: Sep 4, 2025
 *      Author: Daniel Smith
 *
 * icm42688.c
 * IMU Sensor
 *
 */

#include "icm42688.h"
#include <stdio.h>

static HAL_StatusTypeDef icm_probe(sensor_if_t *self);
static HAL_StatusTypeDef icm_init(sensor_if_t *self);
static HAL_StatusTypeDef icm_read(sensor_if_t *self);
static HAL_StatusTypeDef icm_whoami(sensor_if_t *self, uint8_t *out);
void icm42688_bind(sensor_if_t *iface, icm42688_t *inst, I2C_HandleTypeDef *i2c, uint8_t addr7);

static HAL_StatusTypeDef ICM_POWER_mgmt0_config(icm42688_t *self, uint8_t gyro_mode, uint8_t accel_mode, uint8_t temp_disable, uint8_t idle);

static HAL_StatusTypeDef icm_read_temp_raw(icm42688_t *self);
static HAL_StatusTypeDef icm_read_temp_c(icm42688_t *self);

static HAL_StatusTypeDef rd(I2C_HandleTypeDef *i2c, uint8_t a7, uint8_t reg, uint8_t *buf, uint16_t bufferSize);
static HAL_StatusTypeDef wr1(I2C_HandleTypeDef *i2c, uint8_t a7, uint8_t reg, uint8_t val);
static inline uint8_t clip2(uint8_t x);
static icm42688_t* getSelfStatePointer(sensor_if_t *self);

/* ---------- vtable ops ---------- */
static HAL_StatusTypeDef icm_probe(sensor_if_t *self) {
	icm42688_t *s = getSelfStatePointer(self);
	if (!s) return HAL_ERROR;

	//Get the id.
	uint8_t id = 0;
	HAL_StatusTypeDef st = icm_whoami(self, &id);
	if (st != HAL_OK) {
		s->last_hal_err = HAL_I2C_GetError(s->i2c);
		s->last_err = (st == HAL_TIMEOUT) ? ICM_ERR_TIMEOUT : ICM_ERR_I2C;
		return st;
	}

	//Check that the id matches the documentation.
	if (id != ICM_WHO_AM_I_DEFAULT) {
		s->last_err = ICM_ERR_WHOAMI_MISMATCH;
		s->last_whoami = id;
		return HAL_ERROR;
	}

	s->last_err = ICM_OK;
	return HAL_OK;
}

static HAL_StatusTypeDef icm_init(sensor_if_t *self) {
	icm42688_t *s = getSelfStatePointer(self);
	if (!s) return HAL_ERROR;

	return ICM_POWER_mgmt0_config(s, /*gyro_mode*/2, /*accel_mode*/2, /*temp_disable*/0, /*idle*/0);
}

static HAL_StatusTypeDef icm_read(sensor_if_t *self) {
	icm42688_t *s = getSelfStatePointer(self);
	if (!s) return HAL_ERROR;

	HAL_StatusTypeDef st = icm_read_temp_c(s);
	if (st != HAL_OK) return st;

	return HAL_OK;
}

static HAL_StatusTypeDef icm_whoami(sensor_if_t *self, uint8_t *out) {
	icm42688_t *s = getSelfStatePointer(self);
	if (!s) return HAL_ERROR;

	return rd(s->i2c, s->addr7, ICM_WHO_AM_I_REG, out, 1);
}

/* ---------- bind ---------- */
static const sensor_if_vtbl_t ICM_VTBL = {
		.probe = icm_probe,
		.init  = icm_init,
		.read  = icm_read,
		.whoami= icm_whoami
};

void icm42688_bind(sensor_if_t *iface, icm42688_t *inst, I2C_HandleTypeDef *i2c, uint8_t addr7) {
	inst->i2c  = i2c;
	inst->addr7 = addr7;
	iface->vTable    = &ICM_VTBL;
	iface->state = inst;
}

/* ---------- Configuration ---------- */

// RMW: preserve reserved [7:6], only update [5:0]
static HAL_StatusTypeDef ICM_POWER_mgmt0_config(icm42688_t *self,
		uint8_t gyro_mode,
		uint8_t accel_mode,
		uint8_t temp_disable,
		uint8_t idle)
{
	uint8_t v;
	HAL_StatusTypeDef st = HAL_I2C_Mem_Read(self->i2c, (self->addr7 << 1),
			ICM_POWER_MGMT0_REG, I2C_MEMADD_SIZE_8BIT,
			&v, 1, 200);
	if (st != HAL_OK) {
		self->last_hal_err = HAL_I2C_GetError(self->i2c);
		self->last_err = (st == HAL_TIMEOUT) ? ICM_ERR_TIMEOUT : ICM_ERR_I2C;
		return st;
	}

	// Compose only the fields we intend to change
	uint8_t set =
			((clip2(gyro_mode)  << ICM_POWER_MGMT0_GYRO_MODE_POS)  & ICM_POWER_MGMT0_GYRO_MODE_Msk) |
			((clip2(accel_mode) << ICM_POWER_MGMT0_ACCEL_MODE_POS) & ICM_POWER_MGMT0_ACCEL_MODE_Msk) |
			(temp_disable ? ICM_POWER_MGMT0_TEMP_DISABLE_POS : 0u) |
			(idle         ? ICM_POWER_MGMT0_IDLE_Msk     : 0u);

	uint8_t new_v = (uint8_t)((v & ~ICM_POWER_MGMT0_WRITABLE_Msk) | (set & ICM_POWER_MGMT0_WRITABLE_Msk));

	if (new_v == v) {        // no change → skip bus write
		self->last_err = ICM_OK;
		return HAL_OK;
	}

	st = HAL_I2C_Mem_Write(self->i2c, (self->addr7 << 1),
			ICM_POWER_MGMT0_REG, I2C_MEMADD_SIZE_8BIT,
			&new_v, 1, 200);
	if (st != HAL_OK) {
		self->last_hal_err = HAL_I2C_GetError(self->i2c);
		self->last_err = (st == HAL_TIMEOUT) ? ICM_ERR_TIMEOUT : ICM_ERR_I2C;
		return st;
	}

	self->last_err = ICM_OK;
	return HAL_OK;
}

static HAL_StatusTypeDef imc_gyro_config0(icm42688_t *self, uint8_t gyro_fs_sel, uint8_t gyro_odr){
	uint8_t v;
	HAL_StatusTypeDef st = HAL_I2C_Mem_Read(self->i2c, (self->addr7 << 1),
			ICM_GYRO_CONFIG0_REG, I2C_MEMADD_SIZE_8BIT,
			&v, 1, 200);
	if (st != HAL_OK) {
		self->last_hal_err = HAL_I2C_GetError(self->i2c);
		self->last_err = (st == HAL_TIMEOUT) ? ICM_ERR_TIMEOUT : ICM_ERR_I2C;
		return st;
	}

	// Compose only the fields we intend to change
	uint8_t set =
			((clip2(gyro_mode)  << ICM_POWER_MGMT0_GYRO_MODE_POS)  & ICM_POWER_MGMT0_GYRO_MODE_Msk) |
			((clip2(accel_mode) << ICM_POWER_MGMT0_ACCEL_MODE_POS) & ICM_POWER_MGMT0_ACCEL_MODE_Msk) |
			(temp_disable ? ICM_POWER_MGMT0_TEMP_DISABLE_POS : 0u) |
			(idle         ? ICM_POWER_MGMT0_IDLE_Msk     : 0u);

	uint8_t new_v = (uint8_t)((v & ~ICM_POWER_MGMT0_WRITABLE_Msk) | (set & ICM_POWER_MGMT0_WRITABLE_Msk));

	if (new_v == v) {        // no change → skip bus write
		self->last_err = ICM_OK;
		return HAL_OK;
	}
}


/* ---------- Read Values ---------- */
static HAL_StatusTypeDef icm_read_temp_raw(icm42688_t *self) {
	uint8_t raw[2]; // [0]=LSB, [1]=MSB per datasheet
	HAL_StatusTypeDef st = rd(self->i2c, self->addr7, ICM_TEMP_LSB, raw, 2);
	if (st != HAL_OK) {
		self->last_hal_err = HAL_I2C_GetError(self->i2c);
		self->last_err = (st == HAL_TIMEOUT) ? ICM_ERR_TIMEOUT : ICM_ERR_I2C;
		return st;
	}

	self->temp_raw = (int16_t)((raw[1] << 8) | raw[0]);  // MSB<<8 | LSB
	self->last_err = ICM_OK;
	return HAL_OK;
}

static HAL_StatusTypeDef icm_read_temp_c(icm42688_t *self) {

	HAL_StatusTypeDef st = icm_read_temp_raw(self);

	if (st != HAL_OK) return st;
	self->last_temp_c = (self->temp_raw / ICM_TEMP_SENS_LSB_PER_C) + ICM_TEMP_OFFSET_C;
	return HAL_OK;
}

static HAL_StatusTypeDef icm_gyro_config0(icm42688_t *self, uint8_t gyro_fs_sel, uint8_t gyro_odr)
{
    if (!self) return HAL_ERROR;

    uint8_t v;
    HAL_StatusTypeDef st = HAL_I2C_Mem_Read(self->i2c, (self->addr7 << 1),
                                            ICM_GYRO_CONFIG0_REG, I2C_MEMADD_SIZE_8BIT,
                                            &v, 1, 200);
    if (st != HAL_OK) {
        self->last_hal_err = HAL_I2C_GetError(self->i2c);
        self->last_err = (st == HAL_TIMEOUT) ? ICM_ERR_TIMEOUT : ICM_ERR_I2C;
        return st;
    }

    uint8_t set =
        ((clip4(gyro_odr)   << ICM_GYRO_CONFIG0_ODR_Pos) & ICM_GYRO_CONFIG0_ODR_Msk) |
        ((clip3(gyro_fs_sel)<< ICM_GYRO_CONFIG0_FS_Pos)  & ICM_GYRO_CONFIG0_FS_Msk);

    uint8_t new_v = (uint8_t)((v & ~ICM_GYRO_CONFIG0_WRITABLE_Msk) | set);
    if (new_v == v) {                // nothing to change
        self->last_err = ICM_OK;
        return HAL_OK;
    }

    st = HAL_I2C_Mem_Write(self->i2c, (self->addr7 << 1),
                           ICM_GYRO_CONFIG0_REG, I2C_MEMADD_SIZE_8BIT,
                           &new_v, 1, 200);
    if (st != HAL_OK) {
        self->last_hal_err = HAL_I2C_GetError(self->i2c);
        self->last_err = (st == HAL_TIMEOUT) ? ICM_ERR_TIMEOUT : ICM_ERR_I2C;
        return st;
    }

    switch (clip3(gyro_fs_sel)) {
        case 0: /* e.g. ±250 dps */   self->gyro_lsb_per_dps = 131.0f;   break;
        case 1: /* e.g. ±500 dps */   self->gyro_lsb_per_dps = 65.5f;    break;
        case 2: /* e.g. ±1000 dps */  self->gyro_lsb_per_dps = 32.8f;    break;
        case 3: /* e.g. ±2000 dps */  self->gyro_lsb_per_dps = 16.4f;    break;
        // Add cases if your chip exposes 125/62.5 dps etc.
        default: /* unknown */        /* leave as-is or set a safe default */ break;
    }

    self->last_err = ICM_OK;
    return HAL_OK;
}


/* ---------- Helpers ---------- */
/*
 * i2c: HAL I²C handle (from CubeMX).
 * a7: 7-bit slave address (e.g., 0x68/0x69). HAL wants an 8-bit address, so we do (a7 << 1).
 * reg: device register address to read from.
 * buf/n: output buffer & length.
 * 200: timeout in ms.
 * Returns HAL_OK/HAL_ERROR/HAL_BUSY/HAL_TIMEOUT.
 * Why shift? STM32 HAL HAL_I2C_* expects the address in “left-shifted 8-bit” form (R/W bit in bit0). Passing raw 7-bit without shifting will fail.
 */
static HAL_StatusTypeDef rd(I2C_HandleTypeDef *i2c, uint8_t a7, uint8_t reg, uint8_t *buf, uint16_t bufferSize) {
	return HAL_I2C_Mem_Read(i2c, (a7 << 1), reg, I2C_MEMADD_SIZE_8BIT, buf, bufferSize, 200);
}

static HAL_StatusTypeDef wr1(I2C_HandleTypeDef *i2c, uint8_t a7, uint8_t reg, uint8_t val) {
	return HAL_I2C_Mem_Write(i2c, (a7 << 1), reg, I2C_MEMADD_SIZE_8BIT, &val, 1, 200);
}

static inline uint8_t clip2(uint8_t x)	{ return (uint8_t)(x & 0x03u);	} // 2-bit
static inline uint8_t clip3(uint8_t x) 	{ return (uint8_t)(x & 0x07u); 	} // 3-bit
static inline uint8_t clip4(uint8_t x) 	{ return (uint8_t)(x & 0x0Fu); 	} // 4-bit


static icm42688_t* getSelfStatePointer(sensor_if_t *self) {
	if (!self || !self->state) return NULL;
	return (icm42688_t*)self->state;
}

