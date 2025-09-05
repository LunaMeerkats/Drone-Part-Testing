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

// Sensor functions
static HAL_StatusTypeDef icm_probe(sensor_if_t *self);
static HAL_StatusTypeDef icm_init(sensor_if_t *self);
static HAL_StatusTypeDef icm_read(sensor_if_t *self);
static HAL_StatusTypeDef icm_whoami(sensor_if_t *self, uint8_t *out);
void icm42688_bind(sensor_if_t *iface, icm42688_t *inst, I2C_HandleTypeDef *i2c, uint8_t addr7);

//Config
static HAL_StatusTypeDef icm_power_mgmt0_config(icm42688_t *self, uint8_t gyro_mode, uint8_t accel_mode, uint8_t temp_disable, uint8_t idle);
static HAL_StatusTypeDef icm_gyro_config0(icm42688_t *self, uint8_t gyro_fs_sel, uint8_t gyro_odr);
static HAL_StatusTypeDef icm_accel_config0(icm42688_t *self, uint8_t accel_fs_sel, uint8_t accel_odr);
static HAL_StatusTypeDef icm_soft_reset(icm42688_t *self);

//Read Values
static HAL_StatusTypeDef icm_read_temp_raw(icm42688_t *self);
static HAL_StatusTypeDef icm_read_temp_c(icm42688_t *self);
static HAL_StatusTypeDef icm_read_accel_raw(icm42688_t *self);
static HAL_StatusTypeDef icm_read_accel_g(icm42688_t *self);
static HAL_StatusTypeDef icm_read_gyro_raw(icm42688_t *self);
static HAL_StatusTypeDef icm_read_gyro_dps(icm42688_t *self);
static HAL_StatusTypeDef icm_read_all_raw(icm42688_t *self);
static HAL_StatusTypeDef icm_read_all(icm42688_t *self);

// Helpers
static HAL_StatusTypeDef rd(I2C_HandleTypeDef *i2c, uint8_t a7, uint8_t reg, uint8_t *buf, uint16_t bufferSize);
static HAL_StatusTypeDef wr1(I2C_HandleTypeDef *i2c, uint8_t a7, uint8_t reg, uint8_t val);
static inline uint8_t clip2(uint8_t x);
static inline uint8_t clip3(uint8_t x);
static inline uint8_t clip4(uint8_t x);
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

	HAL_StatusTypeDef response;

	response = icm_soft_reset(s);
	if (response != HAL_OK){
		s->last_err = ICM_ERR_NOT_INITIALIZED;
		return HAL_ERROR;
	}

	HAL_Delay(500);

	// Put in standby power modes
	response = icm_power_mgmt0_config(s, /*gyro_mode*/0, /*accel_mode*/0, /*temp_disable*/1, /*idle*/0);
	if (response != HAL_OK){
		s->last_err = ICM_ERR_NOT_INITIALIZED;
		return HAL_ERROR;
	}

	HAL_Delay(500);

	response = icm_gyro_config0(s, 0, 6);
	if (response != HAL_OK){
		s->last_err = ICM_ERR_NOT_INITIALIZED;
		return HAL_ERROR;
	}

	HAL_Delay(500);

	response = icm_accel_config0(s, 0 , 6);
	if (response != HAL_OK){
		s->last_err = ICM_ERR_NOT_INITIALIZED;
		return HAL_ERROR;
	}

	HAL_Delay(500);

	// Configure power modes
	response = icm_power_mgmt0_config(s, /*gyro_mode*/2, /*accel_mode*/2, /*temp_disable*/0, /*idle*/0);
	if (response != HAL_OK){
		s->last_err = ICM_ERR_NOT_INITIALIZED;
		return HAL_ERROR;
	}

	HAL_Delay(500);

	s->last_err = ICM_OK;
	return HAL_OK;
}

static HAL_StatusTypeDef icm_read(sensor_if_t *self) {
	icm42688_t *s = getSelfStatePointer(self);
	if (!s) return HAL_ERROR;

	return icm_read_all(s);
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
static HAL_StatusTypeDef icm_power_mgmt0_config(icm42688_t *self,
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
			(temp_disable ? ICM_POWER_MGMT0_TEMP_DISABLE_Msk  : 0u) |
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
			((clip4(gyro_odr)    << ICM_GYRO_CONFIG0_ODR_POS) & ICM_GYRO_CONFIG0_ODR_Msk) |
			((clip3(gyro_fs_sel) << ICM_GYRO_CONFIG0_FS_POS)  & ICM_GYRO_CONFIG0_FS_Msk);

	uint8_t new_v = (uint8_t)((v & ~ICM_GYRO_CONFIG0_WRITABLE_Msk) | set);

	if (new_v != v) {
		st = HAL_I2C_Mem_Write(self->i2c, (self->addr7 << 1),
				ICM_GYRO_CONFIG0_REG, I2C_MEMADD_SIZE_8BIT,
				&new_v, 1, 200);
		if (st != HAL_OK) {
			self->last_hal_err = HAL_I2C_GetError(self->i2c);
			self->last_err = (st == HAL_TIMEOUT) ? ICM_ERR_TIMEOUT : ICM_ERR_I2C;
			return st;
		}
	}

	// >>> ALWAYS derive effective settings from the value we now have in the register
	uint8_t eff = (new_v != v) ? new_v : v;
	uint8_t fs_code  = (eff & ICM_GYRO_CONFIG0_FS_Msk)  >> ICM_GYRO_CONFIG0_FS_POS;
	uint8_t odr_code = (eff & ICM_GYRO_CONFIG0_ODR_Msk) >> ICM_GYRO_CONFIG0_ODR_POS;

	// FS -> LSB per dps  (keep switch for readability)
	switch (fs_code & 0x7u) {
	case 0: self->gyro_lsb_per_dps = 16.384f;   break; // ±2000 dps
	case 1: self->gyro_lsb_per_dps = 32.768f;   break; // ±1000
	case 2: self->gyro_lsb_per_dps = 65.536f;   break; // ±500
	case 3: self->gyro_lsb_per_dps = 131.072f;  break; // ±250
	case 4: self->gyro_lsb_per_dps = 262.144f;  break; // ±125
	case 5: self->gyro_lsb_per_dps = 524.288f;  break; // ±62.5
	case 6: self->gyro_lsb_per_dps = 1048.576f; break; // ±31.25
	case 7: self->gyro_lsb_per_dps = 2097.152f; break; // ±15.625
	}

	// ODR code -> Hz
	switch (odr_code & 0xFu) {
	case 1:  self->gyro_odr = 32000.0f; break;
	case 2:  self->gyro_odr = 16000.0f; break;
	case 3:  self->gyro_odr = 8000.0f;  break;
	case 4:  self->gyro_odr = 4000.0f;  break;
	case 5:  self->gyro_odr = 2000.0f;  break;
	case 6:  self->gyro_odr = 1000.0f;  break; // reset default
	case 7:  self->gyro_odr = 200.0f;   break;
	case 8:  self->gyro_odr = 100.0f;   break;
	case 9:  self->gyro_odr = 50.0f;    break;
	case 10: self->gyro_odr = 25.0f;    break;
	case 11: self->gyro_odr = 12.5f;    break;
	case 15: self->gyro_odr = 500.0f;   break;
	default: self->gyro_odr = 0.0f;     break; // reserved codes
	}

	self->last_err = ICM_OK;
	return HAL_OK;
}

static HAL_StatusTypeDef icm_accel_config0(icm42688_t *self, uint8_t accel_fs_sel, uint8_t accel_odr)
{
	if (!self) return HAL_ERROR;

	uint8_t v;
	HAL_StatusTypeDef st = HAL_I2C_Mem_Read(self->i2c, (self->addr7 << 1),
			ICM_ACCEL_CONFIG0_REG, I2C_MEMADD_SIZE_8BIT,
			&v, 1, 200);
	if (st != HAL_OK) {
		self->last_hal_err = HAL_I2C_GetError(self->i2c);
		self->last_err = (st == HAL_TIMEOUT) ? ICM_ERR_TIMEOUT : ICM_ERR_I2C;
		return st;
	}

	uint8_t set =
			((clip4(accel_odr)    << ICM_ACCEL_CONFIG0_ODR_POS) & ICM_ACCEL_CONFIG0_ODR_Msk) |
			((clip3(accel_fs_sel) << ICM_ACCEL_CONFIG0_FS_POS)  & ICM_ACCEL_CONFIG0_FS_Msk);

	uint8_t new_v = (uint8_t)((v & ~ICM_ACCEL_CONFIG0_WRITABLE_Msk) | set);

	if (new_v != v) {
		st = HAL_I2C_Mem_Write(self->i2c, (self->addr7 << 1),
				ICM_ACCEL_CONFIG0_REG, I2C_MEMADD_SIZE_8BIT,
				&new_v, 1, 200);
		if (st != HAL_OK) {
			self->last_hal_err = HAL_I2C_GetError(self->i2c);
			self->last_err = (st == HAL_TIMEOUT) ? ICM_ERR_TIMEOUT : ICM_ERR_I2C;
			return st;
		}
	}

	// >>> ALWAYS derive effective settings
	uint8_t eff = (new_v != v) ? new_v : v;
	uint8_t fs_code  = (eff & ICM_ACCEL_CONFIG0_FS_Msk)  >> ICM_ACCEL_CONFIG0_FS_POS;
	uint8_t odr_code = (eff & ICM_ACCEL_CONFIG0_ODR_Msk) >> ICM_ACCEL_CONFIG0_ODR_POS;

	// FS -> LSB per g (assuming 00=±2g, 01=±4g, 10=±8g, 11=±16g)
	switch (fs_code & 0x3u) {
	case 0: self->accel_lsb_per_g = 2048.0f;  break; // ±16g
	case 1: self->accel_lsb_per_g = 4096.0f;  break; // ±8g
	case 2: self->accel_lsb_per_g = 8192.0f;  break; // ±4g
	case 3: self->accel_lsb_per_g = 16384.0f; break; // ±2g
	}

	// ODR code -> Hz
	switch (odr_code & 0xFu) {
	case 1:  self->accel_odr = 32000.0f; break;
	case 2:  self->accel_odr = 16000.0f; break;
	case 3:  self->accel_odr = 8000.0f;  break;
	case 4:  self->accel_odr = 4000.0f;  break;
	case 5:  self->accel_odr = 2000.0f;  break;
	case 6:  self->accel_odr = 1000.0f;  break; // reset default
	case 7:  self->accel_odr = 200.0f;   break;
	case 8:  self->accel_odr = 100.0f;   break;
	case 9:  self->accel_odr = 50.0f;    break;
	case 10: self->accel_odr = 25.0f;    break;
	case 11: self->accel_odr = 12.5f;    break;
	case 12: self->accel_odr = 6.25f;    break;
	case 13: self->accel_odr = 3.125f;   break;
	case 14: self->accel_odr = 1.5625f;  break;
	case 15: self->accel_odr = 500.0f;   break;
	default: self->accel_odr = 0.0f;     break;
	}

	self->last_err = ICM_OK;
	return HAL_OK;
}

static HAL_StatusTypeDef icm_soft_reset(icm42688_t *s)
{
	if (!s) return HAL_ERROR;

	uint8_t cmd = ICM_SOFT_RESET_Msk;                 // write-one-to-reset
	HAL_StatusTypeDef st = HAL_I2C_Mem_Write(s->i2c, (s->addr7 << 1),
			ICM_SOFT_RESET_REG, I2C_MEMADD_SIZE_8BIT,
			&cmd, 1, 200);
	if (st != HAL_OK) {
		s->last_hal_err = HAL_I2C_GetError(s->i2c);
		s->last_err = (st == HAL_TIMEOUT) ? ICM_ERR_TIMEOUT : ICM_ERR_I2C;
		return st;
	}

	// Give the device time to reboot and reload defaults.
	// Typical is a few ms; be conservative.
	HAL_Delay(10);

	// Optional: verify reset by reading a known reset-value register
	// (e.g., WHO_AM_I or checking PWR_MGMT0 returned to its reset state).
	uint8_t who = 0;
	st = HAL_I2C_Mem_Read(s->i2c, (s->addr7 << 1), ICM_WHO_AM_I_REG,
			I2C_MEMADD_SIZE_8BIT, &who, 1, 200);
	if (st != HAL_OK || who != ICM_WHO_AM_I_DEFAULT) {
		s->last_hal_err = HAL_I2C_GetError(s->i2c);
		s->last_err = ICM_ERR_WHOAMI_MISMATCH;
		s->last_whoami = who;
		return (st != HAL_OK) ? st : HAL_ERROR;
	}

	s->last_err = ICM_OK;
	return HAL_OK;
}
/* ---------- Read Values ---------- */
static HAL_StatusTypeDef icm_read_temp_raw(icm42688_t *self) {
	uint8_t raw[2];
	HAL_StatusTypeDef st = rd(self->i2c, self->addr7, ICM_TEMP_MSB, raw, 2);
	if (st != HAL_OK) {
		self->last_hal_err = HAL_I2C_GetError(self->i2c);
		self->last_err = (st == HAL_TIMEOUT) ? ICM_ERR_TIMEOUT : ICM_ERR_I2C;
		return st;
	}

	self->temp_raw = (int16_t)((raw[0] << 8) | raw[1]);  // MSB, LSB
	self->last_err = ICM_OK;
	return HAL_OK;
}

static HAL_StatusTypeDef icm_read_temp_c(icm42688_t *self) {

	HAL_StatusTypeDef st = icm_read_temp_raw(self);

	if (st != HAL_OK) return st;
	self->last_temp_c = (self->temp_raw / ICM_TEMP_SENS_LSB_PER_C) + ICM_TEMP_OFFSET_C;
	return HAL_OK;
}

static HAL_StatusTypeDef icm_read_accel_raw(icm42688_t *self) {
	uint8_t raw[6];
	HAL_StatusTypeDef st = rd(self->i2c, self->addr7, ICM_ACCEL_X_MSB, raw, 6);
	if (st != HAL_OK) {
		self->last_hal_err = HAL_I2C_GetError(self->i2c);
		self->last_err = (st == HAL_TIMEOUT) ? ICM_ERR_TIMEOUT : ICM_ERR_I2C;
		return st;
	}

	self->accel_raw[0] = (int16_t)((raw[0] << 8) | raw[1]);  // MSB, LSB
	self->accel_raw[1] = (int16_t)((raw[2] << 8) | raw[3]);  // MSB, LSB
	self->accel_raw[2] = (int16_t)((raw[4] << 8) | raw[5]);  // MSB, LSB
	self->last_err = ICM_OK;
	return HAL_OK;
}

static HAL_StatusTypeDef icm_read_accel_g(icm42688_t *self) {
	if (!self) return HAL_ERROR;
	HAL_StatusTypeDef st = icm_read_accel_raw(self);
	if (st != HAL_OK) return st;

	const float scale = self->accel_lsb_per_g;
	self->accel_g[0] = self->accel_raw[0] / scale;
	self->accel_g[1] = self->accel_raw[1] / scale;
	self->accel_g[2] = self->accel_raw[2] / scale;
	return HAL_OK;
}

static HAL_StatusTypeDef icm_read_gyro_raw(icm42688_t *self) {
	uint8_t raw[6];
	HAL_StatusTypeDef st = rd(self->i2c, self->addr7, ICM_GYRO_X_MSB, raw, 6);
	if (st != HAL_OK) {
		self->last_hal_err = HAL_I2C_GetError(self->i2c);
		self->last_err = (st == HAL_TIMEOUT) ? ICM_ERR_TIMEOUT : ICM_ERR_I2C;
		return st;
	}

	self->gyro_raw[0] = (int16_t)((raw[0] << 8) | raw[1]);  // MSB, LSB
	self->gyro_raw[1] = (int16_t)((raw[2] << 8) | raw[3]);  // MSB, LSB
	self->gyro_raw[2] = (int16_t)((raw[4] << 8) | raw[5]);  // MSB, LSB
	self->last_err = ICM_OK;
	return HAL_OK;
}

static HAL_StatusTypeDef icm_read_gyro_dps(icm42688_t *self) {
	if (!self) return HAL_ERROR;
	HAL_StatusTypeDef st = icm_read_gyro_raw(self);
	if (st != HAL_OK) return st;

	const float scale = self->gyro_lsb_per_dps;
	self->gyro_dps[0] = self->gyro_raw[0] / scale;
	self->gyro_dps[1] = self->gyro_raw[1] / scale;
	self->gyro_dps[2] = self->gyro_raw[2] / scale;
	return HAL_OK;
}

// Read TEMP(2) + ACCEL(6) + GYRO(6) in one shot: 0x1D..0x2A (14 bytes)
static HAL_StatusTypeDef icm_read_all_raw(icm42688_t *self) {
	if (!self) return HAL_ERROR;

	uint8_t raw[14];

	HAL_StatusTypeDef st = rd(self->i2c, self->addr7, ICM_TEMP_MSB, raw, 14);
	if (st != HAL_OK) {
		self->last_hal_err = HAL_I2C_GetError(self->i2c);
		self->last_err = (st == HAL_TIMEOUT) ? ICM_ERR_TIMEOUT : ICM_ERR_I2C;
		return st;
	}

	// index map: 0..1 Temp, 2..3 Ax, 4..5 Ay, 6..7 Az, 8..9 Gx, 10..11 Gy, 12..13 Gz
	self->temp_raw     = (int16_t)((raw[0]  << 8) | raw[1]);
	self->accel_raw[0] = (int16_t)((raw[2]  << 8) | raw[3]);
	self->accel_raw[1] = (int16_t)((raw[4]  << 8) | raw[5]);
	self->accel_raw[2] = (int16_t)((raw[6]  << 8) | raw[7]);
	self->gyro_raw[0]  = (int16_t)((raw[8]  << 8) | raw[9]);
	self->gyro_raw[1]  = (int16_t)((raw[10] << 8) | raw[11]);
	self->gyro_raw[2]  = (int16_t)((raw[12] << 8) | raw[13]);

	self->last_err = ICM_OK;
	return HAL_OK;
}

// Convert everything: raw → °C, g, dps (uses configured scale factors)
static HAL_StatusTypeDef icm_read_all(icm42688_t *self) {
	if (!self) return HAL_ERROR;

	HAL_StatusTypeDef st = icm_read_all_raw(self);
	if (st != HAL_OK) return st;

	// Temperature
	self->last_temp_c = (self->temp_raw / ICM_TEMP_SENS_LSB_PER_C) + ICM_TEMP_OFFSET_C;

	// Scales: guard against zero if config wasn’t called for some reason
	const float a_scale = self->accel_lsb_per_g;
	const float g_scale = self->gyro_lsb_per_dps;

	// Accel
	self->accel_g[0] = self->accel_raw[0] / a_scale;
	self->accel_g[1] = self->accel_raw[1] / a_scale;
	self->accel_g[2] = self->accel_raw[2] / a_scale;

	// Gyro
	self->gyro_dps[0] = self->gyro_raw[0] / g_scale;
	self->gyro_dps[1] = self->gyro_raw[1] / g_scale;
	self->gyro_dps[2] = self->gyro_raw[2] / g_scale;

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

