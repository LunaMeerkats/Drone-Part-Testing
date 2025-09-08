/*
 * dps310.c — minimal bring-up: probe + WHO_AM_I only
 * You fill in init()/read() later.
 */

#include "dps310.h"

// Sensor functions
static HAL_StatusTypeDef dps_probe(sensor_if_t *self);
static HAL_StatusTypeDef dps_init(sensor_if_t *self);
static HAL_StatusTypeDef dps_read(sensor_if_t *self);
static HAL_StatusTypeDef dps_whoami(sensor_if_t *self, uint8_t *out);
void dps310_bind(sensor_if_t *iface, dps310_t *inst, I2C_HandleTypeDef *i2c, uint8_t addr7);

//Config
static HAL_StatusTypeDef dsp_pressure_configuration(dps310_t *self, uint8_t pressure_measurement_rate, uint8_t pressure_oversampling_rate);
static HAL_StatusTypeDef dsp_temperature_configuration(dps310_t *self, uint8_t temperature_measurement_sensor, uint8_t temperate_measurement_rate, uint8_t temperature_oversampling_rate);
static HAL_StatusTypeDef dsp_status_configuration(dps310_t *self, uint8_t meas_ctrl);
static HAL_StatusTypeDef dsp_soft_reset(dps310_t *self);
static HAL_StatusTypeDef dsp_read_coefficients(dps310_t *self);
static HAL_StatusTypeDef dsp_compensate_all(dps310_t *self);

//Read Values
static HAL_StatusTypeDef dsp_read_pressure_raw(dps310_t *self);
static HAL_StatusTypeDef dsp_read_pressure(dps310_t *self);
static HAL_StatusTypeDef dsp_read_temperature_raw(dps310_t *self);
static HAL_StatusTypeDef dsp_read_temperature(dps310_t *self);

// Helpers
static inline uint8_t clip2(uint8_t x);
static inline uint8_t clip3(uint8_t x);
static inline uint8_t clip4(uint8_t x);
static inline int32_t sx24(uint32_t u);
static inline int16_t sx12(uint16_t u);
static inline int32_t sx20(uint32_t u);
static inline float   dps_osr_scale(uint8_t osr_code);
static dps310_t* getSelfStatePointer(sensor_if_t *self);

/* ---- vtable ops (minimal) ---- */
static HAL_StatusTypeDef dps_probe(sensor_if_t *self)
{
	dps310_t *s = (dps310_t*)self->state;
	if (!s) return HAL_ERROR;

	uint8_t id = 0;
	HAL_StatusTypeDef st = HAL_I2C_Mem_Read(s->i2c, s->addr7 << 1,
			DPS_ID_REG, I2C_MEMADD_SIZE_8BIT,
			&id, 1, 200);
	if (st != HAL_OK) return st;

	s->last_whoami = id;
	return (id == DPS_WHOAMI_EXPECT) ? HAL_OK : HAL_ERROR;
}

static HAL_StatusTypeDef dps_init(sensor_if_t *self)
{
	dps310_t *s = getSelfStatePointer(self);

	HAL_StatusTypeDef response;

	//TODO: If this is 8 or higher, a bit shift needs to be applied. Check the docs.
	const uint8_t pressure_oversampling_rate = 6;

	response = dsp_soft_reset(s);
	if(response != HAL_OK){
		s->last_err = DPS_ERR_NOT_INITIALIZED;
		return HAL_ERROR;
	}

	HAL_Delay(500);

	response = dsp_pressure_configuration(s, 6, pressure_oversampling_rate);
	if(response != HAL_OK){
		s->last_err = DPS_ERR_NOT_INITIALIZED;
		return response;
	}

	HAL_Delay(500);

	response = dsp_temperature_configuration(s, 1, 6, 6);
	if(response != HAL_OK){
		s->last_err = DPS_ERR_NOT_INITIALIZED;
		return response;
	}

	HAL_Delay(500);
	response = dsp_status_configuration(s, 7);
	if(response != HAL_OK){
		s->last_err = DPS_ERR_NOT_INITIALIZED;
		return response;
	}

	HAL_Delay(500);

	response = dsp_read_coefficients(s);
	if(response != HAL_OK){
		s->last_err = DPS_ERR_NOT_INITIALIZED;
		return response;
	}

	HAL_Delay(500);

	// read latest raw values
	response = dsp_read_temperature_raw(s);
	if (response != HAL_OK)
		return response;

	response = dsp_read_pressure_raw(s);
	if (response != HAL_OK)
		return response;

	// compensate to engineering units
	response = dsp_compensate_all(s);
	if (response != HAL_OK)
		return response;

	s->last_err = DPS_OK;
	return HAL_OK;
}

static HAL_StatusTypeDef dps_read(sensor_if_t *self)
{
	dps310_t *s = getSelfStatePointer(self);

	HAL_StatusTypeDef response;

	response = dsp_read_pressure_raw(s);
	if(response != HAL_OK){
		s->last_err = DPS_ERR_NOT_INITIALIZED;
		return HAL_ERROR;
	}

	response = dsp_read_temperature_raw(s);  // fill self->temperature_raw   (sign-extended 24-bit, as float)
	if(response != HAL_OK){
		s->last_err = DPS_ERR_NOT_INITIALIZED;
		return HAL_ERROR;
	}

	response = dsp_read_pressure_raw(s);     // fill self->pressure_raw      (sign-extended 24-bit, as float)
	if(response != HAL_OK){
		s->last_err = DPS_ERR_NOT_INITIALIZED;
		return HAL_ERROR;
	}

	response = dsp_compensate_all(s);        // fills self->temperature (°C) and self->pressure (Pa)
	if(response != HAL_OK){
		s->last_err = DPS_ERR_NOT_INITIALIZED;
		return HAL_ERROR;
	}

}

static HAL_StatusTypeDef dps_whoami(sensor_if_t *self, uint8_t *out)
{
	dps310_t *s = (dps310_t*)self->state;
	if (!s || !out) return HAL_ERROR;

	return HAL_I2C_Mem_Read(s->i2c, s->addr7 << 1,
			DPS_ID_REG, I2C_MEMADD_SIZE_8BIT,
			out, 1, 200);
}

/* ---- bind ---- */
static const sensor_if_vtbl_t DPS310_VTBL = {
		.probe  = dps_probe,
		.init   = dps_init,
		.read   = dps_read,
		.whoami = dps_whoami
};

void dps310_bind(sensor_if_t *iface, dps310_t *inst,
		I2C_HandleTypeDef *i2c, uint8_t addr7)
{
	inst->i2c   = i2c;
	inst->addr7 = addr7;
	iface->vTable    = &DPS310_VTBL;
	iface->state = inst;
}

/* ---------- Configuration ---------- */
static HAL_StatusTypeDef dsp_pressure_configuration(dps310_t *self, uint8_t pressure_measurement_rate, uint8_t pressure_oversampling_rate){
	if(!self) {return HAL_ERROR;}

	uint8_t v;
	HAL_StatusTypeDef response;

	pressure_measurement_rate = clip3(pressure_measurement_rate);
	pressure_oversampling_rate = clip4(pressure_oversampling_rate);

	//Read
	response = HAL_I2C_Mem_Read(self->i2c, self->addr7<<1, DPS_PRESSURE_CONFIGURATION_Reg, I2C_MEMADD_SIZE_8BIT, &v, sizeof(v), 200);
	if(response != HAL_OK){
		self->last_err = DPS_ERR_I2C;
		return HAL_ERROR;
	}

	//Modify
	uint8_t set = (uint8_t)	(pressure_measurement_rate 	<< DPS_PRESSURE_CONFIGURATION_MR_Pos) | \
			(pressure_oversampling_rate << DPS_PRESSURE_CONFIGURATION_POR_Pos);

	uint8_t nv   = (uint8_t)((v & ~DPS_PRESSURE_CONFIGURATION_Msk) | set);


	//Write
	if (nv != v){
		response = HAL_I2C_Mem_Write(self->i2c, self->addr7<<1, DPS_PRESSURE_CONFIGURATION_Reg, I2C_MEMADD_SIZE_8BIT, &nv, sizeof(nv), 200);
		if(response != HAL_OK){
			self->last_err = DPS_ERR_I2C;
			return HAL_ERROR;
		}
	}

	self->p_osr_code = (uint8_t)(pressure_oversampling_rate & 0x0F);
	self->kP = dps_osr_scale(self->p_osr_code);


	self->last_err = DPS_OK;
	return HAL_OK;
}

static HAL_StatusTypeDef dsp_temperature_configuration(dps310_t *self, uint8_t temperature_sensor, uint8_t temperature_measurement_rate, uint8_t temperature_oversampling_rate){
	if(!self) {return HAL_ERROR;}

	uint8_t v;
	HAL_StatusTypeDef response;

	temperature_measurement_rate = clip3(temperature_measurement_rate);
	temperature_oversampling_rate = clip4(temperature_oversampling_rate);

	//Read
	response = HAL_I2C_Mem_Read(self->i2c, self->addr7<<1, DPS_TEMPERATURE_CONFIGURATION_Reg, I2C_MEMADD_SIZE_8BIT, &v, sizeof(v), 200);
	if(response != HAL_OK){
		self->last_err = DPS_ERR_I2C;
		return HAL_ERROR;
	}

	//Modify
	uint8_t set = (uint8_t)	(temperature_sensor << DPS_TEMPERATURE_CONFIGURATION_SENSE_Pos) | \
			(temperature_measurement_rate 	<< DPS_TEMPERATURE_CONFIGURATION_MR_Pos) | \
			(temperature_oversampling_rate << DPS_TEMPERATURE_CONFIGURATION_TOS_Pos);

	uint8_t nv   = (uint8_t)((v & ~DPS_TEMPERATURE_CONFIGURATION_Msk) | set);


	//Write
	if (nv != v){
		response = HAL_I2C_Mem_Write(self->i2c, self->addr7<<1, DPS_TEMPERATURE_CONFIGURATION_Reg, I2C_MEMADD_SIZE_8BIT, &nv, sizeof(nv), 200);
		if(response != HAL_OK){
			self->last_err = DPS_ERR_I2C;
			return HAL_ERROR;
		}
	}

	self->t_osr_code = (uint8_t)(temperature_oversampling_rate & 0x0F);
	self->kT = dps_osr_scale(self->t_osr_code);

	self->last_err = DPS_OK;
	return HAL_OK;
}

static HAL_StatusTypeDef dsp_status_configuration(dps310_t *self, uint8_t meas_ctrl){
	if (!self) return HAL_ERROR;
	meas_ctrl = clip3(meas_ctrl);

	uint8_t v = 0;
	uint32_t t0 = HAL_GetTick();
	for (;;) {
		HAL_StatusTypeDef st = HAL_I2C_Mem_Read(self->i2c, (self->addr7 << 1),
				DPS_STATUS_Reg, I2C_MEMADD_SIZE_8BIT,
				&v, 1, 200);
		if (st != HAL_OK) { self->last_err = DPS_ERR_I2C; return st; }
		if ((v & DPS_STATUS_READY_Msk) == DPS_STATUS_READY_Msk) break;
		if ((HAL_GetTick() - t0) > 5000) {
			self->last_err = DPS_ERR_NOT_INITIALIZED;
			return HAL_TIMEOUT;
		}
		HAL_Delay(1000);
	}

	// ---- RMW only MEAS_CTRL[2:0] ----
	uint8_t set = (uint8_t)(meas_ctrl << DPS_STATUS_MEAS_CTRL_Pos);
	uint8_t nv  = (uint8_t)((v & ~DPS_STATUS_MEAS_CTRL_Msk) | set);

	if (nv != v) {
		HAL_StatusTypeDef st = HAL_I2C_Mem_Write(self->i2c, (self->addr7 << 1), DPS_STATUS_Reg, I2C_MEMADD_SIZE_8BIT, &nv, 1, 200);
		if (st != HAL_OK) { self->last_err = DPS_ERR_I2C; return st; }
	}

	self->last_err = DPS_OK;
	return HAL_OK;
}

static HAL_StatusTypeDef dsp_soft_reset(dps310_t *self){
	if (!self) return HAL_ERROR;
	uint8_t v = 0x09; // SOFT_RST = 0b1001
	return HAL_I2C_Mem_Write(self->i2c, self->addr7 << 1,
			DPS_SOFT_RESET_Reg, I2C_MEMADD_SIZE_8BIT,
			&v, 1, 200);
}

static HAL_StatusTypeDef dsp_read_coefficients(dps310_t *self)
{
	if (!self) return HAL_ERROR;

	uint8_t b[DPS_COEF_BLOCK_LEN];  // 0x10 .. 0x21
	HAL_StatusTypeDef st = HAL_I2C_Mem_Read(self->i2c, self->addr7 << 1,
			DPS_COEF_BLOCK_START, I2C_MEMADD_SIZE_8BIT,
			b, sizeof(b), 200);
	if (st != HAL_OK) {
		self->last_hal_err = HAL_I2C_GetError(self->i2c);
		self->last_err = (st == HAL_TIMEOUT) ? DPS_ERR_TIMEOUT : DPS_ERR_I2C;
		return st;
	}

	/* Index helper: b[0] = 0x10, b[1] = 0x11, ... b[18] = 0x22 (but we read to 0x21) */
	/* c0: 12-bit signed: 0x10[7:0]=c0[11:4], 0x11[7:4]=c0[3:0] */
	uint16_t c0u = ((uint16_t)b[0] << 4) | (b[1] >> 4);
	self->c0 = sx12(c0u);

	/* c1: 12-bit signed: 0x11[3:0]=c1[11:8], 0x12= c1[7:0] */
	uint16_t c1u = ((uint16_t)(b[1] & 0x0F) << 8) | b[2];
	self->c1 = sx12(c1u);

	/* c00: 20-bit signed: 0x13[7:0]=[19:12], 0x14[7:0]=[11:4], 0x15[7:4]=[3:0] */
	uint32_t c00u = ((uint32_t)b[3] << 12) | ((uint32_t)b[4] << 4) | (b[5] >> 4);
	self->c00 = sx20(c00u);

	/* c10: 20-bit signed: 0x15[3:0]=[19:16], 0x16[7:0]=[15:8], 0x17[7:0]=[7:0] */
	uint32_t c10u = ((uint32_t)(b[5] & 0x0F) << 16) | ((uint32_t)b[6] << 8) | b[7];
	self->c10 = sx20(c10u);

	/* signed 16-bit pairs */
	self->c01 = (int16_t)((b[8]  << 8) | b[9]);   // 0x18,0x19
	self->c11 = (int16_t)((b[10] << 8) | b[11]);  // 0x1A,0x1B
	self->c20 = (int16_t)((b[12] << 8) | b[13]);  // 0x1C,0x1D
	self->c21 = (int16_t)((b[14] << 8) | b[15]);  // 0x1E,0x1F
	self->c30 = (int16_t)((b[16] << 8) | b[17]);  // 0x20,0x21

	/* If you captured OSR codes in your config, prime the scale factors now */
	self->kT = dps_osr_scale(self->t_osr_code);
	self->kP = dps_osr_scale(self->p_osr_code);

	self->last_err = DPS_OK;
	return HAL_OK;
}

/* ---- compute engineering units from latest raw values ----
 * Uses:
 *   T_sc = temperature_raw / kT
 *   P_sc = pressure_raw    / kP
 *   T(°C) = c0*0.5 + c1*T_sc
 *   P(Pa) = c00 + P_sc*(c10 + P_sc*(c20 + P_sc*c30))
 *                + T_sc*(c01 + P_sc*(c11 + P_sc*c21))
 */
static HAL_StatusTypeDef dsp_compensate_all(dps310_t *self)
{
	if (!self) return HAL_ERROR;
	if (self->kT <= 0.f || self->kP <= 0.f) {
		/* Set from your config routines or default to OSR code 0 if unset */
		if (self->kT <= 0.f) self->kT = dps_osr_scale(self->t_osr_code);
		if (self->kP <= 0.f) self->kP = dps_osr_scale(self->p_osr_code);
		if (self->kT <= 0.f || self->kP <= 0.f) return HAL_ERROR;
	}

	const float T_sc = self->temperature_raw / self->kT;
	const float P_sc = self->pressure_raw    / self->kP;

	/* Temperature in °C */
	self->temperature = 0.5f * (float)self->c0 + (float)self->c1 * T_sc;

	/* Pressure in Pa (polynomial from datasheet) */
	const float P_lin =
			(float)self->c00
			+ P_sc * ((float)self->c10 + P_sc * ((float)self->c20 + P_sc * (float)self->c30))
			+ T_sc * ((float)self->c01 + P_sc * ((float)self->c11 + P_sc * (float)self->c21));

	self->pressure = P_lin;

	self->last_err = DPS_OK;
	return HAL_OK;
}


/* ---------- Read Values ---------- */
static HAL_StatusTypeDef dsp_read_pressure_raw(dps310_t *self)
{
	if (!self) return HAL_ERROR;

	uint8_t v[3];
	HAL_StatusTypeDef st = HAL_I2C_Mem_Read(self->i2c, self->addr7 << 1,
			DPS_PRESSURE_BYTE3_Reg, I2C_MEMADD_SIZE_8BIT,
			v, 3, 200);
	if (st != HAL_OK) {
		self->last_hal_err = HAL_I2C_GetError(self->i2c);
		self->last_err = (st == HAL_TIMEOUT) ? DPS_ERR_TIMEOUT : DPS_ERR_I2C;
		return st;
	}

	uint32_t u = ((uint32_t)v[0] << 16) | ((uint32_t)v[1] << 8) | (uint32_t)v[2];
	self->pressure_raw = sx24(u);

	self->last_err = DPS_OK;
	return HAL_OK;
}


static HAL_StatusTypeDef dsp_read_temperature_raw(dps310_t *self)
{
	if (!self) return HAL_ERROR;

	uint8_t v[3];
	HAL_StatusTypeDef st = HAL_I2C_Mem_Read(self->i2c, self->addr7 << 1,
			DPS_TEMPERATURE_BYTE3_Reg, I2C_MEMADD_SIZE_8BIT,
			v, 3, 200);
	if (st != HAL_OK) {
		self->last_hal_err = HAL_I2C_GetError(self->i2c);
		self->last_err = (st == HAL_TIMEOUT) ? DPS_ERR_TIMEOUT : DPS_ERR_I2C;
		return st;
	}

	const uint32_t u = ((uint32_t)v[0] << 16) | ((uint32_t)v[1] << 8) | (uint32_t)v[2];
	self->temperature_raw = sx24(u);

	self->last_err = DPS_OK;
	return HAL_OK;
}


/* ---------- Helpers ---------- */
static inline uint8_t clip2(uint8_t x)	{ return (uint8_t)(x & 0x03u);	} // 2-bit
static inline uint8_t clip3(uint8_t x) 	{ return (uint8_t)(x & 0x07u); 	} // 3-bit
static inline uint8_t clip4(uint8_t x) 	{ return (uint8_t)(x & 0x0Fu); 	} // 4-bit

static inline int32_t sx24(uint32_t u) {
	return (u & 0x00800000u) ? (int32_t)(u | 0xFF000000u) : (int32_t)u;
}

static inline int16_t sx12(uint16_t u) {            // 12-bit signed -> int16
	return (u & 0x0800u) ? (int16_t)(u | 0xF000u) : (int16_t)u;
}
static inline int32_t sx20(uint32_t u) {            // 20-bit signed -> int32
	return (u & 0x00080000u) ? (int32_t)(u | 0xFFF00000u) : (int32_t)u;
}

/* OSR code (0..7) -> scale factor used for raw scaling (Infineon table) */
static inline float dps_osr_scale(uint8_t osr_code) {
	static const float K[8] = {
			524288.0f,  1572864.0f, 3670016.0f, 7864320.0f,
			253952.0f,   516096.0f,  1040384.0f, 2088960.0f
	};
	return K[osr_code & 7u];
}

static dps310_t* getSelfStatePointer(sensor_if_t *self) {
	if (!self || !self->state) return NULL;
	return (dps310_t*)self->state;
}
