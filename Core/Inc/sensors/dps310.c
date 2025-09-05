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

//Read Values
static HAL_StatusTypeDef dsp_read_pressure_raw(dps310_t *self);
static HAL_StatusTypeDef dsp_read_pressure(dps310_t *self);
static HAL_StatusTypeDef dsp_read_temperature_raw(dps310_t *self);
static HAL_StatusTypeDef dsp_read_temperature(dps310_t *self);

// Helpers
static inline uint8_t clip2(uint8_t x);
static inline uint8_t clip3(uint8_t x);
static inline uint8_t clip4(uint8_t x);
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
		return HAL_ERROR;
	}

	HAL_Delay(500);

	response = dsp_temperature_configuration(s, 1, 6, 6);
	if(response != HAL_OK){
		s->last_err = DPS_ERR_NOT_INITIALIZED;
		return HAL_ERROR;
	}

	HAL_Delay(500);
	response = dsp_status_configuration(s, 7);
	if(response != HAL_OK){
		s->last_err = DPS_ERR_NOT_INITIALIZED;
		return HAL_ERROR;
	}

	HAL_Delay(500);


	s->last_err = DPS_OK;
	return HAL_OK;
}

static HAL_StatusTypeDef dps_read(sensor_if_t *self)
{
	(void)self;
	return HAL_OK;
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
		if ((HAL_GetTick() - t0) > 200) {
			self->last_err = DPS_ERR_NOT_INITIALIZED;
			return HAL_TIMEOUT;
		}
		HAL_Delay(5);
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


/* ---------- Read Values ---------- */
static HAL_StatusTypeDef dsp_read_pressure_raw(dps310_t *self){

}

static HAL_StatusTypeDef dsp_read_pressure(dps310_t *self){

}

static HAL_StatusTypeDef dsp_read_temperature_raw(dps310_t *self){

}

static HAL_StatusTypeDef dsp_read_temperature(dps310_t *self){

}

/* ---------- Helpers ---------- */
static inline uint8_t clip2(uint8_t x)	{ return (uint8_t)(x & 0x03u);	} // 2-bit
static inline uint8_t clip3(uint8_t x) 	{ return (uint8_t)(x & 0x07u); 	} // 3-bit
static inline uint8_t clip4(uint8_t x) 	{ return (uint8_t)(x & 0x0Fu); 	} // 4-bit

static dps310_t* getSelfStatePointer(sensor_if_t *self) {
	if (!self || !self->state) return NULL;
	return (dps310_t*)self->state;
}
