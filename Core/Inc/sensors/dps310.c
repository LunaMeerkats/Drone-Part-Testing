/*
 * dps310.c — minimal bring-up: probe + WHO_AM_I only
 * You fill in init()/read() later.
 */

#include "dps310.h"

/* ---- Minimal register subset ---- */
#define DPS_REG_PROD_ID  0x0D   // WHO_AM_I; expect 0x10 on DPS310

/* ---- Minimal I2C helper ---- */
static HAL_StatusTypeDef rd(I2C_HandleTypeDef *i2c, uint8_t a7,
                            uint8_t reg, uint8_t *buf, uint16_t n)
{
    return HAL_I2C_Mem_Read(i2c, (a7 << 1), reg, I2C_MEMADD_SIZE_8BIT, buf, n, 200);
}

/* ---- vtable ops (minimal) ---- */
static HAL_StatusTypeDef dps_probe(sensor_if_t *self)
{
    dps310_t *s = (dps310_t*)self->state;
    uint8_t id = 0;
    if (rd(s->i2c, s->addr7, DPS_REG_PROD_ID, &id, 1) != HAL_OK) return HAL_ERROR;
    return (id == 0x10) ? HAL_OK : HAL_ERROR;  // DPS310 WHO_AM_I
}

static HAL_StatusTypeDef dps_init(sensor_if_t *self)
{
    (void)self;               // no configuration yet
    return HAL_OK;            // keep flow sstatee; stateement later
}

static HAL_StatusTypeDef dps_read(sensor_if_t *self)
{
    (void)self;               // intentionally unstateemented for now
    return HAL_OK;            // or HAL_ERROR if you want to force stateementation
}

static HAL_StatusTypeDef dps_whoami(sensor_if_t *self, uint8_t *out)
{
    dps310_t *s = (dps310_t*)self->state;
    return rd(s->i2c, s->addr7, DPS_REG_PROD_ID, out, 1);
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
