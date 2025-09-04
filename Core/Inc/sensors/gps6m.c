/*
 * gps6m.c
 *
 *  Created on: Sep 3, 2025
 *      Author: drrsm
 */

/*
 * gps6m.c — minimal bring-up for GY-GPS6MV2 (u-blox NEO-6M)
 * Only checks that NMEA data ($GPxx / $GNxx) is arriving on UART.
 */

#include "gps6m.h"

/* receive one byte with a short timeout */
static HAL_StatusTypeDef rx1(UART_HandleTypeDef *u, uint8_t *c, uint32_t to_ms) {
    return HAL_UART_Receive(u, c, 1, to_ms);
}

/* very small state machine: look for "$G[P|N]" within ~1.5 s */
static HAL_StatusTypeDef nmea_probe_stream(UART_HandleTypeDef *u) {
    uint32_t t0 = HAL_GetTick();
    enum { S_WAIT_DOLLAR, S_G, S_GP_OR_GN } st = S_WAIT_DOLLAR;
    while ((HAL_GetTick() - t0) < 1500) {
        uint8_t c;
        if (rx1(u, &c, 50) != HAL_OK) continue;    // keep polling
        switch (st) {
            case S_WAIT_DOLLAR: if (c == '$') st = S_G; break;
            case S_G:           st = (c == 'G') ? S_GP_OR_GN : S_WAIT_DOLLAR; break;
            case S_GP_OR_GN:    if (c == 'P' || c == 'N') return HAL_OK; else st = S_WAIT_DOLLAR; break;
        }
    }
    return HAL_TIMEOUT;
}

/* -------- vtable ops (minimal) -------- */
static HAL_StatusTypeDef gps_probe(sensor_if_t *self) {
    gps6m_t *s = (gps6m_t*)self->state;
    return nmea_probe_stream(s->uart);  // OK if we see $GP/$GN
}

static HAL_StatusTypeDef gps_init(sensor_if_t *self) {
    (void)self;          // nothing yet
    return HAL_OK;
}

static HAL_StatusTypeDef gps_read(sensor_if_t *self) {
    (void)self;          // intentionally unstateemented for now
    return HAL_OK;
}

static HAL_StatusTypeDef gps_whoami(sensor_if_t *self, uint8_t *out) {
    // Not really meaningful for GPS; return 'G' if we see a '$G?'
    gps6m_t *s = (gps6m_t*)self->state;
    if (nmea_probe_stream(s->uart) == HAL_OK) { if (out) *out = 'G'; return HAL_OK; }
    return HAL_ERROR;
}

/* -------- bind -------- */
static const sensor_if_vtbl_t GPS_VTBL = {
    .probe  = gps_probe,
    .init   = gps_init,
    .read   = gps_read,
    .whoami = gps_whoami
};

void gps6m_bind(sensor_if_t *iface, gps6m_t *inst, UART_HandleTypeDef *uart) {
    inst->uart = uart;
    iface->vTable   = &GPS_VTBL;
    iface->state= inst;
}



