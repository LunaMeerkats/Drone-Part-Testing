// Core/Src/gps6m.c
#include "gps6m.h"
#include <stdio.h>      // printf
#include <stdlib.h>     // atoi, atof
#include <string.h>     // size_t
#include <ctype.h>      // isprint

/* ---- private types ---- */
typedef struct {
    int has_fix;      // 0/1
    int fix_quality;  // GGA field 6
    int sats_used;    // GGA field 7
    float hdop;       // GGA field 8
    char rmc_status;  // 'A' or 'V'
} gps_fix_status_t;

typedef enum { S_WAIT_DOLLAR = 0, S_G, S_GP_OR_GN } probe_state_t;

/* ---- private prototypes (static) ---- */
static HAL_StatusTypeDef rx1(UART_HandleTypeDef *u, uint8_t *c, uint32_t to_ms);
static void uart_sniff(UART_HandleTypeDef *u);
static HAL_StatusTypeDef nmea_probe_stream(UART_HandleTypeDef *u);

static int hexval(char c);
static int nmea_checksum_ok(const char *line);
static const char* nth_field(const char *s, int n);
static HAL_StatusTypeDef nmea_read_line(UART_HandleTypeDef *u, char *buf, size_t maxlen, uint32_t timeout_ms);
static void parse_status(const char *line, gps_fix_status_t *st);

/* ---- impl ---- */
static HAL_StatusTypeDef rx1(UART_HandleTypeDef *u, uint8_t *c, uint32_t to_ms) {
    return HAL_UART_Receive(u, c, 1, to_ms);
}

static void uart_sniff(UART_HandleTypeDef *u) {
    uint32_t t0 = HAL_GetTick();
    printf("[gps] sniffing 1s...\n");
    while (HAL_GetTick() - t0 < 1000) {
        uint8_t c;
        if (rx1(u, &c, 50) == HAL_OK) {
            printf("%c", (c >= 32 && c <= 126) ? c : '.');
        } else {
            printf("_");
        }
    }
    printf("\n");
}

static HAL_StatusTypeDef nmea_probe_stream(UART_HandleTypeDef *u) {
    probe_state_t st = S_WAIT_DOLLAR;
    uint32_t t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < 5000) {
        uint8_t c;
        if (rx1(u, &c, 50) != HAL_OK) continue;
        switch (st) {
            case S_WAIT_DOLLAR: if (c == '$') st = S_G; break;
            case S_G:           st = (c == 'G') ? S_GP_OR_GN : S_WAIT_DOLLAR; break;
            case S_GP_OR_GN:    if (c == 'P' || c == 'N') return HAL_OK; else st = S_WAIT_DOLLAR; break;
        }
    }
    return HAL_TIMEOUT;
}

/* ---- Read Values ---- */

static double parse_lat_ddmm(const char *f_lat, const char *f_ns) {
    if (!f_lat || !*f_lat || !f_ns || !*f_ns) return 0.0;
    double v = atof(f_lat);                 // ddmm.mmmm
    int deg = (int)(v / 100.0);
    double min = v - deg * 100.0;
    double dd = deg + (min / 60.0);
    if (*f_ns == 'S' || *f_ns == 's') dd = -dd;
    return dd;
}

static double parse_lon_dddmm(const char *f_lon, const char *f_ew) {
    if (!f_lon || !*f_lon || !f_ew || !*f_ew) return 0.0;
    double v = atof(f_lon);                 // dddmm.mmmm
    int deg = (int)(v / 100.0);
    double min = v - deg * 100.0;
    double dd = deg + (min / 60.0);
    if (*f_ew == 'W' || *f_ew == 'w') dd = -dd;
    return dd;
}

static float knots_to_mps(const char *f_knots) {
    if (!f_knots || !*f_knots) return 0.0f;
    return (float)(atof(f_knots) * 0.514444444);  // 1 knot = 0.514444... m/s
}

static void parse_RMC(const char *line, gps6m_t *s) {
    // $..RMC,1:time,2:status,3:lat,4:N/S,5:lon,6:E/W,7:sog(kn),8:cog,9:date,...
    if (!nmea_checksum_ok(line)) return;
    if (!(line[3]=='R' && line[4]=='M' && line[5]=='C')) return;

    const char *f_time = nth_field(line, 1);
    const char *f_stat = nth_field(line, 2);
    const char *f_lat  = nth_field(line, 3);
    const char *f_ns   = nth_field(line, 4);
    const char *f_lon  = nth_field(line, 5);
    const char *f_ew   = nth_field(line, 6);
    const char *f_sog  = nth_field(line, 7);
    const char *f_cog  = nth_field(line, 8);

    if (f_stat && *f_stat == 'A') { s->has_fix = 1; s->last_fix_ms = HAL_GetTick(); }
    if (f_time && *f_time) s->utc_hms = (uint32_t)atoi(f_time);  // e.g., 063911.00 -> 63911

    if (f_lat && f_ns && f_lon && f_ew) {
        s->lat_deg = parse_lat_ddmm(f_lat, f_ns);
        s->lon_deg = parse_lon_dddmm(f_lon, f_ew);
    }
    if (f_sog && *f_sog) s->speed_mps = knots_to_mps(f_sog);
    if (f_cog && *f_cog) s->course_deg = (float)atof(f_cog);
}

static void parse_GGA(const char *line, gps6m_t *s) {
    // $..GGA,1:time,2:lat,3:N/S,4:lon,5:E/W,6:fix,7:sats,8:hdop,9:alt,10:M,...
    if (!nmea_checksum_ok(line)) return;
    if (!(line[3]=='G' && line[4]=='G' && line[5]=='A')) return;

    const char *f_time = nth_field(line, 1);
    const char *f_lat  = nth_field(line, 2);
    const char *f_ns   = nth_field(line, 3);
    const char *f_lon  = nth_field(line, 4);
    const char *f_ew   = nth_field(line, 5);
    const char *f_fix  = nth_field(line, 6);
    const char *f_sats = nth_field(line, 7);
    const char *f_hdop = nth_field(line, 8);
    const char *f_alt  = nth_field(line, 9);

    if (f_time && *f_time) s->utc_hms = (uint32_t)atoi(f_time);
    if (f_fix && *f_fix && atoi(f_fix) > 0) { s->has_fix = 1; s->last_fix_ms = HAL_GetTick(); }

    if (f_lat && f_ns && f_lon && f_ew) {
        s->lat_deg = parse_lat_ddmm(f_lat, f_ns);
        s->lon_deg = parse_lon_dddmm(f_lon, f_ew);
    }
    if (f_sats && *f_sats) s->sats_used = (uint8_t)atoi(f_sats);
    if (f_hdop && *f_hdop) s->hdop      = (float)atof(f_hdop);
    if (f_alt  && *f_alt)  s->alt_m     = (float)atof(f_alt);
}

/* ---- small NMEA helpers ---- */
static int hexval(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

static int nmea_checksum_ok(const char *line) {
    if (!line || line[0] != '$') return 0;
    uint8_t sum = 0;
    const char *p = line + 1, *star = NULL;
    while (*p && *p != '\r' && *p != '\n') {
        if (*p == '*') { star = p; break; }
        sum ^= (uint8_t)*p++;
    }
    if (!star || !star[1] || !star[2]) return 0;
    int hi = hexval(star[1]), lo = hexval(star[2]);
    if (hi < 0 || lo < 0) return 0;
    return sum == (uint8_t)((hi << 4) | lo);
}

static const char* nth_field(const char *s, int n) {
    int idx = 0;
    if (!s) return NULL;
    if (n == 0) return s;
    while (*s && *s != '\n' && *s != '\r') {
        if (*s == ',') {
            idx++;
            if (idx == n) return s + 1;
        }
        s++;
    }
    return NULL;
}

static HAL_StatusTypeDef nmea_read_line(UART_HandleTypeDef *u, char *buf, size_t maxlen, uint32_t timeout_ms) {
    size_t i = 0;
    uint32_t t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < timeout_ms) {
        uint8_t c;
        if (HAL_UART_Receive(u, &c, 1, 50) != HAL_OK) continue;
        if (i == 0 && c != '$') continue;
        if (i < maxlen - 1) buf[i++] = (char)c;
        if (c == '\n') { buf[i] = 0; return HAL_OK; }
    }
    return HAL_TIMEOUT;
}

static void parse_status(const char *line, gps_fix_status_t *st) {
    if (!st || !nmea_checksum_ok(line)) return;
    if (line[3]=='G' && line[4]=='G' && line[5]=='A') {
        const char *f6 = nth_field(line, 6);
        const char *f7 = nth_field(line, 7);
        const char *f8 = nth_field(line, 8);
        if (f6 && *f6) st->fix_quality = atoi(f6);
        if (f7 && *f7) st->sats_used   = atoi(f7);
        if (f8 && *f8) st->hdop        = (float)atof(f8);
        st->has_fix |= st->fix_quality > 0;
    } else if (line[3]=='R' && line[4]=='M' && line[5]=='C') {
        const char *f2 = nth_field(line, 2);
        if (f2 && *f2) st->rmc_status = *f2;   // 'A' or 'V'
        if (st->rmc_status == 'A') st->has_fix = 1;
    }
}

/* ---- vtable ops ---- */
static HAL_StatusTypeDef gps_probe(sensor_if_t *self) {
    gps6m_t *s = (gps6m_t*)self->state;
    printf("[gps] probe start (UART=%p)\n", (void*)s->uart);
    uart_sniff(s->uart);
    HAL_StatusTypeDef st = nmea_probe_stream(s->uart);
    printf("[gps] probe done: %ld\n", (long)st);
    return st;
}

static HAL_StatusTypeDef gps_init(sensor_if_t *self) {
    gps6m_t *s = (gps6m_t*)self->state;
    char line[128];
    gps_fix_status_t fs = {0};
    uint32_t t0 = HAL_GetTick();
    const uint32_t MAX_WAIT = 30000;

    printf("[gps] waiting for fix (%lu ms)\n", (unsigned long)MAX_WAIT);
    while ((HAL_GetTick() - t0) < MAX_WAIT) {
        if (nmea_read_line(s->uart, line, sizeof line, 1200) != HAL_OK) continue;
        parse_status(line, &fs);
        if (line[3]=='G' && line[4]=='G' && line[5]=='A') {
            printf("[gps] GGA fix=%d sats=%d hdop=%.2f\n", fs.fix_quality, fs.sats_used, fs.hdop);
        } else if (line[3]=='R' && line[4]=='M' && line[5]=='C') {
            printf("[gps] RMC status=%c\n", fs.rmc_status ? fs.rmc_status : '?');
        }
        if (fs.has_fix && fs.sats_used >= 4) {
            printf("[gps] fix acquired\n");
            return HAL_OK;
        }
    }
    printf("[gps] no fix within window\n");
    return HAL_TIMEOUT;
}

static HAL_StatusTypeDef gps_read(sensor_if_t *self) {
    gps6m_t *s = (gps6m_t*)self->state;
    char line[128];
    uint32_t t0 = HAL_GetTick();
    const uint32_t BUDGET_MS   = 200;   // keep it snappy
    const uint32_t FIX_HOLD_MS = 2000;  // keep fix valid for 2 s since last confirmed fix

    int got_any = 0;

    while ((HAL_GetTick() - t0) < BUDGET_MS) {
        if (nmea_read_line(s->uart, line, sizeof line, 80) != HAL_OK) break;
        got_any = 1;

        // classify & parse
        if (line[0] == '$') {
            if (line[3]=='R' && line[4]=='M' && line[5]=='C') {
                parse_RMC(line, s);
            } else if (line[3]=='G' && line[4]=='G' && line[5]=='A') {
                parse_GGA(line, s);
            }
        }
    }

    if (got_any) s->last_update_ms = HAL_GetTick();

    // AGE-OUT: only drop fix if we’ve gone too long without confirming it
    if (s->has_fix) {
        if ((HAL_GetTick() - s->last_fix_ms) > FIX_HOLD_MS) {
            s->has_fix = 0;   // stale fix; keep lat/lon/alt but mark invalid
        }
    }

    return HAL_OK;
}



static HAL_StatusTypeDef gps_whoami(sensor_if_t *self, uint8_t *out) {
    gps6m_t *s = (gps6m_t*)self->state;
    if (nmea_probe_stream(s->uart) == HAL_OK) { if (out) *out = 'G'; return HAL_OK; }
    return HAL_ERROR;
}

/* ---- bind ---- */
static const sensor_if_vtbl_t GPS_VTBL = {
    .probe  = gps_probe,
    .init   = gps_init,
    .read   = gps_read,
    .whoami = gps_whoami
};

void gps6m_bind(sensor_if_t *iface, gps6m_t *inst, UART_HandleTypeDef *uart) {
    memset(inst, 0, sizeof(*inst));   // ensure clean state
    inst->uart   = uart;
    iface->vTable= &GPS_VTBL;
    iface->state = inst;
}

