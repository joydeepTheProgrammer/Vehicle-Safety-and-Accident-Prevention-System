/**
 * @file gps_neo6m.c
 * @brief NEO-6M GPS Module driver implementation
 * @project Integrated Intelligent Vehicle Safety and Accident Prevention System
 *
 * Reads NMEA $GPRMC sentences via software UART (PD6=RX) at 9600 baud.
 *
 * $GPRMC sentence format:
 *   $GPRMC,HHMMSS.ss,A,LLLL.LL,a,YYYYY.YY,a,x.x,x.x,DDMMYY,x.x,a*hh<CR><LF>
 *   Fields:
 *     [0] = GPRMC identifier
 *     [1] = UTC time (HHMMSS.ss)
 *     [2] = Status (A=valid, V=invalid)
 *     [3] = Latitude  (DDMM.MMMM)
 *     [4] = N/S indicator
 *     [5] = Longitude (DDDMM.MMMM)
 *     [6] = E/W indicator
 *     [7] = Speed over ground (knots)
 *     [8] = Course over ground (degrees)
 *     [9] = Date (DDMMYY)
 *
 * Latitude conversion: decimal degrees = DD + MM.MMMM/60
 */

#include "gps_neo6m.h"
#include "../drivers/uart.h"
#include <util/delay.h>
#include <stdint.h>
#include <string.h>

/* ── NMEA receive buffer ─────────────────────────────────────────────────── */
static char s_nmea_buf[GPS_NMEA_BUF_SIZE];

/* ════════════════════════════════════════════════════════════════════════════
 *  Internal string-to-float (avoids linking full strtod on AVR)
 * ════════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Convert ASCII decimal string to float.
 *        Supports optional leading '-' and single decimal point.
 * @param s  Null-terminated numeric string.
 * @return Floating-point value.
 */
static float str_to_float(const char *s)
{
    float  result  = 0.0f;
    float  frac    = 0.0f;
    float  divisor = 1.0f;
    uint8_t negative = 0;
    uint8_t in_frac  = 0;

    if (*s == '-') { negative = 1; s++; }

    while (*s != '\0' && *s != ',') {
        if (*s == '.') {
            in_frac = 1;
        } else if (*s >= '0' && *s <= '9') {
            uint8_t digit = (uint8_t)(*s - '0');
            if (!in_frac) {
                result = result * 10.0f + (float)digit;
            } else {
                divisor *= 10.0f;
                frac    += (float)digit / divisor;
            }
        }
        s++;
    }
    result += frac;
    return negative ? -result : result;
}

/**
 * @brief Extract field N from a comma-delimited NMEA sentence into buf.
 * @param sentence  Full NMEA sentence string.
 * @param field_n   Zero-based field index.
 * @param buf       Output buffer.
 * @param buflen    Size of output buffer.
 */
static void nmea_get_field(const char *sentence, uint8_t field_n,
                            char *buf, uint8_t buflen)
{
    uint8_t cur_field = 0;
    uint8_t i = 0;

    while (*sentence && cur_field < field_n) {
        if (*sentence++ == ',') { cur_field++; }
    }

    while (*sentence && *sentence != ',' && *sentence != '*' && i < buflen - 1U) {
        buf[i++] = *sentence++;
    }
    buf[i] = '\0';
}

/**
 * @brief Convert NMEA DDDMM.MMMM format to decimal degrees.
 * @param nmea_val  Raw NMEA position value (e.g., "1234.5678").
 * @param dir       Direction char: 'N', 'S', 'E', 'W'.
 * @return Decimal degrees (negative for S/W).
 */
static float nmea_to_decimal_degrees(float nmea_val, char dir)
{
    /* nmea_val = DDDMM.MMMM (degrees × 100 + minutes) */
    int16_t deg = (int16_t)(nmea_val / 100.0f);
    float   min = nmea_val - (float)(deg * 100);
    float   decimal = (float)deg + (min / 60.0f);

    if (dir == 'S' || dir == 'W') { decimal = -decimal; }
    return decimal;
}

/* ════════════════════════════════════════════════════════════════════════════
 *  Receive one complete NMEA sentence
 * ════════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Read characters until a complete NMEA sentence is received.
 *        Blocks until '$' start found, then reads until LF.
 * @param buf     Output buffer.
 * @param buflen  Buffer size.
 * @return 1 if sentence received, 0 on buffer overflow.
 */
static uint8_t gps_read_sentence(char *buf, uint8_t buflen)
{
    char    c;
    uint8_t i = 0;

    /* Synchronise to start of NMEA sentence */
    do {
        c = soft_uart_gps_getchar();
    } while (c != '$');
    buf[i++] = c;

    /* Read until newline or buffer full */
    while (i < buflen - 1U) {
        c = soft_uart_gps_getchar();
        if (c == '\n') { break; }
        buf[i++] = c;
    }
    buf[i] = '\0';

    return (i < buflen - 1U) ? 1U : 0U;
}

/* ════════════════════════════════════════════════════════════════════════════
 *  Public API
 * ════════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Initialise NEO-6M GPS module software UART.
 */
void gps_init(void)
{
    soft_uart_gps_init();
    _delay_ms(1000);  /* Allow GPS to start streaming */
}

/**
 * @brief Read and parse one $GPRMC sentence from GPS.
 * @param data  Pointer to GPS_Data_t to populate.
 * @return GPS_OK if valid fix obtained.
 */
GPS_Status_t gps_read(GPS_Data_t *data)
{
    char field[GPS_FIELD_BUF_SIZE];
    uint8_t attempts = 20U;  /* Try up to 20 sentences to find $GPRMC */

    while (attempts--) {
        if (!gps_read_sentence(s_nmea_buf, GPS_NMEA_BUF_SIZE)) {
            continue;
        }

        /* Check if this is a $GPRMC sentence */
        if (strncmp(s_nmea_buf, "$GPRMC", 6U) != 0) {
            continue;
        }

        /* Field 2: Status (A=valid, V=void) */
        nmea_get_field(s_nmea_buf, 2U, field, GPS_FIELD_BUF_SIZE);
        if (field[0] != 'A') {
            data->fix_valid = 0;
            return GPS_ERR_NO_FIX;
        }
        data->fix_valid = 1U;

        /* Field 1: UTC Time (HHMMSS.ss) */
        nmea_get_field(s_nmea_buf, 1U, field, GPS_FIELD_BUF_SIZE);
        float utc = str_to_float(field);
        int32_t utc_int = (int32_t)utc;
        data->hour   = (uint8_t)(utc_int / 10000);
        data->minute = (uint8_t)((utc_int / 100) % 100);
        data->second = (uint8_t)(utc_int % 100);

        /* Field 3: Latitude */
        nmea_get_field(s_nmea_buf, 3U, field, GPS_FIELD_BUF_SIZE);
        float lat_raw = str_to_float(field);

        /* Field 4: N/S */
        nmea_get_field(s_nmea_buf, 4U, field, GPS_FIELD_BUF_SIZE);
        data->latitude = nmea_to_decimal_degrees(lat_raw, field[0]);

        /* Field 5: Longitude */
        nmea_get_field(s_nmea_buf, 5U, field, GPS_FIELD_BUF_SIZE);
        float lon_raw = str_to_float(field);

        /* Field 6: E/W */
        nmea_get_field(s_nmea_buf, 6U, field, GPS_FIELD_BUF_SIZE);
        data->longitude = nmea_to_decimal_degrees(lon_raw, field[0]);

        /* Field 7: Speed over ground (knots) → km/h */
        nmea_get_field(s_nmea_buf, 7U, field, GPS_FIELD_BUF_SIZE);
        data->speed_kmh = str_to_float(field) * GPS_SPEED_KNOTS_TO_KMH;

        /* Field 8: Course over ground */
        nmea_get_field(s_nmea_buf, 8U, field, GPS_FIELD_BUF_SIZE);
        data->course_deg = str_to_float(field);

        return GPS_OK;
    }

    return GPS_ERR_PARSE;
}

/**
 * @brief Convert lat/lon floats to ASCII strings for display.
 * @param lat      Latitude decimal degrees.
 * @param lon      Longitude decimal degrees.
 * @param lat_str  Output buffer for latitude string.
 * @param lon_str  Output buffer for longitude string.
 * @param buflen   Size of each output buffer.
 */
void gps_lat_lon_to_str(float lat, float lon,
                         char *lat_str, char *lon_str, uint8_t buflen)
{
    /* Format: ±DD.DDDD */
    for (uint8_t which = 0; which < 2U; which++) {
        float val = (which == 0) ? lat : lon;
        char *buf = (which == 0) ? lat_str : lon_str;
        uint8_t pos = 0;

        if (val < 0.0f) { buf[pos++] = '-'; val = -val; }

        int32_t int_part = (int32_t)val;
        int32_t dec_part = (int32_t)((val - (float)int_part) * 10000.0f);

        /* Integer part */
        if (int_part == 0) {
            buf[pos++] = '0';
        } else {
            char tmp[8]; int8_t ti = 0;
            int32_t v = int_part;
            while (v > 0) { tmp[ti++] = (char)('0' + v % 10); v /= 10; }
            while (ti > 0) buf[pos++] = tmp[--ti];
        }
        buf[pos++] = '.';
        /* Decimal part (4 digits, zero-padded) */
        for (int8_t d = 3; d >= 0; d--) {
            buf[pos + d] = (char)('0' + dec_part % 10);
            dec_part /= 10;
        }
        pos += 4;
        if (pos < buflen) buf[pos] = '\0';
    }
}
