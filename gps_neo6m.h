/**
 * @file gps_neo6m.h
 * @brief NEO-6M GPS Module driver header
 * @project Integrated Intelligent Vehicle Safety and Accident Prevention System
 *
 * Parses NMEA $GPRMC sentences from GPS module at 9600 baud.
 * Software UART: PD5=TX (to GPS RX), PD6=RX (from GPS TX).
 * Provides latitude, longitude, speed (knots), and fix validity.
 */

#ifndef GPS_NEO6M_H
#define GPS_NEO6M_H

#include <stdint.h>

/* ── NMEA buffer sizes ───────────────────────────────────────────────────── */
#define GPS_NMEA_BUF_SIZE   96U
#define GPS_FIELD_BUF_SIZE  16U

/* ── Speed thresholds for drowsiness detection ───────────────────────────── */
#define GPS_SPEED_KNOTS_TO_KMH   1.852f
#define GPS_MIN_SPEED_KMH        10.0f    /**< Below this: stop, no drowsiness check */
#define GPS_DROWSY_SPEED_DELTA   5.0f     /**< Speed variance below this triggers check */

/* ── GPS data structure ──────────────────────────────────────────────────── */
typedef struct {
    float   latitude;       /**< Decimal degrees, positive = North */
    float   longitude;      /**< Decimal degrees, positive = East  */
    float   speed_kmh;      /**< Speed over ground in km/h         */
    float   course_deg;     /**< Course over ground in degrees      */
    uint8_t fix_valid;      /**< 1 = valid fix, 0 = no fix          */
    uint8_t satellites;     /**< Number of satellites in use        */
    uint8_t hour;           /**< UTC hour                           */
    uint8_t minute;         /**< UTC minute                         */
    uint8_t second;         /**< UTC second                         */
} GPS_Data_t;

/* ── Return codes ────────────────────────────────────────────────────────── */
typedef enum {
    GPS_OK = 0,
    GPS_ERR_NO_FIX,
    GPS_ERR_PARSE
} GPS_Status_t;

/* ── API ─────────────────────────────────────────────────────────────────── */
void         gps_init(void);
GPS_Status_t gps_read(GPS_Data_t *data);
void         gps_lat_lon_to_str(float lat, float lon,
                                char *lat_str, char *lon_str,
                                uint8_t buflen);

#endif /* GPS_NEO6M_H */
