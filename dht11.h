/**
 * @file dht11.h
 * @brief DHT11 Temperature & Humidity sensor driver header
 * @project Integrated Intelligent Vehicle Safety and Accident Prevention System
 *
 * Single-wire protocol.  Data pin: PD7.
 * Resolution: Temperature 1°C, Humidity 1% RH.
 * Min read interval: 2 seconds.
 */

#ifndef DHT11_H
#define DHT11_H

#include <stdint.h>
#include <avr/io.h>

/* ── Pin assignment ──────────────────────────────────────────────────────── */
#define DHT11_DDR    DDRD
#define DHT11_PORT   PORTD
#define DHT11_PINR   PIND
#define DHT11_PIN    PD7

/* ── Overheat thresholds ─────────────────────────────────────────────────── */
#define DHT11_TEMP_WARN_C    55U   /**< Warning temperature (°C) */
#define DHT11_TEMP_DANGER_C  65U   /**< Danger: possible fire, evacuate */

/* ── Return status ───────────────────────────────────────────────────────── */
typedef enum {
    DHT11_OK = 0,
    DHT11_ERR_TIMEOUT,
    DHT11_ERR_CHECKSUM
} DHT11_Status_t;

/* ── Data structure ──────────────────────────────────────────────────────── */
typedef struct {
    uint8_t temperature_c;   /**< Temperature in Celsius        */
    uint8_t humidity_pct;    /**< Relative humidity in percent  */
} DHT11_Data_t;

/* ── API ─────────────────────────────────────────────────────────────────── */
void           dht11_init(void);
DHT11_Status_t dht11_read(DHT11_Data_t *data);

#endif /* DHT11_H */
