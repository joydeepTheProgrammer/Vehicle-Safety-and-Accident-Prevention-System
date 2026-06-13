/**
 * @file dht11.c
 * @brief DHT11 Temperature & Humidity Sensor driver implementation
 * @project Integrated Intelligent Vehicle Safety and Accident Prevention System
 *
 * Single-wire protocol timing (all in µs):
 *   Host pull-down: ≥18 ms
 *   Host release:   20–40 µs
 *   DHT response:   80 µs LOW + 80 µs HIGH
 *   Data '0':       50 µs LOW + 26–28 µs HIGH
 *   Data '1':       50 µs LOW + 70 µs HIGH
 *
 * Total payload: 40 bits = 8 humidity int + 8 humidity dec +
 *                          8 temp int + 8 temp dec + 8 checksum
 */

#include "dht11.h"
#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>

/* ── Timing constants (µs) ───────────────────────────────────────────────── */
#define DHT11_START_LOW_MS   18U    /* Host start signal: pull LOW ≥18 ms */
#define DHT11_RESPONSE_WAIT  100U   /* Max wait for DHT response pulse     */
#define DHT11_BIT_TIMEOUT    100U   /* Timeout per bit                     */

/* ── Pin direction helpers ───────────────────────────────────────────────── */
#define DHT11_OUTPUT()  do { DHT11_DDR |=  (1U << DHT11_PIN); } while (0)
#define DHT11_INPUT()   do { DHT11_DDR &= ~(1U << DHT11_PIN); } while (0)
#define DHT11_HIGH()    do { DHT11_PORT |=  (1U << DHT11_PIN); } while (0)
#define DHT11_LOW()     do { DHT11_PORT &= ~(1U << DHT11_PIN); } while (0)
#define DHT11_READ()    (DHT11_PINR & (1U << DHT11_PIN))

/* ════════════════════════════════════════════════════════════════════════════
 *  Public API
 * ════════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Initialise DHT11 data pin as input with pull-up (idle HIGH).
 */
void dht11_init(void)
{
    DHT11_INPUT();
    DHT11_HIGH();  /* Enable pull-up */
    _delay_ms(1000); /* Allow sensor to stabilise after power-on */
}

/**
 * @brief Read temperature and humidity from DHT11.
 *
 * @param data  Pointer to DHT11_Data_t to populate.
 * @return DHT11_OK on success, error code otherwise.
 */
DHT11_Status_t dht11_read(DHT11_Data_t *data)
{
    uint8_t  raw[5] = {0, 0, 0, 0, 0};
    uint16_t timeout;

    /* ── Step 1: Host sends START signal ─────────────────────────────────── */
    DHT11_OUTPUT();
    DHT11_LOW();
    _delay_ms(DHT11_START_LOW_MS);
    DHT11_HIGH();
    _delay_us(30);

    /* ── Step 2: Switch to input, wait for DHT response ──────────────────── */
    DHT11_INPUT();

    /* Wait for DHT to pull LOW (response start) */
    timeout = DHT11_RESPONSE_WAIT;
    while (DHT11_READ() && timeout--) {
        _delay_us(1);
    }
    if (timeout == 0) { return DHT11_ERR_TIMEOUT; }

    /* Wait for DHT to release (pull HIGH = 80 µs) */
    timeout = DHT11_RESPONSE_WAIT;
    while (!DHT11_READ() && timeout--) {
        _delay_us(1);
    }
    if (timeout == 0) { return DHT11_ERR_TIMEOUT; }

    /* Wait for end of 80 µs HIGH response pulse */
    timeout = DHT11_RESPONSE_WAIT;
    while (DHT11_READ() && timeout--) {
        _delay_us(1);
    }
    if (timeout == 0) { return DHT11_ERR_TIMEOUT; }

    /* ── Step 3: Read 40 bits (5 bytes) ──────────────────────────────────── */
    for (uint8_t byte = 0; byte < 5U; byte++) {
        for (int8_t bit = 7; bit >= 0; bit--) {
            /* Wait for 50 µs LOW pre-bit pulse */
            timeout = DHT11_BIT_TIMEOUT;
            while (!DHT11_READ() && timeout--) {
                _delay_us(1);
            }
            if (timeout == 0) { return DHT11_ERR_TIMEOUT; }

            /* Measure HIGH duration to determine bit value */
            _delay_us(40);  /* Sample at 40 µs into HIGH phase */
            if (DHT11_READ()) {
                /* HIGH after 40 µs → bit = 1 (≈70 µs HIGH) */
                raw[byte] |= (1U << (uint8_t)bit);
                /* Wait for end of HIGH phase */
                timeout = DHT11_BIT_TIMEOUT;
                while (DHT11_READ() && timeout--) {
                    _delay_us(1);
                }
            }
            /* else: bit = 0 (≈26 µs HIGH, already finished) */
        }
    }

    /* ── Step 4: Verify checksum ─────────────────────────────────────────── */
    uint8_t checksum = (uint8_t)(raw[0] + raw[1] + raw[2] + raw[3]);
    if (checksum != raw[4]) {
        return DHT11_ERR_CHECKSUM;
    }

    /* ── Step 5: Populate output structure ───────────────────────────────── */
    data->humidity_pct    = raw[0];   /* Integer part of humidity    */
    data->temperature_c   = raw[2];   /* Integer part of temperature */

    return DHT11_OK;
}
