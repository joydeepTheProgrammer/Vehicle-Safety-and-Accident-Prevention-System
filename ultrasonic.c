/**
 * @file ultrasonic.c
 * @brief HC-SR04 Ultrasonic Sensor driver implementation
 * @project Integrated Intelligent Vehicle Safety and Accident Prevention System
 *
 * Measures distance using HC-SR04 sensors on 4 vehicle sides.
 * Algorithm:
 *   1. Pull TRIG HIGH for 10 µs, then LOW.
 *   2. Measure ECHO pulse width in µs.
 *   3. Distance (cm) = pulse_width_µs / 58.
 *
 * Timeout: If ECHO does not go HIGH within 30 ms, returns US_TIMEOUT_CM.
 * Timer1 is used as a free-running µs counter (prescaler = 8 → 0.5 µs/tick
 * at 16 MHz → scale by 2 for µs).
 *
 * @note All four sensors are read sequentially to avoid cross-talk.
 */

#include "ultrasonic.h"
#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>

/* ── Timer1 helpers ──────────────────────────────────────────────────────── */
/* Timer1 configured as free-running counter, prescaler=8 → 0.5 µs/tick    */
#define TIMER1_START()  do { TCCR1B = (1U << CS11); } while (0)
#define TIMER1_STOP()   do { TCCR1B = 0; } while (0)
#define TIMER1_RESET()  do { TCNT1 = 0; } while (0)
#define TIMER1_VAL()    (TCNT1)

/* Timeout: 30 ms = 30000 µs / 0.5 µs per tick = 60000 ticks */
#define ECHO_TIMEOUT_TICKS  60000U

/* ── Pin helper macros ───────────────────────────────────────────────────── */
#define TRIG_HIGH(port, pin)  do { (port) |=  (1U << (pin)); } while (0)
#define TRIG_LOW(port, pin)   do { (port) &= ~(1U << (pin)); } while (0)

/* ════════════════════════════════════════════════════════════════════════════
 *  Initialisation
 * ════════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Configure TRIG (output) and ECHO (input) pins for all 4 sensors.
 *        Also initialises Timer1 in normal mode with prescaler=8.
 */
void ultrasonic_init(void)
{
    /* FRONT */
    US_FRONT_TRIG_DDR  |=  (1U << US_FRONT_TRIG_PIN);  /* TRIG = output */
    US_FRONT_TRIG_PORT &= ~(1U << US_FRONT_TRIG_PIN);  /* TRIG idle LOW */
    US_FRONT_ECHO_DDR  &= ~(1U << US_FRONT_ECHO_PIN);  /* ECHO = input  */

    /* REAR */
    US_REAR_TRIG_DDR   |=  (1U << US_REAR_TRIG_PIN);
    US_REAR_TRIG_PORT  &= ~(1U << US_REAR_TRIG_PIN);
    US_REAR_ECHO_DDR   &= ~(1U << US_REAR_ECHO_PIN);

    /* LEFT */
    US_LEFT_TRIG_DDR   |=  (1U << US_LEFT_TRIG_PIN);
    US_LEFT_TRIG_PORT  &= ~(1U << US_LEFT_TRIG_PIN);
    US_LEFT_ECHO_DDR   &= ~(1U << US_LEFT_ECHO_PIN);

    /* RIGHT */
    US_RIGHT_TRIG_DDR  |=  (1U << US_RIGHT_TRIG_PIN);
    US_RIGHT_TRIG_PORT &= ~(1U << US_RIGHT_TRIG_PIN);
    US_RIGHT_ECHO_DDR  &= ~(1U << US_RIGHT_ECHO_PIN);

    /* Timer1: Normal mode, no output compare, prescaler=8 */
    TCCR1A = 0;
    TCCR1B = 0;  /* Stopped initially */
    TIMSK1 = 0;  /* No interrupts     */
}

/* ════════════════════════════════════════════════════════════════════════════
 *  Internal: read distance for a specific sensor
 * ════════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Trigger one measurement and return pulse width in Timer1 ticks.
 *        Each tick = 0.5 µs at 16 MHz / prescaler 8.
 *
 * @param trig_port  Pointer to TRIG port register (PORTx).
 * @param trig_pin   TRIG pin number.
 * @param echo_pinr  Pointer to ECHO PIN register (PINx).
 * @param echo_pin   ECHO pin number.
 * @return Pulse width in Timer1 ticks, or 0xFFFF on timeout.
 */
static uint16_t measure_pulse(volatile uint8_t *trig_port, uint8_t trig_pin,
                               volatile uint8_t *echo_pinr, uint8_t echo_pin)
{
    uint16_t ticks;

    /* Send 10 µs trigger pulse */
    *trig_port |=  (1U << trig_pin);
    _delay_us(10);
    *trig_port &= ~(1U << trig_pin);

    /* Wait for ECHO to go HIGH (start of pulse) with timeout */
    TIMER1_RESET();
    TIMER1_START();
    while (!(*echo_pinr & (1U << echo_pin))) {
        if (TIMER1_VAL() > ECHO_TIMEOUT_TICKS) {
            TIMER1_STOP();
            return US_TIMEOUT_CM;   /* Reuse timeout sentinel for early exit */
        }
    }

    /* Measure ECHO pulse width */
    TIMER1_RESET();
    while (*echo_pinr & (1U << echo_pin)) {
        if (TIMER1_VAL() > ECHO_TIMEOUT_TICKS) {
            TIMER1_STOP();
            return US_TIMEOUT_CM;
        }
    }
    ticks = TIMER1_VAL();
    TIMER1_STOP();

    return ticks;
}

/* ════════════════════════════════════════════════════════════════════════════
 *  Public API
 * ════════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Read distance from one ultrasonic sensor.
 *
 * Formula: distance_cm = (ticks × 0.5) / 58  →  ticks / 116
 * (Speed of sound ≈ 343 m/s; round-trip factor = 2; 1 cm = 29 µs one-way)
 *
 * @param sensor  USSensor_t sensor identifier.
 * @return Distance in cm, or US_TIMEOUT_CM (0xFFFF) if no echo / out of range.
 */
uint16_t ultrasonic_read_cm(USSensor_t sensor)
{
    uint16_t ticks;

    /* 60 ms between reads to avoid cross-sensor echo interference */
    _delay_ms(60);

    switch (sensor) {
        case US_FRONT:
            ticks = measure_pulse(&US_FRONT_TRIG_PORT, US_FRONT_TRIG_PIN,
                                  &US_FRONT_ECHO_PINR, US_FRONT_ECHO_PIN);
            break;
        case US_REAR:
            ticks = measure_pulse(&US_REAR_TRIG_PORT, US_REAR_TRIG_PIN,
                                  &US_REAR_ECHO_PINR, US_REAR_ECHO_PIN);
            break;
        case US_LEFT:
            ticks = measure_pulse(&US_LEFT_TRIG_PORT, US_LEFT_TRIG_PIN,
                                  &US_LEFT_ECHO_PINR, US_LEFT_ECHO_PIN);
            break;
        case US_RIGHT:
            ticks = measure_pulse(&US_RIGHT_TRIG_PORT, US_RIGHT_TRIG_PIN,
                                  &US_RIGHT_ECHO_PINR, US_RIGHT_ECHO_PIN);
            break;
        default:
            return US_TIMEOUT_CM;
    }

    if (ticks == US_TIMEOUT_CM) {
        return US_TIMEOUT_CM;
    }

    /* Convert ticks to cm: ticks × 0.5 µs / 58 µs/cm = ticks / 116 */
    return (uint16_t)(ticks / 116U);
}
