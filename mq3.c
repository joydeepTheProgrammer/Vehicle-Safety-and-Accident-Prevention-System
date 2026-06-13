/**
 * @file mq3.c
 * @brief MQ-3 Alcohol Sensor driver implementation
 * @project Integrated Intelligent Vehicle Safety and Accident Prevention System
 *
 * ADC5 (PC5) is used for MQ-3 analog output.
 * ATmega328P ADC: 10-bit, Vref = AVCC = 5V.
 * ADC clock = F_CPU / 128 = 125 kHz (prescaler = 128).
 *
 * Warm-up detection: uses a millisecond counter (set by Timer0 ISR in main.c)
 * The mq3_warmup_ms counter is defined here and incremented externally.
 */

#include "mq3.h"
#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>

/* ── Internal warm-up counter (ms elapsed since power-on) ───────────────── */
/* Incremented by the Timer0 overflow ISR in main.c every 1 ms             */
volatile uint32_t g_system_ms = 0;   /* Shared with main.c */
static   uint32_t s_mq3_start_ms = 0;

/* ════════════════════════════════════════════════════════════════════════════
 *  Internal ADC helpers
 * ════════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Initialise ADC with AVCC reference and prescaler=128.
 *        Must only be called once (shared with ldr.c – both call adc_init,
 *        but the register writes are idempotent).
 */
static void adc_init_shared(void)
{
    /* Reference: AVCC with external capacitor on AREF */
    ADMUX = (1U << REFS0);

    /* ADC Enable, Start Conversion, Auto-trigger off,
     * ADC Interrupt disabled, Prescaler = 128 (16MHz/128 = 125kHz) */
    ADCSRA = (1U << ADEN)  |
             (1U << ADPS2) |
             (1U << ADPS1) |
             (1U << ADPS0);
}

/**
 * @brief Read a 10-bit ADC value from the specified channel (0–7).
 * @param channel  ADC channel number.
 * @return 10-bit ADC reading (0–1023).
 */
static uint16_t adc_read(uint8_t channel)
{
    /* Select channel (preserve REFS bits) */
    ADMUX = (ADMUX & 0xF0U) | (channel & 0x0FU);

    /* Start single conversion */
    ADCSRA |= (1U << ADSC);

    /* Wait for conversion complete */
    while (ADCSRA & (1U << ADSC)) {}

    return ADC;
}

/* ════════════════════════════════════════════════════════════════════════════
 *  Public API
 * ════════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Initialise MQ-3 sensor.
 *        Records start timestamp for warm-up tracking.
 */
void mq3_init(void)
{
    adc_init_shared();
    s_mq3_start_ms = g_system_ms;
    /* PC5 as input (ADC5) – DDR default is input, ensure no pull-up */
    DDRC  &= ~(1U << PC5);
    PORTC &= ~(1U << PC5);
}

/**
 * @brief Read raw ADC value from MQ-3 sensor.
 * @return 10-bit ADC count (0–1023).
 */
uint16_t mq3_read_raw(void)
{
    return adc_read(MQ3_ADC_CHANNEL);
}

/**
 * @brief Get alcohol level status.
 * @return MQ3_Status_t status code.
 */
MQ3_Status_t mq3_get_status(void)
{
    uint16_t raw = mq3_read_raw();

    if (raw >= MQ3_DANGER_THRESHOLD) {
        return MQ3_DANGER;
    } else if (raw >= MQ3_WARN_THRESHOLD) {
        return MQ3_WARN;
    }
    return MQ3_CLEAN;
}

/**
 * @brief Check if MQ-3 warm-up period has elapsed.
 * @return 1 if warmed up, 0 if still warming.
 */
uint8_t mq3_is_warmed_up(void)
{
    return ((g_system_ms - s_mq3_start_ms) >= MQ3_WARMUP_MS) ? 1U : 0U;
}
