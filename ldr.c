/**
 * @file ldr.c
 * @brief LDR Automatic Headlight driver implementation
 * @project Integrated Intelligent Vehicle Safety and Accident Prevention System
 *
 * ADC4 (PC4) reads LDR voltage divider output.
 * Headlight relay on PB4 (active HIGH).
 */

#include "ldr.h"
#include <avr/io.h>
#include <stdint.h>

/* ── Internal: ADC read (shared ADC, already initialised by mq3_init) ────── */
static uint16_t adc_read_ch(uint8_t channel)
{
    ADMUX = (ADMUX & 0xF0U) | (channel & 0x0FU);
    ADCSRA |= (1U << ADSC);
    while (ADCSRA & (1U << ADSC)) {}
    return ADC;
}

/**
 * @brief Initialise LDR ADC channel and headlight relay pin.
 *        Assumes adc_init_shared() was already called by mq3_init().
 */
void ldr_init(void)
{
    /* PC4 as analog input (clear DDR bit, no pull-up) */
    DDRC  &= ~(1U << PC4);
    PORTC &= ~(1U << PC4);

    /* Headlight relay pin: output, initially OFF */
    HEADLIGHT_RELAY_DDR  |=  (1U << HEADLIGHT_RELAY_PIN);
    HEADLIGHT_RELAY_PORT &= ~(1U << HEADLIGHT_RELAY_PIN);
}

/**
 * @brief Read raw ADC value from LDR.
 * @return 10-bit ADC count.
 */
uint16_t ldr_read_raw(void)
{
    return adc_read_ch(LDR_ADC_CHANNEL);
}

/**
 * @brief Check if ambient light is below dark threshold.
 * @return 1 if dark (headlights needed), 0 if bright.
 */
uint8_t ldr_is_dark(void)
{
    return (ldr_read_raw() > LDR_DARK_THRESHOLD) ? 1U : 0U;
}

/**
 * @brief Update headlight relay state based on LDR reading.
 *        Call periodically from main loop.
 */
void ldr_update_headlights(void)
{
    if (ldr_is_dark()) {
        HEADLIGHT_RELAY_PORT |=  (1U << HEADLIGHT_RELAY_PIN);  /* ON  */
    } else {
        HEADLIGHT_RELAY_PORT &= ~(1U << HEADLIGHT_RELAY_PIN);  /* OFF */
    }
}
