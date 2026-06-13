/**
 * @file ldr.h
 * @brief LDR (Light Dependent Resistor) driver header – Auto Headlights
 * @project Integrated Intelligent Vehicle Safety and Accident Prevention System
 *
 * LDR output voltage is read via ADC.
 * High ADC count  => Low ambient light => Headlights ON
 * Low  ADC count  => Bright daylight   => Headlights OFF
 *
 * ADC channel: ADC4 (PC4) — shared with IR; since IR is digital and LDR is
 * analog, use a separate ADC channel. Assigned to ADC4.
 */

#ifndef LDR_H
#define LDR_H

#include <stdint.h>

/* ── ADC channel ─────────────────────────────────────────────────────────── */
#define LDR_ADC_CHANNEL     4U   /**< ADC4 / PC4 (analog, no conflict) */

/* ── Threshold ───────────────────────────────────────────────────────────── */
#define LDR_DARK_THRESHOLD  700U  /**< ADC counts; above => headlights ON */

/* ── Headlight relay ─────────────────────────────────────────────────────── */
#define HEADLIGHT_RELAY_DDR   DDRB
#define HEADLIGHT_RELAY_PORT  PORTB
#define HEADLIGHT_RELAY_PIN   PB4   /**< Active HIGH – drives relay module */

/* ── API ─────────────────────────────────────────────────────────────────── */
void     ldr_init(void);
uint16_t ldr_read_raw(void);
uint8_t  ldr_is_dark(void);
void     ldr_update_headlights(void);

#endif /* LDR_H */
