/**
 * @file mq3.h
 * @brief MQ-3 Alcohol Gas Sensor driver header
 * @project Integrated Intelligent Vehicle Safety and Accident Prevention System
 *
 * MQ-3 output is an analog voltage proportional to alcohol concentration.
 * ADC channel: ADC_CHANNEL_MQ3 (ADC0 / PC0 … here mapped to ADC2 to avoid
 * conflict with ultrasonic echo pins on PC0-PC3).
 *
 * NOTE: Sensor requires ~30-second warm-up after power-on before readings
 * are stable.  The driver exposes a warm-up check.
 */

#ifndef MQ3_H
#define MQ3_H

#include <stdint.h>

/* ── ADC channel ─────────────────────────────────────────────────────────── */
#define MQ3_ADC_CHANNEL     5U   /**< ADC5 / PC5 */

/* ── Calibration thresholds (ADC counts, 10-bit, Vref=5V) ───────────────── */
/* 
 * Rs/R0 ratio calibration:
 *  ADC count ~ 0 – 100  => clean air
 *  ADC count > 300      => detectable alcohol
 *  ADC count > 500      => dangerous – engine cut-off
 *
 * These values should be calibrated per-unit in the field.
 */
#define MQ3_WARMUP_MS           30000U  /**< 30-second warm-up period       */
#define MQ3_WARN_THRESHOLD      300U    /**< Warning level (ADC counts)     */
#define MQ3_DANGER_THRESHOLD    500U    /**< Danger: engine cut-off level   */

/* ── Status codes ────────────────────────────────────────────────────────── */
typedef enum {
    MQ3_CLEAN  = 0,   /**< Alcohol below warning threshold */
    MQ3_WARN,         /**< Alcohol above warning, below danger */
    MQ3_DANGER        /**< Alcohol above danger – cut engine  */
} MQ3_Status_t;

/* ── API ─────────────────────────────────────────────────────────────────── */
void          mq3_init(void);
uint16_t      mq3_read_raw(void);
MQ3_Status_t  mq3_get_status(void);
uint8_t       mq3_is_warmed_up(void);

#endif /* MQ3_H */
