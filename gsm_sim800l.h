/**
 * @file gsm_sim800l.h
 * @brief SIM800L GSM Module driver header
 * @project Integrated Intelligent Vehicle Safety and Accident Prevention System
 *
 * Communicates via software UART (PD3=TX, PD4=RX) at 9600 baud.
 * Used to send emergency SMS alerts with GPS coordinates when a crash or
 * alcohol danger event is detected.
 */

#ifndef GSM_SIM800L_H
#define GSM_SIM800L_H

#include <stdint.h>

/* ── Emergency contact number ────────────────────────────────────────────── */
/* Set to the emergency contact phone number including country code */
#define GSM_EMERGENCY_NUMBER    "+911234567890"

/* ── Power key pin (optional – can be NC if module auto-powers) ─────────── */
#define GSM_PWR_DDR    DDRD
#define GSM_PWR_PORT   PORTD
#define GSM_PWR_PIN    PD2

/* ── Response buffer size ────────────────────────────────────────────────── */
#define GSM_RESP_BUF_SIZE  64U

/* ── Return codes ────────────────────────────────────────────────────────── */
typedef enum {
    GSM_OK = 0,
    GSM_ERR_NO_RESPONSE,
    GSM_ERR_SEND_FAIL
} GSM_Status_t;

/* ── API ─────────────────────────────────────────────────────────────────── */
void        gsm_init(void);
GSM_Status_t gsm_send_sms(const char *number, const char *message);
GSM_Status_t gsm_send_emergency_sms(float lat, float lon, const char *reason);
uint8_t     gsm_wait_for_ok(uint16_t timeout_ms);

#endif /* GSM_SIM800L_H */
