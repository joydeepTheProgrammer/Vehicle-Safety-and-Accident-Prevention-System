/**
 * @file gsm_sim800l.c
 * @brief SIM800L GSM Module driver implementation
 * @project Integrated Intelligent Vehicle Safety and Accident Prevention System
 *
 * Communication via software UART (PD3=TX, PD4=RX) at 9600 baud.
 *
 * Emergency SMS format:
 *   "VEHICLE ALERT: [reason]. Location: [lat],[lon]
 *    Maps: https://maps.google.com/?q=[lat],[lon]"
 *
 * AT command flow for SMS:
 *   AT           → OK
 *   AT+CMGF=1    → OK  (text mode)
 *   AT+CMGS="number" → > (prompt)
 *   [message] + Ctrl+Z (0x1A) → +CMGS: <n>, OK
 */

#include "gsm_sim800l.h"
#include "../drivers/uart.h"
#include <util/delay.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

/* ── Internal buffer for responses ──────────────────────────────────────── */
static char s_resp_buf[GSM_RESP_BUF_SIZE];

/* ════════════════════════════════════════════════════════════════════════════
 *  Internal helpers
 * ════════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Read characters from GSM UART into buffer until timeout or buffer full.
 * @param buf      Output buffer.
 * @param max_len  Maximum bytes to read (including null terminator).
 * @param timeout_ms  Timeout in milliseconds.
 */
static void gsm_read_response(char *buf, uint8_t max_len, uint16_t timeout_ms)
{
    uint8_t i = 0;
    uint16_t elapsed = 0;

    while (i < max_len - 1U) {
        /* Approximate 1 ms delay per loop iteration */
        _delay_ms(1);
        elapsed++;
        if (elapsed > timeout_ms) { break; }

        /* Check if a character is incoming (poll RX line) */
        if (!(GSM_RX_PINREG & (1U << GSM_RX_PIN))) {
            buf[i++] = soft_uart_gsm_getchar();
        }
    }
    buf[i] = '\0';
}

/**
 * @brief Send an AT command and wait for "OK" response.
 * @param cmd        AT command string (without \r\n).
 * @param timeout_ms Timeout in milliseconds.
 * @return 1 if "OK" received, 0 on timeout.
 */
static uint8_t gsm_at_command(const char *cmd, uint16_t timeout_ms)
{
    soft_uart_gsm_puts(cmd);
    soft_uart_gsm_puts("\r\n");
    return gsm_wait_for_ok(timeout_ms);
}

/* ════════════════════════════════════════════════════════════════════════════
 *  Public API
 * ════════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Initialise SIM800L GSM module.
 *        Sends AT to verify communication, sets SMS text mode.
 */
void gsm_init(void)
{
    soft_uart_gsm_init();
    _delay_ms(3000);  /* SIM800L needs ~3s after power-on */

    /* Toggle power key if needed (pulse LOW for 1s) */
    GSM_PWR_DDR  |= (1U << GSM_PWR_PIN);
    GSM_PWR_PORT &= ~(1U << GSM_PWR_PIN);
    _delay_ms(1000);
    GSM_PWR_PORT |= (1U << GSM_PWR_PIN);
    _delay_ms(2000);

    /* Handshake */
    for (uint8_t retry = 0; retry < 5U; retry++) {
        if (gsm_at_command("AT", 1000)) { break; }
        _delay_ms(500);
    }

    /* Echo off */
    gsm_at_command("ATE0", 500);

    /* Set SMS text mode */
    gsm_at_command("AT+CMGF=1", 1000);

    /* Set character set to GSM */
    gsm_at_command("AT+CSCS=\"GSM\"", 1000);
}

/**
 * @brief Wait for "OK" response from GSM module.
 * @param timeout_ms Timeout in ms.
 * @return 1 if OK received, 0 otherwise.
 */
uint8_t gsm_wait_for_ok(uint16_t timeout_ms)
{
    gsm_read_response(s_resp_buf, GSM_RESP_BUF_SIZE, timeout_ms);
    return (strstr(s_resp_buf, "OK") != NULL) ? 1U : 0U;
}

/**
 * @brief Send an SMS to a phone number.
 * @param number  Destination phone number (e.g. "+911234567890").
 * @param message Message text (max ~160 chars).
 * @return GSM_OK on success.
 */
GSM_Status_t gsm_send_sms(const char *number, const char *message)
{
    /* AT+CMGS="<number>" */
    soft_uart_gsm_puts("AT+CMGS=\"");
    soft_uart_gsm_puts(number);
    soft_uart_gsm_puts("\"\r\n");

    /* Wait for '>' prompt */
    gsm_read_response(s_resp_buf, GSM_RESP_BUF_SIZE, 5000);
    if (strstr(s_resp_buf, ">") == NULL) {
        return GSM_ERR_NO_RESPONSE;
    }

    /* Send message body */
    soft_uart_gsm_puts(message);

    /* Send Ctrl+Z to terminate SMS */
    soft_uart_gsm_putchar(0x1A);

    /* Wait for +CMGS confirmation */
    gsm_read_response(s_resp_buf, GSM_RESP_BUF_SIZE, 10000);
    if (strstr(s_resp_buf, "+CMGS") == NULL) {
        return GSM_ERR_SEND_FAIL;
    }

    return GSM_OK;
}

/**
 * @brief Send a pre-formatted emergency SMS with GPS coordinates.
 *
 * @param lat    Latitude in decimal degrees.
 * @param lon    Longitude in decimal degrees.
 * @param reason Short string describing the emergency (e.g. "CRASH", "ALCOHOL").
 * @return GSM_OK on success.
 */
GSM_Status_t gsm_send_emergency_sms(float lat, float lon, const char *reason)
{
    char msg[160];
    char lat_int_str[12], lon_int_str[12];
    char lat_dec_str[8],  lon_dec_str[8];

    /* Format latitude: avoid sprintf %f on AVR (no float printf by default) */
    int32_t lat_int = (int32_t)lat;
    int32_t lat_dec = (int32_t)((lat - (float)lat_int) * 10000.0f);
    int32_t lon_int = (int32_t)lon;
    int32_t lon_dec = (int32_t)((lon - (float)lon_int) * 10000.0f);
    if (lat_dec < 0) lat_dec = -lat_dec;
    if (lon_dec < 0) lon_dec = -lon_dec;

    /* Build coordinate strings manually */
    /* lat_int_str */
    {
        int32_t v = (lat_int < 0) ? -lat_int : lat_int;
        int8_t i = 0; char tmp[12];
        if (v == 0) { tmp[i++] = '0'; }
        while (v > 0) { tmp[i++] = (char)('0' + v % 10); v /= 10; }
        if (lat_int < 0) { tmp[i++] = '-'; }
        int8_t j = 0;
        while (i > 0) { lat_int_str[j++] = tmp[--i]; }
        lat_int_str[j] = '\0';
    }
    /* lon_int_str */
    {
        int32_t v = (lon_int < 0) ? -lon_int : lon_int;
        int8_t i = 0; char tmp[12];
        if (v == 0) { tmp[i++] = '0'; }
        while (v > 0) { tmp[i++] = (char)('0' + v % 10); v /= 10; }
        if (lon_int < 0) { tmp[i++] = '-'; }
        int8_t j = 0;
        while (i > 0) { lon_int_str[j++] = tmp[--i]; }
        lon_int_str[j] = '\0';
    }
    /* lat_dec_str (4 decimal digits) */
    {
        int8_t i = 0;
        for (int8_t d = 3; d >= 0; d--) {
            lat_dec_str[d] = (char)('0' + lat_dec % 10);
            lat_dec /= 10;
        }
        lat_dec_str[4] = '\0';
    }
    /* lon_dec_str */
    {
        for (int8_t d = 3; d >= 0; d--) {
            lon_dec_str[d] = (char)('0' + lon_dec % 10);
            lon_dec /= 10;
        }
        lon_dec_str[4] = '\0';
    }

    /* Assemble message – keep under 160 chars */
    uint8_t pos = 0;
    const char *p;

    /* "ALERT:" */
    for (p = "VEHICLE ALERT:"; *p; p++) msg[pos++] = *p;
    for (p = reason;           *p; p++) msg[pos++] = *p;
    for (p = " Loc:";          *p; p++) msg[pos++] = *p;
    for (p = lat_int_str;      *p; p++) msg[pos++] = *p;
    msg[pos++] = '.';
    for (p = lat_dec_str;      *p; p++) msg[pos++] = *p;
    msg[pos++] = ',';
    for (p = lon_int_str;      *p; p++) msg[pos++] = *p;
    msg[pos++] = '.';
    for (p = lon_dec_str;      *p; p++) msg[pos++] = *p;
    for (p = " maps.google.com/?q="; *p; p++) msg[pos++] = *p;
    for (p = lat_int_str;      *p; p++) msg[pos++] = *p;
    msg[pos++] = '.';
    for (p = lat_dec_str;      *p; p++) msg[pos++] = *p;
    msg[pos++] = ',';
    for (p = lon_int_str;      *p; p++) msg[pos++] = *p;
    msg[pos++] = '.';
    for (p = lon_dec_str;      *p; p++) msg[pos++] = *p;
    msg[pos] = '\0';

    return gsm_send_sms(GSM_EMERGENCY_NUMBER, msg);
}
