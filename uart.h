/**
 * @file uart.h
 * @brief UART driver header for ATmega328P
 * @project Integrated Intelligent Vehicle Safety and Accident Prevention System
 *
 * Provides hardware UART0 at configurable baud rate.
 * Two software UARTs are bit-banged for GSM and GPS modules.
 */

#ifndef UART_H
#define UART_H

#include <stdint.h>
#include <avr/io.h>

/* ── Hardware UART (USB/debug) ───────────────────────────────────────────── */
#define UART_BAUD          9600UL
#define UART_UBRR_VAL      (F_CPU / (16UL * UART_BAUD) - 1)

void     uart_init(void);
void     uart_putchar(char c);
void     uart_puts(const char *s);
char     uart_getchar(void);
uint8_t  uart_available(void);

/* ── Software UART – GSM (SIM800L) ──────────────────────────────────────── */
#define GSM_TX_PORT   PORTD
#define GSM_TX_DDR    DDRD
#define GSM_TX_PIN    PD3

#define GSM_RX_PORT   PORTD
#define GSM_RX_DDR    DDRD
#define GSM_RX_PINREG PIND
#define GSM_RX_PIN    PD4

void    soft_uart_gsm_init(void);
void    soft_uart_gsm_putchar(char c);
void    soft_uart_gsm_puts(const char *s);
char    soft_uart_gsm_getchar(void);

/* ── Software UART – GPS (NEO-6M) ───────────────────────────────────────── */
#define GPS_TX_PORT   PORTD
#define GPS_TX_DDR    DDRD
#define GPS_TX_PIN    PD5

#define GPS_RX_PORT   PORTD
#define GPS_RX_DDR    DDRD
#define GPS_RX_PINREG PIND
#define GPS_RX_PIN    PD6

void    soft_uart_gps_init(void);
void    soft_uart_gps_putchar(char c);
char    soft_uart_gps_getchar(void);

#endif /* UART_H */
