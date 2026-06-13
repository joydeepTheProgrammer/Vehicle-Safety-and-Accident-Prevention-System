/**
 * @file uart.c
 * @brief UART driver implementation (HW UART0 + 2× Software UARTs)
 * @project Integrated Intelligent Vehicle Safety and Accident Prevention System
 *
 * Hardware UART0 @ 9600 baud: debug output / PC communication.
 * Software UART for GSM (SIM800L): PD3=TX, PD4=RX @ 9600 baud.
 * Software UART for GPS (NEO-6M):  PD5=TX, PD6=RX @ 9600 baud.
 *
 * @note F_CPU must be defined at compile time (e.g. -DF_CPU=16000000UL).
 *       Software UART uses busy-wait bit-banging; disable interrupts during TX.
 */

#include "uart.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>

/* ════════════════════════════════════════════════════════════════════════════
 *  Internal helpers
 * ════════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Bit-period delay in microseconds for 9600 baud soft-UART.
 * One bit = 1/9600 s ≈ 104.167 µs.
 */
#define SOFT_UART_BIT_DELAY_US   104U

/* ════════════════════════════════════════════════════════════════════════════
 *  Hardware UART0
 * ════════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Initialise hardware UART0 at UART_BAUD baud, 8N1.
 */
void uart_init(void)
{
    /* Set baud rate */
    UBRR0H = (uint8_t)(UART_UBRR_VAL >> 8);
    UBRR0L = (uint8_t)(UART_UBRR_VAL);

    /* Enable transmitter and receiver */
    UCSR0B = (1U << RXEN0) | (1U << TXEN0);

    /* 8-bit data frame, 1 stop bit, no parity */
    UCSR0C = (1U << UCSZ01) | (1U << UCSZ00);
}

/**
 * @brief Transmit a single character via UART0.
 */
void uart_putchar(char c)
{
    /* Wait for empty transmit buffer */
    while (!(UCSR0A & (1U << UDRE0))) {}
    UDR0 = (uint8_t)c;
}

/**
 * @brief Transmit a null-terminated string via UART0.
 */
void uart_puts(const char *s)
{
    while (*s != '\0') {
        uart_putchar(*s++);
    }
}

/**
 * @brief Receive a character from UART0 (blocking).
 */
char uart_getchar(void)
{
    while (!(UCSR0A & (1U << RXC0))) {}
    return (char)UDR0;
}

/**
 * @brief Check if a character is available in UART0 receive buffer.
 * @return 1 if data available, 0 otherwise.
 */
uint8_t uart_available(void)
{
    return (UCSR0A & (1U << RXC0)) ? 1U : 0U;
}

/* ════════════════════════════════════════════════════════════════════════════
 *  Software UART – GSM (SIM800L) on PD3 (TX) / PD4 (RX)
 * ════════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Initialise software UART pins for GSM module.
 */
void soft_uart_gsm_init(void)
{
    /* TX: output, idle HIGH (UART idle state) */
    GSM_TX_DDR  |=  (1U << GSM_TX_PIN);
    GSM_TX_PORT |=  (1U << GSM_TX_PIN);

    /* RX: input with pull-up */
    GSM_RX_DDR  &= ~(1U << GSM_RX_PIN);
    GSM_RX_PORT |=  (1U << GSM_RX_PIN);
}

/**
 * @brief Transmit one character via GSM software UART (9600 baud, 8N1).
 * @param c Character to send.
 */
void soft_uart_gsm_putchar(char c)
{
    uint8_t sreg = SREG;
    cli();   /* Disable interrupts for timing accuracy */

    /* Start bit */
    GSM_TX_PORT &= ~(1U << GSM_TX_PIN);
    _delay_us(SOFT_UART_BIT_DELAY_US);

    /* 8 data bits, LSB first */
    for (uint8_t i = 0; i < 8U; i++) {
        if ((uint8_t)c & (1U << i)) {
            GSM_TX_PORT |=  (1U << GSM_TX_PIN);
        } else {
            GSM_TX_PORT &= ~(1U << GSM_TX_PIN);
        }
        _delay_us(SOFT_UART_BIT_DELAY_US);
    }

    /* Stop bit */
    GSM_TX_PORT |= (1U << GSM_TX_PIN);
    _delay_us(SOFT_UART_BIT_DELAY_US);

    SREG = sreg;   /* Restore interrupt state */
}

/**
 * @brief Transmit a null-terminated string via GSM software UART.
 */
void soft_uart_gsm_puts(const char *s)
{
    while (*s != '\0') {
        soft_uart_gsm_putchar(*s++);
    }
}

/**
 * @brief Receive one character from GSM software UART (blocking, 9600 baud).
 * @return Received character.
 */
char soft_uart_gsm_getchar(void)
{
    uint8_t data = 0;

    /* Wait for start bit (LOW) */
    while (GSM_RX_PINREG & (1U << GSM_RX_PIN)) {}

    /* Sample in the middle of the start bit */
    _delay_us(SOFT_UART_BIT_DELAY_US / 2U);
    _delay_us(SOFT_UART_BIT_DELAY_US / 2U);

    /* Sample 8 data bits */
    for (uint8_t i = 0; i < 8U; i++) {
        _delay_us(SOFT_UART_BIT_DELAY_US);
        if (GSM_RX_PINREG & (1U << GSM_RX_PIN)) {
            data |= (1U << i);
        }
    }

    /* Wait for stop bit */
    _delay_us(SOFT_UART_BIT_DELAY_US);

    return (char)data;
}

/* ════════════════════════════════════════════════════════════════════════════
 *  Software UART – GPS (NEO-6M) on PD5 (TX) / PD6 (RX)
 * ════════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Initialise software UART pins for GPS module.
 */
void soft_uart_gps_init(void)
{
    /* TX: output, idle HIGH */
    GPS_TX_DDR  |=  (1U << GPS_TX_PIN);
    GPS_TX_PORT |=  (1U << GPS_TX_PIN);

    /* RX: input with pull-up */
    GPS_RX_DDR  &= ~(1U << GPS_RX_PIN);
    GPS_RX_PORT |=  (1U << GPS_RX_PIN);
}

/**
 * @brief Transmit one character via GPS software UART (9600 baud, 8N1).
 */
void soft_uart_gps_putchar(char c)
{
    uint8_t sreg = SREG;
    cli();

    /* Start bit */
    GPS_TX_PORT &= ~(1U << GPS_TX_PIN);
    _delay_us(SOFT_UART_BIT_DELAY_US);

    /* 8 data bits, LSB first */
    for (uint8_t i = 0; i < 8U; i++) {
        if ((uint8_t)c & (1U << i)) {
            GPS_TX_PORT |=  (1U << GPS_TX_PIN);
        } else {
            GPS_TX_PORT &= ~(1U << GPS_TX_PIN);
        }
        _delay_us(SOFT_UART_BIT_DELAY_US);
    }

    /* Stop bit */
    GPS_TX_PORT |= (1U << GPS_TX_PIN);
    _delay_us(SOFT_UART_BIT_DELAY_US);

    SREG = sreg;
}

/**
 * @brief Receive one character from GPS software UART (blocking, 9600 baud).
 */
char soft_uart_gps_getchar(void)
{
    uint8_t data = 0;

    /* Wait for start bit */
    while (GPS_RX_PINREG & (1U << GPS_RX_PIN)) {}

    _delay_us(SOFT_UART_BIT_DELAY_US / 2U);
    _delay_us(SOFT_UART_BIT_DELAY_US / 2U);

    for (uint8_t i = 0; i < 8U; i++) {
        _delay_us(SOFT_UART_BIT_DELAY_US);
        if (GPS_RX_PINREG & (1U << GPS_RX_PIN)) {
            data |= (1U << i);
        }
    }

    _delay_us(SOFT_UART_BIT_DELAY_US);

    return (char)data;
}
