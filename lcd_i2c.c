/**
 * @file lcd_i2c.c
 * @brief 16×2 LCD driver via PCF8574 I2C expander
 * @project Integrated Intelligent Vehicle Safety and Accident Prevention System
 *
 * Uses the ATmega328P hardware TWI (same bus as MPU-6050).
 * LCD at I2C address 0x27; MPU-6050 at 0x68 – no conflict.
 *
 * Bit mapping on PCF8574:
 *   P0=RS, P1=RW, P2=EN, P3=BL, P4=D4, P5=D5, P6=D6, P7=D7
 */

#include "lcd_i2c.h"
#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>

/* ── Internal state ──────────────────────────────────────────────────────── */
static uint8_t s_backlight = (1U << LCD_BL_BIT);  /* Backlight ON by default */

/* ════════════════════════════════════════════════════════════════════════════
 *  Internal TWI helpers (duplicated subset – lcd_i2c is self-contained)
 * ════════════════════════════════════════════════════════════════════════════ */

static void lcd_twi_start(uint8_t addr)
{
    TWCR = (1U << TWINT) | (1U << TWSTA) | (1U << TWEN);
    while (!(TWCR & (1U << TWINT))) {}
    TWDR = addr;
    TWCR = (1U << TWINT) | (1U << TWEN);
    while (!(TWCR & (1U << TWINT))) {}
}

static void lcd_twi_write(uint8_t data)
{
    TWDR = data;
    TWCR = (1U << TWINT) | (1U << TWEN);
    while (!(TWCR & (1U << TWINT))) {}
}

static void lcd_twi_stop(void)
{
    TWCR = (1U << TWINT) | (1U << TWSTO) | (1U << TWEN);
    while (TWCR & (1U << TWSTO)) {}
}

/* ── Send one byte to PCF8574 ────────────────────────────────────────────── */
static void pcf8574_write(uint8_t data)
{
    lcd_twi_start((LCD_I2C_ADDR << 1U) | 0U);
    lcd_twi_write(data | s_backlight);
    lcd_twi_stop();
}

/* ── Pulse EN bit HIGH→LOW to latch nibble ───────────────────────────────── */
static void lcd_pulse_enable(uint8_t data)
{
    pcf8574_write(data | (1U << LCD_EN_BIT));
    _delay_us(1);
    pcf8574_write(data & ~(1U << LCD_EN_BIT));
    _delay_us(50);
}

/* ── Send a nibble (upper 4 bits of data_byte, already positioned at D4–D7) */
static void lcd_send_nibble(uint8_t nibble)
{
    pcf8574_write(nibble);
    lcd_pulse_enable(nibble);
}

/* ── Send a full byte as two nibbles (4-bit mode) ────────────────────────── */
static void lcd_send_byte(uint8_t rs, uint8_t data)
{
    uint8_t high_nibble = (data & 0xF0U) | (rs ? (1U << LCD_RS_BIT) : 0U);
    uint8_t low_nibble  = ((data << 4U) & 0xF0U) | (rs ? (1U << LCD_RS_BIT) : 0U);
    lcd_send_nibble(high_nibble);
    lcd_send_nibble(low_nibble);
}

static void lcd_command(uint8_t cmd)  { lcd_send_byte(0, cmd); _delay_ms(2); }
static void lcd_data(uint8_t ch)      { lcd_send_byte(1, ch);  _delay_us(50); }

/* ════════════════════════════════════════════════════════════════════════════
 *  Public API
 * ════════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Initialise LCD in 4-bit mode via PCF8574 I2C expander.
 *        Assumes TWI already initialised by mpu6050_init().
 */
void lcd_init(void)
{
    _delay_ms(50);  /* Wait for LCD power-on */

    /* Initialise to 4-bit mode per HD44780 spec (3× function-set with 8-bit) */
    pcf8574_write(0x30U);
    lcd_pulse_enable(0x30U);
    _delay_ms(5);
    lcd_pulse_enable(0x30U);
    _delay_us(150);
    lcd_pulse_enable(0x30U);

    /* Switch to 4-bit mode */
    pcf8574_write(0x20U);
    lcd_pulse_enable(0x20U);

    /* Function Set: 4-bit, 2 lines, 5×8 dots */
    lcd_command(LCD_FUNCTION_SET | LCD_4BIT_MODE | LCD_2LINE | LCD_5x8_DOTS);

    /* Display Control: display ON, cursor OFF, blink OFF */
    lcd_command(LCD_DISPLAY_CTRL | LCD_DISPLAY_ON | LCD_CURSOR_OFF | LCD_BLINK_OFF);

    /* Entry Mode: left-to-right, no display shift */
    lcd_command(LCD_ENTRY_MODE_SET | LCD_ENTRY_LEFT | LCD_ENTRY_SHIFT_DEC);

    lcd_clear();
    lcd_home();
}

void lcd_clear(void)  { lcd_command(LCD_CLEAR_DISPLAY); _delay_ms(2); }
void lcd_home(void)   { lcd_command(LCD_RETURN_HOME);   _delay_ms(2); }

/**
 * @brief Move cursor to specified column (0-based) and row (0-based).
 */
void lcd_set_cursor(uint8_t col, uint8_t row)
{
    static const uint8_t row_offsets[] = {0x00U, 0x40U};
    if (row >= LCD_ROWS) { row = 0; }
    lcd_command(LCD_SET_DDRAM_ADDR | (col + row_offsets[row]));
}

void lcd_print_char(char c)         { lcd_data((uint8_t)c); }

void lcd_print_str(const char *s)
{
    while (*s != '\0') {
        lcd_data((uint8_t)*s++);
    }
}

/**
 * @brief Print a signed 32-bit integer to the LCD.
 */
void lcd_print_int(int32_t val)
{
    char buf[12];
    int8_t i = 0;
    uint8_t negative = 0;

    if (val < 0) { negative = 1; val = -val; }
    if (val == 0) { lcd_data('0'); return; }

    while (val > 0) {
        buf[i++] = (char)('0' + (val % 10));
        val /= 10;
    }
    if (negative) { buf[i++] = '-'; }

    /* Reverse and print */
    while (i > 0) {
        lcd_data((uint8_t)buf[--i]);
    }
}

void lcd_backlight_on(void)
{
    s_backlight = (1U << LCD_BL_BIT);
    pcf8574_write(0);  /* Refresh backlight bit */
}

void lcd_backlight_off(void)
{
    s_backlight = 0;
    pcf8574_write(0);
}
