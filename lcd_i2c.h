/**
 * @file lcd_i2c.h
 * @brief 16×2 LCD driver via PCF8574 I2C expander
 * @project Integrated Intelligent Vehicle Safety and Accident Prevention System
 *
 * Compatible with common I2C LCD backpack modules using PCF8574 or PCF8574A.
 * Default I2C address: 0x27 (PCF8574) or 0x3F (PCF8574A).
 * Operates LCD in 4-bit mode.
 */

#ifndef LCD_I2C_H
#define LCD_I2C_H

#include <stdint.h>

/* ── I2C address ─────────────────────────────────────────────────────────── */
#define LCD_I2C_ADDR        0x27U   /**< Change to 0x3F for PCF8574A */

/* ── LCD dimensions ──────────────────────────────────────────────────────── */
#define LCD_COLS            16U
#define LCD_ROWS            2U

/* ── Commands ────────────────────────────────────────────────────────────── */
#define LCD_CLEAR_DISPLAY   0x01U
#define LCD_RETURN_HOME     0x02U
#define LCD_ENTRY_MODE_SET  0x04U
#define LCD_DISPLAY_CTRL    0x08U
#define LCD_FUNCTION_SET    0x20U
#define LCD_SET_DDRAM_ADDR  0x80U

/* Flags for ENTRY_MODE_SET */
#define LCD_ENTRY_LEFT      0x02U
#define LCD_ENTRY_SHIFT_DEC 0x00U

/* Flags for DISPLAY_CTRL */
#define LCD_DISPLAY_ON      0x04U
#define LCD_CURSOR_OFF      0x00U
#define LCD_BLINK_OFF       0x00U

/* Flags for FUNCTION_SET */
#define LCD_4BIT_MODE       0x00U
#define LCD_2LINE           0x08U
#define LCD_5x8_DOTS        0x00U

/* PCF8574 bit positions for LCD lines */
#define LCD_RS_BIT          0U
#define LCD_RW_BIT          1U
#define LCD_EN_BIT          2U
#define LCD_BL_BIT          3U   /**< Backlight */
#define LCD_D4_BIT          4U
#define LCD_D5_BIT          5U
#define LCD_D6_BIT          6U
#define LCD_D7_BIT          7U

/* ── API ─────────────────────────────────────────────────────────────────── */
void lcd_init(void);
void lcd_clear(void);
void lcd_home(void);
void lcd_set_cursor(uint8_t col, uint8_t row);
void lcd_print_char(char c);
void lcd_print_str(const char *s);
void lcd_print_int(int32_t val);
void lcd_backlight_on(void);
void lcd_backlight_off(void);

#endif /* LCD_I2C_H */
