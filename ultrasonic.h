/**
 * @file ultrasonic.h
 * @brief HC-SR04 Ultrasonic Sensor driver header
 * @project Integrated Intelligent Vehicle Safety and Accident Prevention System
 *
 * Supports 4 HC-SR04 sensors: FRONT, REAR, LEFT, RIGHT.
 * Distances returned in centimetres (uint16_t).
 * Maximum reliable range: 400 cm. Returns 0xFFFF on timeout.
 */

#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <stdint.h>
#include <avr/io.h>

/* ── Pin assignments ─────────────────────────────────────────────────────── */
/* All TRIG pins share PORTB, ECHO pins on PORTB/C */

/* FRONT sensor */
#define US_FRONT_TRIG_DDR  DDRB
#define US_FRONT_TRIG_PORT PORTB
#define US_FRONT_TRIG_PIN  PB0
#define US_FRONT_ECHO_DDR  DDRB
#define US_FRONT_ECHO_PORT PORTB
#define US_FRONT_ECHO_PINR PINB
#define US_FRONT_ECHO_PIN  PB1

/* REAR sensor */
#define US_REAR_TRIG_DDR   DDRB
#define US_REAR_TRIG_PORT  PORTB
#define US_REAR_TRIG_PIN   PB2
#define US_REAR_ECHO_DDR   DDRB
#define US_REAR_ECHO_PORT  PORTB
#define US_REAR_ECHO_PINR  PINB
#define US_REAR_ECHO_PIN   PB3

/* LEFT sensor */
#define US_LEFT_TRIG_DDR   DDRC
#define US_LEFT_TRIG_PORT  PORTC
#define US_LEFT_TRIG_PIN   PC0
#define US_LEFT_ECHO_DDR   DDRC
#define US_LEFT_ECHO_PORT  PORTC
#define US_LEFT_ECHO_PINR  PINC
#define US_LEFT_ECHO_PIN   PC1

/* RIGHT sensor */
#define US_RIGHT_TRIG_DDR  DDRC
#define US_RIGHT_TRIG_PORT PORTC
#define US_RIGHT_TRIG_PIN  PC2
#define US_RIGHT_ECHO_DDR  DDRC
#define US_RIGHT_ECHO_PORT PORTC
#define US_RIGHT_ECHO_PINR PINC
#define US_RIGHT_ECHO_PIN  PC3

/* ── Distance thresholds (cm) ────────────────────────────────────────────── */
#define US_CRITICAL_DIST_CM   30U   /**< Brake assist triggered below this  */
#define US_WARNING_DIST_CM    80U   /**< Warning buzzer triggered below this */
#define US_TIMEOUT_CM         0xFFFFU

/* ── Sensor IDs ──────────────────────────────────────────────────────────── */
typedef enum {
    US_FRONT = 0,
    US_REAR,
    US_LEFT,
    US_RIGHT,
    US_SENSOR_COUNT
} USSensor_t;

/* ── API ─────────────────────────────────────────────────────────────────── */
void     ultrasonic_init(void);
uint16_t ultrasonic_read_cm(USSensor_t sensor);

#endif /* ULTRASONIC_H */
