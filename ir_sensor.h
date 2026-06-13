/**
 * @file ir_sensor.h
 * @brief IR Sensor driver header – Lane Departure Warning
 * @project Integrated Intelligent Vehicle Safety and Accident Prevention System
 *
 * Two digital IR sensors (TCRT5000 or equivalent) detect road lane markings.
 * Mounted on the underside of the vehicle at the front-left and front-right.
 * Output is LOW when white lane detected, HIGH on dark surface (or vice-versa
 * depending on module polarity – adjust IR_LANE_ACTIVE_LEVEL accordingly).
 */

#ifndef IR_SENSOR_H
#define IR_SENSOR_H

#include <stdint.h>
#include <avr/io.h>

/* ── Pin assignments ─────────────────────────────────────────────────────── */
/* Left IR sensor: detects left lane marking */
#define IR_LEFT_DDR    DDRC
#define IR_LEFT_PORT   PORTC
#define IR_LEFT_PINR   PINC
#define IR_LEFT_PIN    PC4

/* Right IR sensor: detects right lane marking */
#define IR_RIGHT_DDR   DDRC
#define IR_RIGHT_PORT  PORTC
#define IR_RIGHT_PINR  PINC
#define IR_RIGHT_PIN   PC5

/* Active level when lane marking IS detected (module-dependent) */
#define IR_LANE_DETECTED_LEVEL  0U   /**< 0 = LOW means lane detected */

/* ── Lane status ─────────────────────────────────────────────────────────── */
typedef enum {
    LANE_OK       = 0,   /**< Both sensors on road, no deviation */
    LANE_DEV_LEFT,       /**< Left sensor lost lane marking      */
    LANE_DEV_RIGHT,      /**< Right sensor lost lane marking     */
    LANE_DEV_BOTH        /**< Both lost – severe deviation        */
} LaneStatus_t;

/* ── API ─────────────────────────────────────────────────────────────────── */
void          ir_sensor_init(void);
LaneStatus_t  ir_get_lane_status(void);

#endif /* IR_SENSOR_H */
