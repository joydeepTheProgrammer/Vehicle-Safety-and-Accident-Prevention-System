/**
 * @file ir_sensor.c
 * @brief IR Sensor driver implementation – Lane Departure Warning
 * @project Integrated Intelligent Vehicle Safety and Accident Prevention System
 */

#include "ir_sensor.h"
#include <avr/io.h>
#include <stdint.h>

/**
 * @brief Initialise IR sensor input pins with internal pull-ups disabled.
 *        IR modules have onboard comparators with open-collector outputs;
 *        an external 10kΩ pull-up to 5V on each ECHO line is required.
 */
void ir_sensor_init(void)
{
    /* Left IR: input, no pull-up (external pull-up on PCB) */
    IR_LEFT_DDR  &= ~(1U << IR_LEFT_PIN);
    IR_LEFT_PORT &= ~(1U << IR_LEFT_PIN);

    /* Right IR: input, no pull-up */
    IR_RIGHT_DDR  &= ~(1U << IR_RIGHT_PIN);
    IR_RIGHT_PORT &= ~(1U << IR_RIGHT_PIN);
}

/**
 * @brief Determine current lane status from both IR sensors.
 *
 * A lane marking is "detected" when the digital output matches
 * IR_LANE_DETECTED_LEVEL (0 = LOW active).
 *
 * @return LaneStatus_t indicating which sensors have lost their marking.
 */
LaneStatus_t ir_get_lane_status(void)
{
    /* Read sensor levels */
    uint8_t left_detected  = !(IR_LEFT_PINR  & (1U << IR_LEFT_PIN));
    uint8_t right_detected = !(IR_RIGHT_PINR & (1U << IR_RIGHT_PIN));

    if (!left_detected && !right_detected) {
        return LANE_DEV_BOTH;
    } else if (!left_detected) {
        return LANE_DEV_LEFT;
    } else if (!right_detected) {
        return LANE_DEV_RIGHT;
    }
    return LANE_OK;
}
