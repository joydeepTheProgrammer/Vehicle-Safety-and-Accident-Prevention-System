/**
 * @file collision.c
 * @brief Collision avoidance system implementation
 * @project Integrated Intelligent Vehicle Safety and Accident Prevention System
 */

#include "collision.h"
#include "../sensors/ultrasonic.h"
#include <stdint.h>

/**
 * @brief Initialise collision module (calls ultrasonic_init internally).
 */
void collision_init(void)
{
    ultrasonic_init();
}

/**
 * @brief Read all 4 ultrasonic sensors and determine collision state.
 *
 * Priority: CRITICAL > WARNING > CLEAR.
 * Only FRONT and REAR sensors trigger brake assist (LEFT/RIGHT are warnings only).
 *
 * @param data  Pointer to CollisionData_t to populate.
 * @return Worst-case CollisionState_t.
 */
CollisionState_t collision_update(CollisionData_t *data)
{
    data->state           = COL_STATE_CLEAR;
    data->direction_flags = 0;

    for (uint8_t s = 0; s < US_SENSOR_COUNT; s++) {
        data->dist_cm[s] = ultrasonic_read_cm((USSensor_t)s);

        if (data->dist_cm[s] == US_TIMEOUT_CM) {
            /* No obstacle detected in this direction */
            continue;
        }

        if (data->dist_cm[s] <= US_CRITICAL_DIST_CM) {
            data->direction_flags |= (1U << s);
            data->state = COL_STATE_CRITICAL;

        } else if (data->dist_cm[s] <= US_WARNING_DIST_CM) {
            data->direction_flags |= (1U << s);
            if (data->state != COL_STATE_CRITICAL) {
                data->state = COL_STATE_WARNING;
            }
        }
    }

    return data->state;
}
