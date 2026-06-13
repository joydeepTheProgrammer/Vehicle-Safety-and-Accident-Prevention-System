/**
 * @file collision.h
 * @brief Collision avoidance system header
 * @project Integrated Intelligent Vehicle Safety and Accident Prevention System
 */

#ifndef COLLISION_H
#define COLLISION_H

#include <stdint.h>
#include "../sensors/ultrasonic.h"

/* ── Collision state ─────────────────────────────────────────────────────── */
typedef enum {
    COL_STATE_CLEAR    = 0,  /**< All directions clear         */
    COL_STATE_WARNING,       /**< Object in warning zone       */
    COL_STATE_CRITICAL       /**< Object in critical zone      */
} CollisionState_t;

typedef struct {
    uint16_t        dist_cm[US_SENSOR_COUNT]; /**< Distance per direction  */
    CollisionState_t state;                    /**< Worst-case state        */
    uint8_t         direction_flags;           /**< Bitmask of active dirs */
} CollisionData_t;

/* Direction bitmask bits */
#define COL_DIR_FRONT  (1U << US_FRONT)
#define COL_DIR_REAR   (1U << US_REAR)
#define COL_DIR_LEFT   (1U << US_LEFT)
#define COL_DIR_RIGHT  (1U << US_RIGHT)

/* ── API ─────────────────────────────────────────────────────────────────── */
void              collision_init(void);
CollisionState_t  collision_update(CollisionData_t *data);

#endif /* COLLISION_H */
