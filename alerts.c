/**
 * @file alerts.c
 * @brief Alerts and Actuator control implementation
 * @project Integrated Intelligent Vehicle Safety and Accident Prevention System
 *
 * Manages buzzer, engine relay, brake relay, and LED indicators.
 * Buzzer tone patterns:
 *   INFO:     1 × 100ms beep
 *   WARNING:  3 × 100ms beeps
 *   CRITICAL: continuous 50ms on/50ms off (called repeatedly from main loop)
 */

#include "alerts.h"
#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>

/* ── Internal state ──────────────────────────────────────────────────────── */
static uint8_t s_engine_cut = 0;   /* 1 = engine relay active (cut off) */
static uint8_t s_brake_on   = 0;   /* 1 = brake relay active            */

/* ════════════════════════════════════════════════════════════════════════════
 *  Initialisation
 * ════════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Initialise all output pins (buzzer, relays, LEDs).
 *        All outputs start in safe/off state.
 */
void alerts_init(void)
{
    /* Buzzer: output, off */
    BUZZER_DDR  |=  (1U << BUZZER_PIN);
    BUZZER_PORT &= ~(1U << BUZZER_PIN);

    /* Engine relay: output, initially NOT engaged (HIGH = relay off for active-LOW module) */
    ENGINE_RELAY_DDR  |=  (1U << ENGINE_RELAY_PIN);
    ENGINE_RELAY_PORT |=  (1U << ENGINE_RELAY_PIN);  /* HIGH = relay OFF */

    /* Brake relay: output, initially NOT engaged */
    BRAKE_RELAY_DDR  |=  (1U << BRAKE_RELAY_PIN);
    BRAKE_RELAY_PORT |=  (1U << BRAKE_RELAY_PIN);   /* HIGH = relay OFF */

    /* LEDs: output, all off */
    LED_COLLISION_DDR |= (1U << LED_COLLISION_PIN);
    LED_COLLISION_PORT &= ~(1U << LED_COLLISION_PIN);

    LED_ALCOHOL_DDR   |= (1U << LED_ALCOHOL_PIN);
    LED_ALCOHOL_PORT  &= ~(1U << LED_ALCOHOL_PIN);

    LED_CRASH_DDR     |= (1U << LED_CRASH_PIN);
    LED_CRASH_PORT    &= ~(1U << LED_CRASH_PIN);

    LED_LANE_DDR      |= (1U << LED_LANE_PIN);
    LED_LANE_PORT     &= ~(1U << LED_LANE_PIN);

    LED_TEMP_DDR      |= (1U << LED_TEMP_PIN);
    LED_TEMP_PORT     &= ~(1U << LED_TEMP_PIN);

    s_engine_cut = 0;
    s_brake_on   = 0;
}

/* ════════════════════════════════════════════════════════════════════════════
 *  Buzzer
 * ════════════════════════════════════════════════════════════════════════════ */

void buzzer_on(void)  { BUZZER_PORT |=  (1U << BUZZER_PIN); }
void buzzer_off(void) { BUZZER_PORT &= ~(1U << BUZZER_PIN); }

/**
 * @brief Sound buzzer with the given alert pattern.
 *
 * @note This function is blocking for INFO/WARNING levels.
 *       For CRITICAL, it sounds one cycle; call repeatedly from main loop.
 */
void buzzer_beep(AlertLevel_t level)
{
    switch (level) {
        case ALERT_INFO:
            buzzer_on();  _delay_ms(150); buzzer_off(); _delay_ms(100);
            break;

        case ALERT_WARNING:
            for (uint8_t i = 0; i < 3U; i++) {
                buzzer_on();  _delay_ms(100);
                buzzer_off(); _delay_ms(100);
            }
            break;

        case ALERT_CRITICAL:
            buzzer_on();  _delay_ms(50);
            buzzer_off(); _delay_ms(50);
            break;

        case ALERT_NONE:
        default:
            buzzer_off();
            break;
    }
}

/* ════════════════════════════════════════════════════════════════════════════
 *  Relay actuators (active-LOW relay module)
 * ════════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Cut engine power via relay (LOW = relay coil energised = contacts open).
 */
void engine_relay_cut(void)
{
    ENGINE_RELAY_PORT &= ~(1U << ENGINE_RELAY_PIN);  /* Pull LOW = relay ON */
    s_engine_cut = 1;
}

/**
 * @brief Restore engine relay (HIGH = relay off = engine normal).
 */
void engine_relay_restore(void)
{
    ENGINE_RELAY_PORT |=  (1U << ENGINE_RELAY_PIN);  /* Pull HIGH = relay OFF */
    s_engine_cut = 0;
}

/**
 * @brief Engage brake assist relay.
 */
void brake_relay_engage(void)
{
    BRAKE_RELAY_PORT &= ~(1U << BRAKE_RELAY_PIN);
    s_brake_on = 1;
}

/**
 * @brief Release brake assist relay.
 */
void brake_relay_release(void)
{
    BRAKE_RELAY_PORT |=  (1U << BRAKE_RELAY_PIN);
    s_brake_on = 0;
}

/* ════════════════════════════════════════════════════════════════════════════
 *  LED control
 * ════════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Set LEDs based on alert_flags bitmask.
 * @param alert_flags  OR of ALERT_FLAG_* constants.
 */
void led_set(uint8_t alert_flags)
{
    if (alert_flags & ALERT_FLAG_COLLISION) {
        LED_COLLISION_PORT |=  (1U << LED_COLLISION_PIN);
    } else {
        LED_COLLISION_PORT &= ~(1U << LED_COLLISION_PIN);
    }

    if (alert_flags & ALERT_FLAG_ALCOHOL) {
        LED_ALCOHOL_PORT |=  (1U << LED_ALCOHOL_PIN);
    } else {
        LED_ALCOHOL_PORT &= ~(1U << LED_ALCOHOL_PIN);
    }

    if (alert_flags & ALERT_FLAG_CRASH) {
        LED_CRASH_PORT |=  (1U << LED_CRASH_PIN);
    } else {
        LED_CRASH_PORT &= ~(1U << LED_CRASH_PIN);
    }

    if (alert_flags & ALERT_FLAG_LANE) {
        LED_LANE_PORT |=  (1U << LED_LANE_PIN);
    } else {
        LED_LANE_PORT &= ~(1U << LED_LANE_PIN);
    }

    if (alert_flags & ALERT_FLAG_OVERHEAT) {
        LED_TEMP_PORT |=  (1U << LED_TEMP_PIN);
    } else {
        LED_TEMP_PORT &= ~(1U << LED_TEMP_PIN);
    }
}

void led_clear_all(void)
{
    led_set(0);
}

/* ════════════════════════════════════════════════════════════════════════════
 *  Composite alert handler
 * ════════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Central alert update: sets LEDs and sounds appropriate buzzer pattern.
 *
 * @param alert_flags  Bitmask of active alert flags.
 * @param level        Highest priority alert level.
 */
void alerts_update(uint8_t alert_flags, AlertLevel_t level)
{
    led_set(alert_flags);
    buzzer_beep(level);
}
