/*
 * Board pin map: the single place where GPIO numbers are defined.
 *
 * Carrier PCB "test-dumbev2" (hardware/pcb/) with a DOIT ESP32 DevKit V1 (30 pin, WROOM-32).
 * See hardware/pcb/REVIEW.md for the review and the required rework.
 * Driver U6 cannot be used: its STEP/DIR are routed to GPIO35/34, which are input-only on the ESP32.
 */
#pragma once

#include <stdbool.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BOARD_NUM_AXES 5

typedef struct {
    int step;
    int dir;
} board_axis_pins_t;

/* J1 base, J2 shoulder, J3 elbow, J4 wrist pitch, J5 wrist roll. */
extern const board_axis_pins_t board_axis_pins[BOARD_NUM_AXES];

#define BOARD_PIN_DRIVER_EN 25 /* shared TMC2209 EN (active low) + Q3 "drivers enabled" LEDs; add 10k pull-up */
#define BOARD_PIN_SERVO     23 /* gripper servo, header H7 */
#define BOARD_PIN_SERVO2    22 /* spare servo, header H8 */
#define BOARD_PIN_LED       2  /* DevKit on-board LED (also on header H11) */

/* Configures EN (drivers disabled) and the status LED. Call first. */
esp_err_t board_init(void);

/* Enables or disables all stepper drivers (EN is active low). */
void board_drivers_enable(bool enable);

void board_led_set(bool on);

#ifdef __cplusplus
}
#endif
