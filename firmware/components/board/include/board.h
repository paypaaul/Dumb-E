/*
 * Board pin map: the single place where GPIO numbers are defined.
 *
 * PLACEHOLDER map for an ESP32 DevKit WROOM-32 (see docs/bringup.md). It will be replaced by the pinout of
 * the project PCB once that has been reviewed.
 * Avoided: 0/2/5/12/15 as STEP/DIR/EN (strapping), 6-11 (flash), 1/3 (console); 34-39 are input only.
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

#define BOARD_PIN_DRIVER_EN 13 /* shared TMC2209 EN, active low; needs an external 10k pull-up to 3V3 */
#define BOARD_PIN_SERVO     22 /* gripper servo PWM */
#define BOARD_PIN_LED       2  /* on-board LED */
#define BOARD_PIN_ESTOP     34 /* e-stop input (NC to GND + external pull-up), used if enabled in Kconfig */

/* Configures EN (drivers disabled) and the status LED. Call first. */
esp_err_t board_init(void);

/* Enables or disables all stepper drivers (EN is active low). */
void board_drivers_enable(bool enable);

void board_led_set(bool on);

#ifdef __cplusplus
}
#endif
