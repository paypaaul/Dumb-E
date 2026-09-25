/*
 * Servo gripper on LEDC (50 Hz). Position is a percentage mapped linearly onto [pulse_min_us, pulse_max_us].
 * Moves are rate-limited in software so the servo does not slam.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int gpio;
    uint16_t pulse_min_us;   /* pulse at 0 % */
    uint16_t pulse_max_us;   /* pulse at 100 % */
    float speed_pct_per_s;   /* rate limit, <= 0 means immediate */
} gripper_config_t;

esp_err_t gripper_init(const gripper_config_t *config);

/* Moves to `percent` (0..100). The first command after init/off jumps directly (position unknown). */
esp_err_t gripper_set(float percent);

/* Stops the PWM: the servo is no longer driven. */
esp_err_t gripper_off(void);

bool gripper_is_on(void);
float gripper_position(void);
float gripper_target(void);

#ifdef __cplusplus
}
#endif
