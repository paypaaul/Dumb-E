#include "robot_config.h"

/*
 * PLACEHOLDER values. 200 steps x 16 microsteps x 40:1 reproduces the original 32000 steps per 90 degrees.
 * Link lengths, limits and park pose must be measured on the arm.
 */
const robot_config_t robot_default_config = {
    .d1_mm = 100.0f,
    .a1_mm = 0.0f,
    .a2_mm = 150.0f,
    .a3_mm = 150.0f,
    .d5_mm = 60.0f,
    .joints =
        {
            {.name = "base", .full_steps_per_rev = 200, .microsteps = 16, .gear_ratio = 40.0f,
             .min_deg = -150.0f, .max_deg = 150.0f, .v_max_deg_s = 45.0f, .a_max_deg_s2 = 90.0f, .park_deg = 0.0f},
            {.name = "shoulder", .full_steps_per_rev = 200, .microsteps = 16, .gear_ratio = 40.0f,
             .min_deg = 0.0f, .max_deg = 180.0f, .v_max_deg_s = 45.0f, .a_max_deg_s2 = 90.0f, .park_deg = 90.0f},
            {.name = "elbow", .full_steps_per_rev = 200, .microsteps = 16, .gear_ratio = 40.0f,
             .min_deg = -150.0f, .max_deg = 0.0f, .v_max_deg_s = 45.0f, .a_max_deg_s2 = 90.0f, .park_deg = -90.0f},
            {.name = "wrist_pitch", .full_steps_per_rev = 200, .microsteps = 16, .gear_ratio = 40.0f,
             .min_deg = -120.0f, .max_deg = 120.0f, .v_max_deg_s = 60.0f, .a_max_deg_s2 = 120.0f, .park_deg = 0.0f},
            {.name = "wrist_roll", .full_steps_per_rev = 200, .microsteps = 16, .gear_ratio = 40.0f,
             .min_deg = -180.0f, .max_deg = 180.0f, .v_max_deg_s = 60.0f, .a_max_deg_s2 = 120.0f, .park_deg = 0.0f},
        },
    .unreferenced_speed_scale = 0.25f,
    .gripper_pulse_min_us = 1000,
    .gripper_pulse_max_us = 2000,
    .gripper_open_pct = 0.0f,
    .gripper_close_pct = 100.0f,
    .gripper_speed_pct_s = 100.0f,
};
