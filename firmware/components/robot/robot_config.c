#include "robot_config.h"

/*
 * Geometry measured on the author's CAD model (not in the repo) (TCP = finger tips) and 20:1 cycloidal drives.
 * Microsteps, limits, speeds and the park pose are placeholders to verify on the arm.
 */
const robot_config_t robot_default_config = {
    .d1_mm = 104.8f,
    .a2_mm = 159.2f,
    .a3_mm = 151.8f,
    .tool_offset_mm = 67.4f,
    .tool_length_mm = 155.4f,
    .joints =
        {
            {.name = "base", .full_steps_per_rev = 200, .microsteps = 16, .gear_ratio = 20.0f,
             .min_deg = -150.0f, .max_deg = 150.0f, .v_max_deg_s = 45.0f, .a_max_deg_s2 = 90.0f, .park_deg = 0.0f},
            {.name = "shoulder", .full_steps_per_rev = 200, .microsteps = 16, .gear_ratio = 20.0f,
             .min_deg = 0.0f, .max_deg = 180.0f, .v_max_deg_s = 45.0f, .a_max_deg_s2 = 90.0f, .park_deg = 90.0f},
            {.name = "elbow", .full_steps_per_rev = 200, .microsteps = 16, .gear_ratio = 20.0f,
             .min_deg = -150.0f, .max_deg = 150.0f, .v_max_deg_s = 45.0f, .a_max_deg_s2 = 90.0f, .park_deg = -90.0f},
            {.name = "forearm_roll", .full_steps_per_rev = 200, .microsteps = 16, .gear_ratio = 20.0f,
             .min_deg = -180.0f, .max_deg = 180.0f, .v_max_deg_s = 60.0f, .a_max_deg_s2 = 120.0f, .park_deg = 0.0f},
            {.name = "wrist_pitch", .full_steps_per_rev = 200, .microsteps = 16, .gear_ratio = 20.0f,
             .min_deg = -120.0f, .max_deg = 120.0f, .v_max_deg_s = 60.0f, .a_max_deg_s2 = 120.0f, .park_deg = 0.0f},
        },
    .unreferenced_speed_scale = 0.25f,
    .gripper_pulse_min_us = 1000,
    .gripper_pulse_max_us = 2000,
    .gripper_open_pct = 0.0f,
    .gripper_close_pct = 100.0f,
    .gripper_speed_pct_s = 100.0f,
};
