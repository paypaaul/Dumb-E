/*
 * Mechanical configuration of the arm. Human-friendly units (mm, degrees) — converted at init.
 * Geometry and gear ratio come from hardware/cad/robot.step; limits, speeds, microsteps and the park pose are
 * still PLACEHOLDERS to verify on the real arm (see docs/kinematics.md).
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "kinematics.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    const char *name;
    uint32_t full_steps_per_rev; /* motor: 200 for 1.8 deg */
    uint32_t microsteps;         /* must match the TMC2209 MS1/MS2 jumpers */
    float gear_ratio;            /* cycloidal (and belt) reduction */
    bool invert;                 /* flip if positive steps move the joint the wrong way */
    float min_deg, max_deg;      /* soft limits */
    float v_max_deg_s;           /* max joint speed */
    float a_max_deg_s2;          /* max joint acceleration */
    float park_deg;              /* rest pose used by `zero` and `park` */
} robot_joint_config_t;

typedef struct {
    float d1_mm, a2_mm, a3_mm;            /* see kinematics.h */
    float tool_offset_mm, tool_length_mm; /* TCP relative to the wrist centre */
    robot_joint_config_t joints[KIN_NUM_JOINTS];
    float unreferenced_speed_scale; /* jog speed cap while not referenced (0..1] */
    uint16_t gripper_pulse_min_us, gripper_pulse_max_us;
    float gripper_open_pct, gripper_close_pct;
    float gripper_speed_pct_s;
} robot_config_t;

extern const robot_config_t robot_default_config;

#ifdef __cplusplus
}
#endif
