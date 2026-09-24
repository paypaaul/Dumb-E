/*
 * Motion task: turns joint-space move commands into step segments for the step generator.
 *
 * Runs on core 1 at the segment rate (1 kHz by default), woken by the step ISR. Moves are synchronised: all
 * joints start and stop together along a straight line in joint space, each within its own velocity and
 * acceleration limits. Commands are queued and executed one after the other.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "kinematics.h"
#include "stepgen.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MOTION_MAX_JOINTS STEPGEN_MAX_AXES

typedef struct {
    uint8_t num_joints;
    kin_drive_t drive[MOTION_MAX_JOINTS];
    float v_max[MOTION_MAX_JOINTS]; /* [rad/s] */
    float a_max[MOTION_MAX_JOINTS]; /* [rad/s^2] */
    stepgen_config_t stepgen;
} motion_config_t;

typedef struct {
    bool busy;                          /* moving or commands pending */
    uint32_t queued;                    /* commands waiting in the queue */
    float q_cmd[MOTION_MAX_JOINTS];     /* last commanded joint position [rad] */
    int32_t steps[MOTION_MAX_JOINTS];   /* step positions emitted by the step generator */
    float q_actual[MOTION_MAX_JOINTS];  /* joint position derived from `steps` [rad] */
    stepgen_stats_t stepgen;
} motion_status_t;

/* Creates the motion task on core 1 and starts the step generator. Velocity limits that exceed the step
 * generator capability are clamped (with a warning). */
esp_err_t motion_init(const motion_config_t *config);

/* Queues a synchronised joint move. speed_scale and accel_scale in (0, 1] scale the per-joint limits. */
esp_err_t motion_move_joints(const float q_target[MOTION_MAX_JOINTS], float speed_scale, float accel_scale);

/* Controlled stop: drops queued commands and decelerates the current move along its path. */
esp_err_t motion_stop(void);

/* Immediate stop without deceleration: drops everything. Steps may be lost at high speed. */
esp_err_t motion_abort(void);

bool motion_is_busy(void);

/* Declares the current joint positions (reference). Only allowed while not busy. */
esp_err_t motion_set_joint_positions(const float q[MOTION_MAX_JOINTS]);

void motion_get_status(motion_status_t *status);

#ifdef __cplusplus
}
#endif
