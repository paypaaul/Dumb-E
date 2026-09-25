/*
 * Robot supervisor: state machine, safety checks and the high-level commands used by the protocol.
 *
 * State (derived): DISABLED (drivers off) -> ENABLED (holding, not referenced) -> READY (referenced).
 * ESTOP overrides everything until `robot_reset()`. Without homing sensors the reference is set manually:
 * place the arm in the park pose and call robot_zero(NULL). The reference is lost on disable and reboot.
 * Angles are in degrees at this interface.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "kinematics.h"
#include "robot_config.h"
#include "stepgen.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    ROBOT_DISABLED = 0,
    ROBOT_ENABLED,
    ROBOT_READY,
    ROBOT_ESTOP,
} robot_state_t;

typedef enum {
    ROBOT_OK = 0,
    ROBOT_ERR_ARG,            /* malformed or out-of-range argument */
    ROBOT_ERR_STATE,          /* not allowed in the current state */
    ROBOT_ERR_NOT_REFERENCED, /* absolute move requested before `zero` */
    ROBOT_ERR_BUSY,           /* the arm is moving */
    ROBOT_ERR_LIMIT,          /* target outside the joint limits */
    ROBOT_ERR_UNREACHABLE,    /* pose outside the workspace */
    ROBOT_ERR_SINGULAR,       /* no IK solution found near a singular configuration */
    ROBOT_ERR_ESTOP,          /* e-stop active */
    ROBOT_ERR_QUEUE_FULL,
    ROBOT_ERR_INTERNAL,
} robot_err_t;

typedef struct {
    robot_state_t state;
    bool enabled;
    bool referenced;
    bool moving;
    bool estop_input;                 /* e-stop input currently active */
    uint32_t queued;
    float q_deg[KIN_NUM_JOINTS];      /* commanded joint angles */
    int32_t steps[KIN_NUM_JOINTS];
    bool pose_valid;
    kin_pose_t pose;                  /* TCP position and approach direction from FK (mm, rad) */
    bool gripper_on;
    float gripper_pct;
    stepgen_stats_t stepgen;
} robot_status_t;

/* Initializes board, motion (core 1), gripper, e-stop and status LED. Drivers start disabled. */
robot_err_t robot_init(void);

robot_err_t robot_enable(void);
robot_err_t robot_disable(void);

/* Declares the current joint angles (NULL = park pose from the configuration). */
robot_err_t robot_zero(const float *q_deg);

robot_err_t robot_move_joints(const float q_deg[KIN_NUM_JOINTS], float speed_pct, float accel_pct);

/* IK then synchronised joint move. On ROBOT_ERR_LIMIT, *bad_joint (if not NULL) is the offending joint. */
robot_err_t robot_move_pose(const kin_pose_t *pose, kin_elbow_t elbow, float speed_pct, float accel_pct,
                            int *bad_joint);

/* Relative move of one joint (0-based index). Allowed without reference at reduced speed. */
robot_err_t robot_jog(int joint, float delta_deg, float speed_pct);

robot_err_t robot_park(float speed_pct);
robot_err_t robot_stop(void);
robot_err_t robot_estop(void);
robot_err_t robot_reset(void);

robot_err_t robot_gripper_set(float percent);
robot_err_t robot_gripper_open(void);
robot_err_t robot_gripper_close(void);
robot_err_t robot_gripper_off(void);

void robot_get_status(robot_status_t *status);
const robot_config_t *robot_get_config(void);
const kin_model_t *robot_get_model(void);

const char *robot_state_str(robot_state_t state);
const char *robot_err_str(robot_err_t err);

#ifdef __cplusplus
}
#endif
