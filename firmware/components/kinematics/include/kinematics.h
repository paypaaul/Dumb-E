/*
 * Dumb-E kinematics: 5-DOF arm (base yaw, shoulder pitch, elbow pitch, wrist pitch, wrist roll).
 *
 * Pure C, no ESP-IDF dependencies: compiled and unit-tested on the host.
 * Units: millimetres and radians everywhere. Conversion to degrees happens only at the user interface.
 *
 * Conventions (see docs/kinematics.md):
 *  - Base frame: z up, x forward when q1 = 0, q1 positive counter-clockwise seen from above.
 *  - q2 is the upper-arm angle measured from the horizontal plane (positive = up).
 *  - q3 and q4 are relative to the previous link (0 = collinear, positive = raises the distal link).
 *  - Tool pitch = q2 + q3 + q4 (approach direction w.r.t. horizontal, -pi/2 = pointing down).
 *  - Tool roll = q5 (rotation about the approach axis).
 *  - All pitch axes are parallel and the arm plane contains the base axis (no lateral offsets).
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define KIN_NUM_JOINTS 5

typedef enum {
    KIN_OK = 0,
    KIN_ERR_INVALID_ARG,
    KIN_ERR_UNREACHABLE, /* target outside the workspace of the shoulder/elbow pair */
    KIN_ERR_SINGULAR,    /* target on the base axis: base angle undefined */
    KIN_ERR_JOINT_LIMIT, /* a solution exists but violates the joint limits */
} kin_status_t;

typedef enum {
    KIN_ELBOW_UP = 0, /* elbow above the shoulder-wrist line (q3 <= 0) */
    KIN_ELBOW_DOWN,   /* elbow below the shoulder-wrist line (q3 >= 0) */
} kin_elbow_t;

/* Geometry and joint limits of the arm. */
typedef struct {
    float d1; /* base plane -> shoulder axis height [mm] */
    float a1; /* horizontal offset base axis -> shoulder axis [mm] */
    float a2; /* shoulder axis -> elbow axis [mm] */
    float a3; /* elbow axis -> wrist pitch axis [mm] */
    float d5; /* wrist pitch axis -> tool centre point along the approach axis [mm] */
    float q_min[KIN_NUM_JOINTS]; /* [rad] */
    float q_max[KIN_NUM_JOINTS]; /* [rad] */
} kin_model_t;

/* Tool pose reachable by a 5-DOF arm: the approach axis always lies in the arm plane. */
typedef struct {
    float x, y, z; /* tool centre point [mm] */
    float pitch;   /* approach direction w.r.t. horizontal [rad] */
    float roll;    /* rotation about the approach axis [rad] */
} kin_pose_t;

/* Motor drive of one joint: joint angle <-> motor step position. */
typedef struct {
    float steps_per_rad; /* > 0 */
    bool invert;         /* true if positive motor steps decrease the joint angle */
} kin_drive_t;

/* Validates the model (positive lengths, ordered limits). */
bool kin_model_is_valid(const kin_model_t *model);

/* Forward kinematics. q[] is in radians. */
kin_status_t kin_forward(const kin_model_t *model, const float q[KIN_NUM_JOINTS], kin_pose_t *out);

/*
 * Closed-form inverse kinematics. On KIN_OK or KIN_ERR_JOINT_LIMIT q_out holds the solution
 * (wrapped to (-pi, pi]); on KIN_ERR_JOINT_LIMIT *bad_joint (if not NULL) holds the offending index.
 */
kin_status_t kin_inverse(const kin_model_t *model, const kin_pose_t *target, kin_elbow_t elbow,
                         float q_out[KIN_NUM_JOINTS], int *bad_joint);

/* Returns -1 if all joints are within limits, otherwise the index of the first violating joint. */
int kin_check_limits(const kin_model_t *model, const float q[KIN_NUM_JOINTS]);

/* Wraps an angle to (-pi, pi]. */
float kin_wrap_angle(float a);

const char *kin_status_str(kin_status_t status);

/* steps/rad for a stepper with the given full steps per revolution, microstepping and gear ratio. */
float kin_steps_per_rad(uint32_t full_steps_per_rev, uint32_t microsteps, float gear_ratio);

/* Joint angle [rad] -> absolute motor step position (rounded to nearest). */
int32_t kin_joint_to_steps(const kin_drive_t *drive, float q);

/* Absolute motor step position -> joint angle [rad]. */
float kin_steps_to_joint(const kin_drive_t *drive, int32_t steps);

#ifdef __cplusplus
}
#endif
