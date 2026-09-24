/*
 * Dumb-E kinematics: 5-DOF arm
 *   J1 base yaw, J2 shoulder pitch, J3 elbow pitch, J4 forearm roll, J5 wrist pitch.
 *
 * Pure C, no ESP-IDF dependencies: compiled and unit-tested on the host.
 * Units: millimetres and radians everywhere. Conversion to degrees happens only at the user interface.
 *
 * Geometry (from hardware/cad/robot.step, see docs/kinematics.md):
 *  - Base frame: z up, x forward when q1 = 0, q1 positive counter-clockwise seen from above.
 *  - J2 and J3 axes are horizontal and perpendicular to the arm plane; the lateral offsets of the upper arm
 *    cancel at the elbow, so the forearm axis always lies in the vertical plane of the base (azimuth q1).
 *  - q2 is the upper-arm angle from the horizontal (positive = up); q3 is relative to the upper arm
 *    (0 = straight, positive = raises the forearm). Forearm elevation phi = q2 + q3.
 *  - J4 rolls about the forearm axis; J5 axis is perpendicular to the forearm and meets the J4 axis at the
 *    wrist centre W. With q4 = q5 = 0 the tool points along the forearm and the J5 axis is the arm-plane
 *    normal (left side seen from behind the arm).
 *  - TCP = W + tool_offset * (J5 axis) + tool_length * (approach direction).
 *
 * A 5-DOF arm controls the TCP position and the approach direction (2 angles); the rotation of the gripper
 * about its approach axis follows from them.
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
    KIN_ERR_UNREACHABLE,     /* wrist centre outside the workspace of the shoulder/elbow pair */
    KIN_ERR_NO_CONVERGENCE,  /* the wrist-offset iteration did not converge (near a singularity) */
    KIN_ERR_JOINT_LIMIT,     /* a solution exists but violates the joint limits */
} kin_status_t;

typedef enum {
    KIN_ELBOW_UP = 0, /* elbow above the shoulder-wrist line (q3 <= 0) */
    KIN_ELBOW_DOWN,   /* elbow below the shoulder-wrist line (q3 >= 0) */
} kin_elbow_t;

typedef struct {
    float d1;          /* base plane -> shoulder axis height [mm] */
    float a2;          /* shoulder axis -> elbow axis [mm] */
    float a3;          /* elbow axis -> wrist centre (J4/J5 intersection) [mm] */
    float tool_offset; /* TCP offset along the J5 axis [mm] */
    float tool_length; /* TCP offset along the approach direction [mm] */
    float q_min[KIN_NUM_JOINTS]; /* [rad] */
    float q_max[KIN_NUM_JOINTS]; /* [rad] */
} kin_model_t;

/* TCP position and approach direction. */
typedef struct {
    float x, y, z; /* [mm] */
    float pitch;   /* elevation of the approach direction [rad], -pi/2 = pointing down */
    float yaw;     /* azimuth of the approach direction [rad] (irrelevant when pointing straight up/down) */
} kin_pose_t;

typedef struct {
    kin_elbow_t elbow;
    /* Preferred joint values (usually the current position): picks the wrist flip closest to seed[3] and
     * the base angle when the wrist is on the base axis. NULL = zeros. */
    const float *seed;
} kin_ik_options_t;

/* Motor drive of one joint: joint angle <-> motor step position. */
typedef struct {
    float steps_per_rad; /* > 0 */
    bool invert;         /* true if positive motor steps decrease the joint angle */
} kin_drive_t;

bool kin_model_is_valid(const kin_model_t *model);

/* Forward kinematics. q[] in radians. */
kin_status_t kin_forward(const kin_model_t *model, const float q[KIN_NUM_JOINTS], kin_pose_t *out);

/* Forward kinematics with vectors: TCP position, approach direction, J5 axis direction (unit vectors). */
void kin_forward_vec(const kin_model_t *model, const float q[KIN_NUM_JOINTS], float tcp[3], float approach[3],
                     float wrist_axis[3]);

/*
 * Inverse kinematics. On KIN_OK or KIN_ERR_JOINT_LIMIT q_out holds the solution (wrapped to (-pi, pi]);
 * on KIN_ERR_JOINT_LIMIT *bad_joint (if not NULL) holds the offending index.
 */
kin_status_t kin_inverse(const kin_model_t *model, const kin_pose_t *target, const kin_ik_options_t *opts,
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
