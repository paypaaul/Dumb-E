#include "kinematics.h"

#include <math.h>
#include <stddef.h>

#define KIN_PI       3.14159265358979323846f
#define KIN_TWO_PI   (2.0f * KIN_PI)
/* Tolerance on the elbow cosine before declaring a target unreachable (absorbs float rounding). */
#define KIN_COS_EPS  1e-4f
/* Targets closer than this to the base axis are singular [mm]. */
#define KIN_AXIS_EPS 1e-3f

bool kin_model_is_valid(const kin_model_t *model)
{
    if (model == NULL) {
        return false;
    }
    if (!(model->d1 >= 0.0f) || !(model->a1 >= 0.0f) || !(model->a2 > 0.0f) || !(model->a3 > 0.0f) ||
        !(model->d5 >= 0.0f)) {
        return false;
    }
    for (int i = 0; i < KIN_NUM_JOINTS; i++) {
        if (!(model->q_min[i] < model->q_max[i])) {
            return false;
        }
    }
    return true;
}

float kin_wrap_angle(float a)
{
    a = fmodf(a, KIN_TWO_PI);
    if (a <= -KIN_PI) {
        a += KIN_TWO_PI;
    } else if (a > KIN_PI) {
        a -= KIN_TWO_PI;
    }
    return a;
}

kin_status_t kin_forward(const kin_model_t *model, const float q[KIN_NUM_JOINTS], kin_pose_t *out)
{
    if (model == NULL || q == NULL || out == NULL) {
        return KIN_ERR_INVALID_ARG;
    }
    const float p2 = q[1];
    const float p3 = p2 + q[2];
    const float p4 = p3 + q[3];

    const float r = model->a1 + model->a2 * cosf(p2) + model->a3 * cosf(p3) + model->d5 * cosf(p4);
    out->z = model->d1 + model->a2 * sinf(p2) + model->a3 * sinf(p3) + model->d5 * sinf(p4);
    out->x = r * cosf(q[0]);
    out->y = r * sinf(q[0]);
    out->pitch = kin_wrap_angle(p4);
    out->roll = kin_wrap_angle(q[4]);
    return KIN_OK;
}

int kin_check_limits(const kin_model_t *model, const float q[KIN_NUM_JOINTS])
{
    for (int i = 0; i < KIN_NUM_JOINTS; i++) {
        if (q[i] < model->q_min[i] || q[i] > model->q_max[i]) {
            return i;
        }
    }
    return -1;
}

kin_status_t kin_inverse(const kin_model_t *model, const kin_pose_t *target, kin_elbow_t elbow,
                         float q_out[KIN_NUM_JOINTS], int *bad_joint)
{
    if (model == NULL || target == NULL || q_out == NULL) {
        return KIN_ERR_INVALID_ARG;
    }

    const float r = sqrtf(target->x * target->x + target->y * target->y);
    if (r < KIN_AXIS_EPS) {
        return KIN_ERR_SINGULAR;
    }
    const float q1 = atan2f(target->y, target->x);

    /* Wrist pitch axis position in the arm plane, relative to the shoulder axis. */
    const float rw = r - model->a1 - model->d5 * cosf(target->pitch);
    const float zw = target->z - model->d1 - model->d5 * sinf(target->pitch);

    float c3 = (rw * rw + zw * zw - model->a2 * model->a2 - model->a3 * model->a3) /
               (2.0f * model->a2 * model->a3);
    if (c3 > 1.0f + KIN_COS_EPS || c3 < -1.0f - KIN_COS_EPS) {
        return KIN_ERR_UNREACHABLE;
    }
    c3 = fminf(1.0f, fmaxf(-1.0f, c3));
    float s3 = sqrtf(1.0f - c3 * c3);
    if (elbow == KIN_ELBOW_UP) {
        s3 = -s3;
    }

    const float q3 = atan2f(s3, c3);
    const float q2 = atan2f(zw, rw) - atan2f(model->a3 * s3, model->a2 + model->a3 * c3);
    const float q4 = target->pitch - q2 - q3;

    q_out[0] = kin_wrap_angle(q1);
    q_out[1] = kin_wrap_angle(q2);
    q_out[2] = kin_wrap_angle(q3);
    q_out[3] = kin_wrap_angle(q4);
    q_out[4] = kin_wrap_angle(target->roll);

    const int bad = kin_check_limits(model, q_out);
    if (bad_joint != NULL) {
        *bad_joint = bad;
    }
    return bad < 0 ? KIN_OK : KIN_ERR_JOINT_LIMIT;
}

const char *kin_status_str(kin_status_t status)
{
    switch (status) {
    case KIN_OK:
        return "ok";
    case KIN_ERR_INVALID_ARG:
        return "invalid argument";
    case KIN_ERR_UNREACHABLE:
        return "target unreachable";
    case KIN_ERR_SINGULAR:
        return "target on base axis (singular)";
    case KIN_ERR_JOINT_LIMIT:
        return "joint limit exceeded";
    }
    return "unknown";
}

float kin_steps_per_rad(uint32_t full_steps_per_rev, uint32_t microsteps, float gear_ratio)
{
    return (float)full_steps_per_rev * (float)microsteps * gear_ratio / KIN_TWO_PI;
}

int32_t kin_joint_to_steps(const kin_drive_t *drive, float q)
{
    const float steps = q * drive->steps_per_rad;
    return (int32_t)lroundf(drive->invert ? -steps : steps);
}

float kin_steps_to_joint(const kin_drive_t *drive, int32_t steps)
{
    const float q = (float)steps / drive->steps_per_rad;
    return drive->invert ? -q : q;
}
