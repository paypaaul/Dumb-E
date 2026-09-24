#include "kinematics.h"

#include <math.h>
#include <stddef.h>

#define KIN_PI      3.14159265358979323846f
#define KIN_TWO_PI  (2.0f * KIN_PI)
/* Tolerance on the elbow cosine before declaring a target unreachable (absorbs float rounding). */
#define KIN_COS_EPS 1e-4f
/* Wrist centre closer than this to the base axis: base angle taken from the seed [mm]. */
#define KIN_AXIS_EPS 1e-3f
/* Approach (anti)parallel to the forearm: roll taken from the seed. */
#define KIN_WRIST_EPS 1e-6f
/* Wrist-offset iteration. */
#define KIN_MAX_ITER 60 /* first half Newton, second half relaxed fixed point */
#define KIN_ITER_TOL 1e-4f /* [mm] change of the wrist centre */
/* Final check of the solution with the forward kinematics. */
#define KIN_POS_TOL  0.05f /* [mm] */
#define KIN_DIR_TOL  1e-3f /* [rad] */

typedef struct {
    float v[3];
} vec3;

static vec3 v3(float x, float y, float z)
{
    return (vec3){{x, y, z}};
}

static float dot3(vec3 a, vec3 b)
{
    return a.v[0] * b.v[0] + a.v[1] * b.v[1] + a.v[2] * b.v[2];
}

static vec3 axpy(float s, vec3 a, vec3 b) /* s*a + b */
{
    return v3(s * a.v[0] + b.v[0], s * a.v[1] + b.v[1], s * a.v[2] + b.v[2]);
}

/* Frame of the arm plane for base angle q1 and forearm elevation phi. */
typedef struct {
    vec3 x1; /* horizontal, in the arm plane */
    vec3 y1; /* arm-plane normal = J2/J3 axis direction */
    vec3 f;  /* forearm direction */
    vec3 u;  /* in-plane normal to the forearm (f rotated +90 deg) */
} arm_frame_t;

static arm_frame_t arm_frame(float q1, float phi)
{
    const float c1 = cosf(q1), s1 = sinf(q1), cp = cosf(phi), sp = sinf(phi);
    arm_frame_t fr;
    fr.x1 = v3(c1, s1, 0.0f);
    fr.y1 = v3(-s1, c1, 0.0f);
    fr.f = v3(cp * c1, cp * s1, sp);
    fr.u = v3(-sp * c1, -sp * s1, cp);
    return fr;
}

/* J5 axis after the forearm roll: right-handed rotation of y1 about f. */
static vec3 wrist_axis(const arm_frame_t *fr, float q4)
{
    return axpy(sinf(q4), fr->u, axpy(cosf(q4), fr->y1, v3(0, 0, 0)));
}

/* Approach direction: f pitched by q5 about the J5 axis (positive q5 raises the tool when q4 = 0). */
static vec3 approach_dir(const arm_frame_t *fr, float q4, float q5)
{
    const float c4 = cosf(q4), s4 = sinf(q4), c5 = cosf(q5), s5 = sinf(q5);
    /* a = f cos q5 + (c4 u - s4 y1) sin q5 */
    return axpy(c5, fr->f, axpy(s5 * c4, fr->u, axpy(-s5 * s4, fr->y1, v3(0, 0, 0))));
}

static vec3 pose_dir(const kin_pose_t *p)
{
    const float cp = cosf(p->pitch);
    return v3(cp * cosf(p->yaw), cp * sinf(p->yaw), sinf(p->pitch));
}

bool kin_model_is_valid(const kin_model_t *model)
{
    if (model == NULL) {
        return false;
    }
    if (!(model->d1 >= 0.0f) || !(model->a2 > 0.0f) || !(model->a3 > 0.0f) || !(model->tool_length >= 0.0f) ||
        !isfinite(model->tool_offset)) {
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

void kin_forward_vec(const kin_model_t *model, const float q[KIN_NUM_JOINTS], float tcp[3], float approach[3],
                     float wrist_axis_out[3])
{
    const float phi = q[1] + q[2];
    const arm_frame_t fr = arm_frame(q[0], phi);
    const float r = model->a2 * cosf(q[1]) + model->a3 * cosf(phi);
    const float z = model->d1 + model->a2 * sinf(q[1]) + model->a3 * sinf(phi);
    const vec3 w = axpy(r, fr.x1, v3(0.0f, 0.0f, z));
    const vec3 n = wrist_axis(&fr, q[3]);
    const vec3 a = approach_dir(&fr, q[3], q[4]);
    const vec3 p = axpy(model->tool_length, a, axpy(model->tool_offset, n, w));
    for (int i = 0; i < 3; i++) {
        if (tcp) {
            tcp[i] = p.v[i];
        }
        if (approach) {
            approach[i] = a.v[i];
        }
        if (wrist_axis_out) {
            wrist_axis_out[i] = n.v[i];
        }
    }
}

kin_status_t kin_forward(const kin_model_t *model, const float q[KIN_NUM_JOINTS], kin_pose_t *out)
{
    if (model == NULL || q == NULL || out == NULL) {
        return KIN_ERR_INVALID_ARG;
    }
    float p[3], a[3];
    kin_forward_vec(model, q, p, a, NULL);
    out->x = p[0];
    out->y = p[1];
    out->z = p[2];
    out->pitch = asinf(fmaxf(-1.0f, fminf(1.0f, a[2])));
    out->yaw = (fabsf(a[0]) + fabsf(a[1]) > 1e-6f) ? atan2f(a[1], a[0]) : 0.0f;
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

/* Distance between two angles, in (-pi, pi]. */
static float angle_diff(float a, float b)
{
    return kin_wrap_angle(a - b);
}

typedef struct {
    const kin_model_t *model;
    vec3 p;             /* target TCP */
    vec3 a;             /* target approach direction */
    kin_elbow_t elbow;
    float seed0;        /* base angle used when the wrist centre is on the base axis */
} ik_problem_t;

/*
 * Closed-form joints for a given wrist centre w (q1..q3 from w, q4/q5 from the approach direction, wrist flip
 * closest to q4_ref). Returns the wrist centre implied by those joints; *clamped is set if w was out of reach.
 */
static vec3 wrist_map(const ik_problem_t *pb, vec3 w, float q4_ref, float q[KIN_NUM_JOINTS], bool *clamped)
{
    const kin_model_t *m = pb->model;
    const float r = sqrtf(w.v[0] * w.v[0] + w.v[1] * w.v[1]);
    q[0] = (r < KIN_AXIS_EPS) ? pb->seed0 : atan2f(w.v[1], w.v[0]);
    const float zw = w.v[2] - m->d1;
    float c3 = (r * r + zw * zw - m->a2 * m->a2 - m->a3 * m->a3) / (2.0f * m->a2 * m->a3);
    *clamped = c3 > 1.0f + KIN_COS_EPS || c3 < -1.0f - KIN_COS_EPS;
    c3 = fminf(1.0f, fmaxf(-1.0f, c3));
    float s3 = sqrtf(1.0f - c3 * c3);
    if (pb->elbow == KIN_ELBOW_UP) {
        s3 = -s3;
    }
    q[2] = atan2f(s3, c3);
    q[1] = atan2f(zw, r) - atan2f(m->a3 * s3, m->a2 + m->a3 * c3);

    const arm_frame_t fr = arm_frame(q[0], q[1] + q[2]);
    const vec3 a = pb->a;
    const float ay = dot3(a, fr.y1), au = dot3(a, fr.u), af = dot3(a, fr.f);
    const float sin5 = sqrtf(au * au + ay * ay);
    if (sin5 < KIN_WRIST_EPS) {
        /* Approach along the forearm: the roll is free, keep the preferred one. */
        q[3] = q4_ref;
        q[4] = af >= 0.0f ? 0.0f : KIN_PI;
    } else {
        /* a = f cos q5 + (c4 u - s4 y1) sin q5; two solutions (q4, q5) and (q4 + pi, -q5). */
        const float q4a = atan2f(-ay, au), q5a = atan2f(sin5, af);
        const float q4b = kin_wrap_angle(q4a + KIN_PI);
        const bool pick_a = fabsf(angle_diff(q4a, q4_ref)) <= fabsf(angle_diff(q4b, q4_ref));
        q[3] = pick_a ? q4a : q4b;
        q[4] = pick_a ? q5a : -q5a;
    }
    const vec3 n = wrist_axis(&fr, q[3]);
    return axpy(-m->tool_offset, n, axpy(-m->tool_length, a, pb->p));
}

static float norm3(vec3 v)
{
    return sqrtf(dot3(v, v));
}

static vec3 sub3(vec3 a, vec3 b)
{
    return v3(a.v[0] - b.v[0], a.v[1] - b.v[1], a.v[2] - b.v[2]);
}

/* Solves A x = b for n <= 5 by Gaussian elimination with partial pivoting. A is destroyed. */
static bool solve_linear(int n, float A[5][5], float b[5], float x[5])
{
    for (int c = 0; c < n; c++) {
        int piv = c;
        for (int r = c + 1; r < n; r++) {
            if (fabsf(A[r][c]) > fabsf(A[piv][c])) {
                piv = r;
            }
        }
        if (fabsf(A[piv][c]) < 1e-12f) {
            return false;
        }
        if (piv != c) {
            for (int k = 0; k < n; k++) {
                const float t = A[c][k];
                A[c][k] = A[piv][k];
                A[piv][k] = t;
            }
            const float t = b[c];
            b[c] = b[piv];
            b[piv] = t;
        }
        for (int r = c + 1; r < n; r++) {
            const float f = A[r][c] / A[c][c];
            for (int k = c; k < n; k++) {
                A[r][k] -= f * A[c][k];
            }
            b[r] -= f * b[c];
        }
    }
    for (int r = n - 1; r >= 0; r--) {
        float s = b[r];
        for (int k = r + 1; k < n; k++) {
            s -= A[r][k] * x[k];
        }
        x[r] = s / A[r][r];
    }
    return true;
}

/* Newton iteration on the wrist centre (w - wrist_map(w) = 0), with a relaxed fixed-point fallback. */
static bool solve_wrist_centre(const ik_problem_t *pb, vec3 w, float q4_ref, float q[KIN_NUM_JOINTS],
                               bool *clamped)
{
    for (int iter = 0; iter < KIN_MAX_ITER; iter++) {
        const vec3 wn = wrist_map(pb, w, q4_ref, q, clamped);
        const vec3 res = sub3(wn, w);
        if (norm3(res) < KIN_ITER_TOL && !*clamped) {
            return true;
        }
        q4_ref = q[3];
        /* Jacobian of res(w) by finite differences. */
        const float h = 1e-2f;
        float J[5][5] = {{0}}, rhs[5] = {0}, dw[5] = {0};
        for (int k = 0; k < 3; k++) {
            vec3 wk = w;
            wk.v[k] += h;
            float qk[KIN_NUM_JOINTS];
            bool ck;
            const vec3 rk = sub3(wrist_map(pb, wk, q4_ref, qk, &ck), wk);
            for (int i = 0; i < 3; i++) {
                J[i][k] = (rk.v[i] - res.v[i]) / h;
            }
        }
        for (int i = 0; i < 3; i++) {
            rhs[i] = -res.v[i];
        }
        vec3 step = res; /* fixed-point step as fallback */
        if (solve_linear(3, J, rhs, dw)) {
            const vec3 nstep = v3(dw[0], dw[1], dw[2]);
            if (norm3(nstep) < 50.0f) {
                step = nstep;
            }
        }
        /* After the Newton budget, fall back to a relaxed fixed point. */
        if (iter >= KIN_MAX_ITER / 2) {
            step = axpy(0.5f, res, v3(0, 0, 0));
        }
        w = axpy(1.0f, step, w);
    }
    return false;
}

/* Residual of a joint vector: TCP error [mm] and approach error (scaled to mm-like units). */
static void ik_residual(const ik_problem_t *pb, const float q[KIN_NUM_JOINTS], float r[6])
{
    float p[3], a[3];
    kin_forward_vec(pb->model, q, p, a, NULL);
    for (int i = 0; i < 3; i++) {
        r[i] = p[i] - pb->p.v[i];
        r[3 + i] = 100.0f * (a[i] - pb->a.v[i]);
    }
}

static float sq6(const float r[6])
{
    float s = 0.0f;
    for (int i = 0; i < 6; i++) {
        s += r[i] * r[i];
    }
    return s;
}

/* Levenberg-Marquardt refinement in joint space (last resort near ill-conditioned configurations). */
static bool refine_joints(const ik_problem_t *pb, float q[KIN_NUM_JOINTS])
{
    float r[6];
    ik_residual(pb, q, r);
    float cost = sq6(r);
    float lambda = 1e-2f;
    for (int iter = 0; iter < 50; iter++) {
        if (cost < 1e-6f) {
            return true;
        }
        float J[6][KIN_NUM_JOINTS];
        const float h = 1e-4f;
        for (int k = 0; k < KIN_NUM_JOINTS; k++) {
            float qk[KIN_NUM_JOINTS], rk[6];
            for (int i = 0; i < KIN_NUM_JOINTS; i++) {
                qk[i] = q[i];
            }
            qk[k] += h;
            ik_residual(pb, qk, rk);
            for (int i = 0; i < 6; i++) {
                J[i][k] = (rk[i] - r[i]) / h;
            }
        }
        bool improved = false;
        while (!improved) {
            float H[5][5], g[5], dq[5];
            for (int i = 0; i < KIN_NUM_JOINTS; i++) {
                g[i] = 0.0f;
                for (int k = 0; k < 6; k++) {
                    g[i] -= J[k][i] * r[k];
                }
                for (int j = 0; j < KIN_NUM_JOINTS; j++) {
                    float s = 0.0f;
                    for (int k = 0; k < 6; k++) {
                        s += J[k][i] * J[k][j];
                    }
                    H[i][j] = s + (i == j ? lambda : 0.0f);
                }
            }
            if (solve_linear(KIN_NUM_JOINTS, H, g, dq)) {
                float qn[KIN_NUM_JOINTS], rn[6];
                for (int i = 0; i < KIN_NUM_JOINTS; i++) {
                    qn[i] = q[i] + dq[i];
                }
                ik_residual(pb, qn, rn);
                const float cn = sq6(rn);
                if (cn < cost) {
                    for (int i = 0; i < KIN_NUM_JOINTS; i++) {
                        q[i] = qn[i];
                    }
                    for (int i = 0; i < 6; i++) {
                        r[i] = rn[i];
                    }
                    cost = cn;
                    lambda = fmaxf(lambda / 3.0f, 1e-7f);
                    improved = true;
                    break;
                }
            }
            lambda *= 4.0f;
            if (lambda > 1e8f) {
                return cost < 1e-6f;
            }
        }
    }
    return cost < 1e-6f;
}

static bool solution_ok(const ik_problem_t *pb, const float q[KIN_NUM_JOINTS])
{
    float pc[3], ac[3];
    kin_forward_vec(pb->model, q, pc, ac, NULL);
    const vec3 e = sub3(v3(pc[0], pc[1], pc[2]), pb->p);
    const float cos_err = dot3(v3(ac[0], ac[1], ac[2]), pb->a);
    return norm3(e) <= KIN_POS_TOL && cos_err >= cosf(KIN_DIR_TOL);
}

kin_status_t kin_inverse(const kin_model_t *model, const kin_pose_t *target, const kin_ik_options_t *opts,
                         float q_out[KIN_NUM_JOINTS], int *bad_joint)
{
    if (model == NULL || target == NULL || q_out == NULL) {
        return KIN_ERR_INVALID_ARG;
    }
    const float zero_seed[KIN_NUM_JOINTS] = {0};
    const float *seed = (opts && opts->seed) ? opts->seed : zero_seed;
    const ik_problem_t pb = {
        .model = model,
        .p = v3(target->x, target->y, target->z),
        .a = pose_dir(target),
        .elbow = opts ? opts->elbow : KIN_ELBOW_UP,
        .seed0 = seed[0],
    };

    /*
     * The wrist centre W lies in the arm plane, so q1..q3 follow from W in closed form and q4, q5 from the
     * approach direction. W depends on the J5 axis (tool_offset), which depends on q4: solve for W with
     * Newton, for the preferred wrist flip first. Near singular configurations fall back to a joint-space
     * least-squares refinement.
     */
    const float refs[2] = {seed[3], kin_wrap_angle(seed[3] + KIN_PI)};
    /* Start from the seed's wrist centre (solution closest to the current pose), then from the estimate
     * that ignores the tool offset. */
    const vec3 w_est = axpy(-model->tool_length, pb.a, pb.p);
    vec3 w_seed = w_est;
    const bool has_seed = opts && opts->seed;
    if (has_seed) {
        const float phi = seed[1] + seed[2];
        const arm_frame_t fr = arm_frame(seed[0], phi);
        const float r = model->a2 * cosf(seed[1]) + model->a3 * cosf(phi);
        w_seed = axpy(r, fr.x1, v3(0.0f, 0.0f, model->d1 + model->a2 * sinf(seed[1]) + model->a3 * sinf(phi)));
    }
    float q[KIN_NUM_JOINTS];
    bool found = false, clamped = false;
    if (has_seed) {
        found = solve_wrist_centre(&pb, w_seed, refs[0], q, &clamped) && solution_ok(&pb, q);
    }
    for (int k = 0; k < 2 && !found; k++) {
        found = solve_wrist_centre(&pb, w_est, refs[k], q, &clamped) && solution_ok(&pb, q);
    }
    if (!found && has_seed) {
        for (int i = 0; i < KIN_NUM_JOINTS; i++) {
            q[i] = seed[i];
        }
        found = refine_joints(&pb, q) && solution_ok(&pb, q);
    }
    for (int k = 0; k < 2 && !found; k++) {
        bool c;
        wrist_map(&pb, w_est, refs[k], q, &c);
        found = refine_joints(&pb, q) && solution_ok(&pb, q);
    }
    if (!found) {
        return clamped ? KIN_ERR_UNREACHABLE : KIN_ERR_NO_CONVERGENCE;
    }

    for (int i = 0; i < KIN_NUM_JOINTS; i++) {
        q_out[i] = kin_wrap_angle(q[i]);
    }
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
    case KIN_ERR_NO_CONVERGENCE:
        return "no solution (near a singularity)";
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
