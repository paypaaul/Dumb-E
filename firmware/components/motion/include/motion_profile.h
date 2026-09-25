/*
 * 1-D motion profiles along a path parameter s, used to synchronise all joints.
 *
 * Pure C, no ESP-IDF dependencies. A profile is a sequence of constant-acceleration phases starting from an
 * initial state (s0, v0); it is evaluated in closed form (no numerical integration, no drift).
 */
#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MP_MAX_PHASES 3

typedef struct {
    float duration; /* [s] */
    float accel;    /* [unit/s^2] */
} mp_phase_t;

typedef struct {
    float s0, v0;     /* initial position and velocity */
    float s_end;      /* exact final position */
    float total_time; /* [s] */
    int num_phases;
    mp_phase_t phases[MP_MAX_PHASES];
} mp_profile_t;

/*
 * Rest-to-rest trapezoidal profile covering `distance` (> 0) with velocity <= v_max and |accel| <= a_max.
 * Falls back to a triangular profile when v_max cannot be reached. Returns false on invalid arguments.
 */
bool mp_profile_trapezoid(mp_profile_t *p, float distance, float v_max, float a_max);

/* Position and velocity at time t (clamped to [0, total_time]). Either output may be NULL. */
void mp_profile_eval(const mp_profile_t *p, float t, float *s, float *v);

/* True once t has reached the end of the profile. */
bool mp_profile_done(const mp_profile_t *p, float t);

/*
 * Replaces the profile with a controlled stop starting from its state at time t_now: constant deceleration
 * a_max until the velocity reaches zero. The new profile starts at t = 0.
 */
void mp_profile_stop(mp_profile_t *p, float t_now, float a_max);

/*
 * Limits of the common path parameter for a synchronised joint move q(s) = q0 + dq * s, s in [0, 1]:
 * sd_max = min_i v_max[i] / |dq[i]|, sdd_max = min_i a_max[i] / |dq[i]| over joints that move.
 * Returns false if no joint moves (all |dq| below `eps`) or a limit is not positive.
 */
bool mp_sync_limits(const float *dq, const float *v_max, const float *a_max, int n, float eps, float *sd_max,
                    float *sdd_max);

#ifdef __cplusplus
}
#endif
