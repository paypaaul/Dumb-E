#include "motion_profile.h"

#include <math.h>
#include <stddef.h>

bool mp_profile_trapezoid(mp_profile_t *p, float distance, float v_max, float a_max)
{
    if (p == NULL || !(distance > 0.0f) || !(v_max > 0.0f) || !(a_max > 0.0f)) {
        return false;
    }
    p->s0 = 0.0f;
    p->v0 = 0.0f;
    p->s_end = distance;

    const float d_acc = v_max * v_max / (2.0f * a_max);
    if (2.0f * d_acc >= distance) {
        /* Triangular: v_max is never reached. */
        const float t_acc = sqrtf(distance / a_max);
        p->num_phases = 2;
        p->phases[0] = (mp_phase_t){.duration = t_acc, .accel = a_max};
        p->phases[1] = (mp_phase_t){.duration = t_acc, .accel = -a_max};
        p->total_time = 2.0f * t_acc;
    } else {
        const float t_acc = v_max / a_max;
        const float t_cruise = (distance - 2.0f * d_acc) / v_max;
        p->num_phases = 3;
        p->phases[0] = (mp_phase_t){.duration = t_acc, .accel = a_max};
        p->phases[1] = (mp_phase_t){.duration = t_cruise, .accel = 0.0f};
        p->phases[2] = (mp_phase_t){.duration = t_acc, .accel = -a_max};
        p->total_time = 2.0f * t_acc + t_cruise;
    }
    return true;
}

void mp_profile_eval(const mp_profile_t *p, float t, float *s, float *v)
{
    float pos = p->s0;
    float vel = p->v0;
    if (t >= p->total_time) {
        pos = p->s_end;
        vel = 0.0f;
    } else if (t > 0.0f) {
        float remaining = t;
        for (int i = 0; i < p->num_phases; i++) {
            const float dt = fminf(remaining, p->phases[i].duration);
            const float a = p->phases[i].accel;
            pos += vel * dt + 0.5f * a * dt * dt;
            vel += a * dt;
            remaining -= dt;
            if (remaining <= 0.0f) {
                break;
            }
        }
    }
    if (s != NULL) {
        *s = pos;
    }
    if (v != NULL) {
        *v = vel;
    }
}

bool mp_profile_done(const mp_profile_t *p, float t)
{
    return t >= p->total_time;
}

void mp_profile_stop(mp_profile_t *p, float t_now, float a_max)
{
    float s, v;
    mp_profile_eval(p, t_now, &s, &v);
    p->s0 = s;
    p->v0 = v;
    if (!(a_max > 0.0f) || fabsf(v) <= 0.0f) {
        p->num_phases = 0;
        p->total_time = 0.0f;
        p->s_end = s;
        return;
    }
    const float t_stop = fabsf(v) / a_max;
    p->num_phases = 1;
    p->phases[0] = (mp_phase_t){.duration = t_stop, .accel = v > 0.0f ? -a_max : a_max};
    p->total_time = t_stop;
    p->s_end = s + 0.5f * v * t_stop;
}

bool mp_sync_limits(const float *dq, const float *v_max, const float *a_max, int n, float eps, float *sd_max,
                    float *sdd_max)
{
    float sd = INFINITY;
    float sdd = INFINITY;
    bool moving = false;
    for (int i = 0; i < n; i++) {
        const float d = fabsf(dq[i]);
        if (d <= eps) {
            continue;
        }
        if (!(v_max[i] > 0.0f) || !(a_max[i] > 0.0f)) {
            return false;
        }
        moving = true;
        sd = fminf(sd, v_max[i] / d);
        sdd = fminf(sdd, a_max[i] / d);
    }
    if (!moving) {
        return false;
    }
    *sd_max = sd;
    *sdd_max = sdd;
    return true;
}
