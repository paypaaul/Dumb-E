#include <math.h>

#include "motion_profile.h"
#include "unity.h"

void setUp(void) {}
void tearDown(void) {}

/* Samples the profile and checks endpoints, monotonicity and velocity/acceleration bounds. */
static void check_profile(const mp_profile_t *p, float distance, float v_max, float a_max)
{
    const float dt = 1e-4f;
    float s_prev, v_prev;
    mp_profile_eval(p, 0.0f, &s_prev, &v_prev);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, s_prev);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, v_prev);
    for (float t = dt; t < p->total_time + 2 * dt; t += dt) {
        float s, v;
        mp_profile_eval(p, t, &s, &v);
        TEST_ASSERT_TRUE(s >= s_prev - 1e-6f);
        TEST_ASSERT_TRUE(v <= v_max * 1.0001f);
        TEST_ASSERT_TRUE(v >= -1e-5f);
        TEST_ASSERT_TRUE(fabsf(v - v_prev) / dt <= a_max * 1.01f);
        s_prev = s;
        v_prev = v;
    }
    float s_end, v_end;
    mp_profile_eval(p, p->total_time, &s_end, &v_end);
    TEST_ASSERT_EQUAL_FLOAT(distance, s_end);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, v_end);
    TEST_ASSERT_TRUE(mp_profile_done(p, p->total_time));
    TEST_ASSERT_FALSE(mp_profile_done(p, p->total_time * 0.5f));
}

static void test_trapezoid(void)
{
    mp_profile_t p;
    TEST_ASSERT_TRUE(mp_profile_trapezoid(&p, 1.0f, 0.5f, 1.0f));
    TEST_ASSERT_EQUAL(3, p.num_phases);
    /* t_acc = 0.5, d_acc = 0.125, cruise = 0.75 / 0.5 = 1.5 -> total 2.5 s */
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 2.5f, p.total_time);
    check_profile(&p, 1.0f, 0.5f, 1.0f);
    float s;
    mp_profile_eval(&p, p.total_time / 2.0f, &s, NULL);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.5f, s); /* symmetric */
}

static void test_triangle(void)
{
    mp_profile_t p;
    TEST_ASSERT_TRUE(mp_profile_trapezoid(&p, 1.0f, 10.0f, 4.0f));
    TEST_ASSERT_EQUAL(2, p.num_phases);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, p.total_time); /* 2 * sqrt(1/4) */
    check_profile(&p, 1.0f, 2.0f, 4.0f);
}

static void test_invalid_arguments(void)
{
    mp_profile_t p;
    TEST_ASSERT_FALSE(mp_profile_trapezoid(&p, 0.0f, 1.0f, 1.0f));
    TEST_ASSERT_FALSE(mp_profile_trapezoid(&p, 1.0f, 0.0f, 1.0f));
    TEST_ASSERT_FALSE(mp_profile_trapezoid(&p, 1.0f, 1.0f, -1.0f));
    TEST_ASSERT_FALSE(mp_profile_trapezoid(NULL, 1.0f, 1.0f, 1.0f));
}

static void test_stop_during_cruise(void)
{
    mp_profile_t p;
    mp_profile_trapezoid(&p, 1.0f, 0.5f, 1.0f);
    float s_now, v_now;
    mp_profile_eval(&p, 1.0f, &s_now, &v_now);
    mp_profile_stop(&p, 1.0f, 1.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.5f, p.total_time); /* 0.5 / 1.0 */
    float s, v;
    mp_profile_eval(&p, 0.0f, &s, &v);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, s_now, s);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, v_now, v);
    mp_profile_eval(&p, p.total_time, &s, &v);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, s_now + 0.125f, s);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, v);
    TEST_ASSERT_TRUE(s < 1.0f);
}

static void test_stop_at_rest(void)
{
    mp_profile_t p;
    mp_profile_trapezoid(&p, 1.0f, 0.5f, 1.0f);
    mp_profile_stop(&p, 0.0f, 1.0f);
    TEST_ASSERT_TRUE(mp_profile_done(&p, 0.0f));
    float s;
    mp_profile_eval(&p, 0.0f, &s, NULL);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, s);
}

static void test_sync_limits(void)
{
    const float dq[3] = {1.0f, -0.25f, 0.0f};
    const float v[3] = {2.0f, 0.1f, 0.0f}; /* joint 2 does not move, its limits are ignored */
    const float a[3] = {1.0f, 1.0f, 0.0f};
    float sd, sdd;
    TEST_ASSERT_TRUE(mp_sync_limits(dq, v, a, 3, 1e-6f, &sd, &sdd));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.4f, sd);  /* min(2/1, 0.1/0.25) */
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, sdd); /* min(1/1, 1/0.25) */

    const float still[3] = {0.0f, 0.0f, 0.0f};
    TEST_ASSERT_FALSE(mp_sync_limits(still, v, a, 3, 1e-6f, &sd, &sdd));
}

static void test_synchronised_joints_respect_their_limits(void)
{
    const float dq[2] = {2.0f, -0.5f};
    const float v[2] = {1.0f, 0.2f};
    const float a[2] = {2.0f, 0.3f};
    float sd, sdd;
    TEST_ASSERT_TRUE(mp_sync_limits(dq, v, a, 2, 1e-6f, &sd, &sdd));
    mp_profile_t p;
    TEST_ASSERT_TRUE(mp_profile_trapezoid(&p, 1.0f, sd, sdd));
    const float dt = 1e-3f;
    float s_prev = 0.0f, sv_prev = 0.0f;
    for (float t = dt; t <= p.total_time; t += dt) {
        float s, sv;
        mp_profile_eval(&p, t, &s, &sv);
        for (int i = 0; i < 2; i++) {
            TEST_ASSERT_TRUE(fabsf(dq[i]) * sv <= v[i] * 1.0001f);
            TEST_ASSERT_TRUE(fabsf(dq[i]) * fabsf(sv - sv_prev) / dt <= a[i] * 1.01f);
        }
        s_prev = s;
        sv_prev = sv;
    }
    (void)s_prev;
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_trapezoid);
    RUN_TEST(test_triangle);
    RUN_TEST(test_invalid_arguments);
    RUN_TEST(test_stop_during_cruise);
    RUN_TEST(test_stop_at_rest);
    RUN_TEST(test_sync_limits);
    RUN_TEST(test_synchronised_joints_respect_their_limits);
    return UNITY_END();
}
