#include <math.h>

#include "kinematics.h"
#include "unity.h"

#define DEG(x) ((float)(x) * 3.14159265358979323846f / 180.0f)

static kin_model_t model;

void setUp(void)
{
    model = (kin_model_t){
        .d1 = 100.0f,
        .a1 = 20.0f,
        .a2 = 150.0f,
        .a3 = 150.0f,
        .d5 = 60.0f,
        .q_min = {DEG(-180), DEG(-180), DEG(-180), DEG(-180), DEG(-180)},
        .q_max = {DEG(180), DEG(180), DEG(180), DEG(180), DEG(180)},
    };
}

void tearDown(void) {}

static void test_model_validation(void)
{
    TEST_ASSERT_TRUE(kin_model_is_valid(&model));
    kin_model_t bad = model;
    bad.a2 = 0.0f;
    TEST_ASSERT_FALSE(kin_model_is_valid(&bad));
    bad = model;
    bad.q_min[2] = bad.q_max[2];
    TEST_ASSERT_FALSE(kin_model_is_valid(&bad));
    TEST_ASSERT_FALSE(kin_model_is_valid(NULL));
}

static void test_forward_zero_pose_is_stretched_horizontal(void)
{
    const float q[KIN_NUM_JOINTS] = {0, 0, 0, 0, 0};
    kin_pose_t p;
    TEST_ASSERT_EQUAL(KIN_OK, kin_forward(&model, q, &p));
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 20.0f + 150.0f + 150.0f + 60.0f, p.x);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, p.y);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 100.0f, p.z);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, p.pitch);
}

static void test_forward_known_pose(void)
{
    /* Base 90deg, upper arm vertical, forearm horizontal forward, tool pointing down. */
    const float q[KIN_NUM_JOINTS] = {DEG(90), DEG(90), DEG(-90), DEG(-90), DEG(30)};
    kin_pose_t p;
    TEST_ASSERT_EQUAL(KIN_OK, kin_forward(&model, q, &p));
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, p.x);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 20.0f + 150.0f, p.y);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 100.0f + 150.0f - 60.0f, p.z);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, DEG(-90), p.pitch);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, DEG(30), p.roll);
}

/* Elbow position in the arm plane, relative to the shoulder axis. */
static void elbow_point(const float q[KIN_NUM_JOINTS], float *r, float *z)
{
    *r = model.a2 * cosf(q[1]);
    *z = model.a2 * sinf(q[1]);
}

static void check_round_trip(kin_elbow_t elbow)
{
    int checked = 0;
    for (int b = -150; b <= 150; b += 50) {
        for (int s = -30; s <= 150; s += 15) {
            for (int e = -150; e <= 150; e += 15) {
                for (int w = -120; w <= 120; w += 40) {
                    /* Elbow-up solutions have q3 <= 0, elbow-down q3 >= 0. */
                    if ((elbow == KIN_ELBOW_UP && e > 0) || (elbow == KIN_ELBOW_DOWN && e < 0)) {
                        continue;
                    }
                    const float q[KIN_NUM_JOINTS] = {DEG(b), DEG(s), DEG(e), DEG(w), DEG(45)};
                    kin_pose_t pose;
                    kin_forward(&model, q, &pose);
                    if (sqrtf(pose.x * pose.x + pose.y * pose.y) < 1.0f) {
                        continue; /* near the singular base axis */
                    }
                    /* The shoulder-elbow pair is ambiguous when the wrist sits "behind" the base. */
                    const float r = sqrtf(pose.x * pose.x + pose.y * pose.y);
                    const float rw = r - model.a1 - model.d5 * cosf(pose.pitch);
                    const float r_expected = model.a1 + model.a2 * cosf(q[1]) + model.a3 * cosf(q[1] + q[2]) +
                                             model.d5 * cosf(q[1] + q[2] + q[3]);
                    if (r_expected < 0.0f || rw < 0.0f) {
                        continue;
                    }

                    float sol[KIN_NUM_JOINTS];
                    const kin_status_t st = kin_inverse(&model, &pose, elbow, sol, NULL);
                    TEST_ASSERT_EQUAL_MESSAGE(KIN_OK, st, kin_status_str(st));

                    kin_pose_t back;
                    kin_forward(&model, sol, &back);
                    TEST_ASSERT_FLOAT_WITHIN(1e-2f, pose.x, back.x);
                    TEST_ASSERT_FLOAT_WITHIN(1e-2f, pose.y, back.y);
                    TEST_ASSERT_FLOAT_WITHIN(1e-2f, pose.z, back.z);
                    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, kin_wrap_angle(pose.pitch - back.pitch));
                    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, kin_wrap_angle(pose.roll - back.roll));
                    checked++;
                }
            }
        }
    }
    TEST_ASSERT_GREATER_THAN(200, checked);
}

static void test_round_trip_elbow_up(void)
{
    check_round_trip(KIN_ELBOW_UP);
}

static void test_round_trip_elbow_down(void)
{
    check_round_trip(KIN_ELBOW_DOWN);
}

static void test_elbow_up_is_above_shoulder_wrist_line(void)
{
    /* Wrist straight ahead at shoulder height: elbow up must be above z = 0, elbow down below. */
    const kin_pose_t target = {.x = 20.0f + 212.13f + 60.0f, .y = 0, .z = 100.0f, .pitch = 0, .roll = 0};
    float up[KIN_NUM_JOINTS], down[KIN_NUM_JOINTS];
    TEST_ASSERT_EQUAL(KIN_OK, kin_inverse(&model, &target, KIN_ELBOW_UP, up, NULL));
    TEST_ASSERT_EQUAL(KIN_OK, kin_inverse(&model, &target, KIN_ELBOW_DOWN, down, NULL));
    float r, z;
    elbow_point(up, &r, &z);
    TEST_ASSERT_GREATER_THAN_FLOAT(100.0f, z);
    elbow_point(down, &r, &z);
    TEST_ASSERT_LESS_THAN_FLOAT(-100.0f, z);
    TEST_ASSERT_FLOAT_WITHIN(1e-2f, DEG(45), up[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-2f, DEG(-90), up[2]);
}

static void test_unreachable_and_singular(void)
{
    float q[KIN_NUM_JOINTS];
    const kin_pose_t far = {.x = 1000.0f, .y = 0, .z = 100.0f, .pitch = 0, .roll = 0};
    TEST_ASSERT_EQUAL(KIN_ERR_UNREACHABLE, kin_inverse(&model, &far, KIN_ELBOW_UP, q, NULL));
    const kin_pose_t axis = {.x = 0.0f, .y = 0.0f, .z = 300.0f, .pitch = 0, .roll = 0};
    TEST_ASSERT_EQUAL(KIN_ERR_SINGULAR, kin_inverse(&model, &axis, KIN_ELBOW_UP, q, NULL));
    TEST_ASSERT_EQUAL(KIN_ERR_INVALID_ARG, kin_inverse(&model, NULL, KIN_ELBOW_UP, q, NULL));
}

static void test_boundary_of_workspace_is_reachable(void)
{
    /* Fully stretched arm: cosine exactly 1, must not be rejected by rounding. */
    const float q[KIN_NUM_JOINTS] = {DEG(10), DEG(20), 0, 0, 0};
    kin_pose_t p;
    kin_forward(&model, q, &p);
    float sol[KIN_NUM_JOINTS];
    TEST_ASSERT_EQUAL(KIN_OK, kin_inverse(&model, &p, KIN_ELBOW_UP, sol, NULL));
    TEST_ASSERT_FLOAT_WITHIN(2e-2f, DEG(20), sol[1]);
}

static void test_joint_limits_reported(void)
{
    model.q_min[2] = DEG(-45);
    const kin_pose_t target = {.x = 20.0f + 212.13f + 60.0f, .y = 0, .z = 100.0f, .pitch = 0, .roll = 0};
    float q[KIN_NUM_JOINTS];
    int bad = -1;
    TEST_ASSERT_EQUAL(KIN_ERR_JOINT_LIMIT, kin_inverse(&model, &target, KIN_ELBOW_UP, q, &bad));
    TEST_ASSERT_EQUAL(2, bad);
}

static void test_wrap_angle(void)
{
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, kin_wrap_angle(DEG(360)));
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, DEG(-170), kin_wrap_angle(DEG(190)));
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, DEG(170), kin_wrap_angle(DEG(-190)));
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, DEG(180), kin_wrap_angle(DEG(-180)));
}

static void test_drive_mapping(void)
{
    /* 200 full steps x 16 microsteps x 40:1 = 32000 steps per 90 degrees. */
    const kin_drive_t drive = {.steps_per_rad = kin_steps_per_rad(200, 16, 40.0f), .invert = false};
    TEST_ASSERT_EQUAL_INT32(32000, kin_joint_to_steps(&drive, DEG(90)));
    TEST_ASSERT_EQUAL_INT32(-32000, kin_joint_to_steps(&drive, DEG(-90)));
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, DEG(90), kin_steps_to_joint(&drive, 32000));

    const kin_drive_t inv = {.steps_per_rad = drive.steps_per_rad, .invert = true};
    TEST_ASSERT_EQUAL_INT32(-32000, kin_joint_to_steps(&inv, DEG(90)));
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, DEG(90), kin_steps_to_joint(&inv, -32000));

    /* Rounding, not truncation. */
    TEST_ASSERT_EQUAL_INT32(1, kin_joint_to_steps(&drive, 0.6f / drive.steps_per_rad));
    TEST_ASSERT_EQUAL_INT32(-1, kin_joint_to_steps(&drive, -0.6f / drive.steps_per_rad));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_model_validation);
    RUN_TEST(test_forward_zero_pose_is_stretched_horizontal);
    RUN_TEST(test_forward_known_pose);
    RUN_TEST(test_round_trip_elbow_up);
    RUN_TEST(test_round_trip_elbow_down);
    RUN_TEST(test_elbow_up_is_above_shoulder_wrist_line);
    RUN_TEST(test_unreachable_and_singular);
    RUN_TEST(test_boundary_of_workspace_is_reachable);
    RUN_TEST(test_joint_limits_reported);
    RUN_TEST(test_wrap_angle);
    RUN_TEST(test_drive_mapping);
    return UNITY_END();
}
