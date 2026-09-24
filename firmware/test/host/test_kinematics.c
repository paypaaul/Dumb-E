#include <math.h>
#include <stdlib.h>

#include "kinematics.h"
#include "unity.h"

#define DEG(x) ((float)(x) * 3.14159265358979323846f / 180.0f)

/* Geometry measured on the author's CAD model (not in the repo). */
static kin_model_t model;

void setUp(void)
{
    model = (kin_model_t){
        .d1 = 104.8f,
        .a2 = 159.2f,
        .a3 = 151.8f,
        .tool_offset = 67.4f,
        .tool_length = 155.4f,
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

static void test_forward_cad_pose(void)
{
    /* Arm straight up as in the CAD: TCP at (0, 67.4, 571.2), pointing up. */
    const float q[KIN_NUM_JOINTS] = {0, DEG(90), 0, 0, 0};
    float p[3], a[3], n[3];
    kin_forward_vec(&model, q, p, a, n);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, p[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 67.4f, p[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-2f, 104.8f + 159.2f + 151.8f + 155.4f, p[2]);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, a[2]);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, n[1]);
}

static void test_forward_known_pose(void)
{
    /* Upper arm vertical, forearm horizontal forward, wrist pitched down, base turned 90 deg. */
    const float q[KIN_NUM_JOINTS] = {DEG(90), DEG(90), DEG(-90), 0, DEG(-90)};
    kin_pose_t p;
    TEST_ASSERT_EQUAL(KIN_OK, kin_forward(&model, q, &p));
    /* Arm plane is the y-z plane; the J5 axis is the arm-plane normal (-x). */
    TEST_ASSERT_FLOAT_WITHIN(1e-2f, -67.4f, p.x);
    TEST_ASSERT_FLOAT_WITHIN(1e-2f, 151.8f, p.y);
    TEST_ASSERT_FLOAT_WITHIN(1e-2f, 104.8f + 159.2f - 155.4f, p.z);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, DEG(-90), p.pitch);
}

static void test_forearm_roll_moves_tool_out_of_plane(void)
{
    /* Forearm horizontal, roll 90 deg, wrist pitched 90 deg: the tool swings sideways. */
    const float q[KIN_NUM_JOINTS] = {0, DEG(90), DEG(-90), DEG(90), DEG(90)};
    float p[3], a[3];
    kin_forward_vec(&model, q, p, a, NULL);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, a[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, -1.0f, a[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, a[2]);
}

static float pose_distance(const kin_pose_t *a, const kin_pose_t *b)
{
    return sqrtf((a->x - b->x) * (a->x - b->x) + (a->y - b->y) * (a->y - b->y) + (a->z - b->z) * (a->z - b->z));
}

static float dir_angle(const kin_pose_t *a, const kin_pose_t *b)
{
    const float da[3] = {cosf(a->pitch) * cosf(a->yaw), cosf(a->pitch) * sinf(a->yaw), sinf(a->pitch)};
    const float db[3] = {cosf(b->pitch) * cosf(b->yaw), cosf(b->pitch) * sinf(b->yaw), sinf(b->pitch)};
    const float c = da[0] * db[0] + da[1] * db[1] + da[2] * db[2];
    return acosf(fminf(1.0f, c));
}

static void test_round_trip_random(void)
{
    srand(42);
    int solved = 0, tried = 0, comparable = 0, recovered = 0;
    for (int k = 0; k < 4000; k++) {
        const float q[KIN_NUM_JOINTS] = {
            DEG((rand() % 300) - 150), DEG((rand() % 150) + 10), DEG((rand() % 280) - 140),
            DEG((rand() % 340) - 170), DEG((rand() % 220) - 110),
        };
        kin_pose_t pose;
        kin_forward(&model, q, &pose);
        tried++;
        const kin_ik_options_t opts = {.elbow = q[2] <= 0 ? KIN_ELBOW_UP : KIN_ELBOW_DOWN, .seed = q};
        float sol[KIN_NUM_JOINTS];
        kin_status_t st = kin_inverse(&model, &pose, &opts, sol, NULL);
        if (st != KIN_OK) {
            /* Wrist centre behind the base: the front solution uses the other elbow. */
            const kin_ik_options_t other = {.elbow = opts.elbow == KIN_ELBOW_UP ? KIN_ELBOW_DOWN : KIN_ELBOW_UP,
                                            .seed = q};
            st = kin_inverse(&model, &pose, &other, sol, NULL);
        }
        if (st == KIN_ERR_NO_CONVERGENCE || st == KIN_ERR_UNREACHABLE) {
            continue; /* close to a singularity: allowed, but must stay rare (checked below) */
        }
        TEST_ASSERT_EQUAL_MESSAGE(KIN_OK, st, kin_status_str(st));
        kin_pose_t back;
        kin_forward(&model, sol, &back);
        TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.0f, pose_distance(&pose, &back));
        TEST_ASSERT_FLOAT_WITHIN(2e-3f, 0.0f, dir_angle(&pose, &back));
        /* With the right seed and elbow the original joints are usually recovered. Near singular
         * configurations two solutions can be very close, so only count them; skip poses whose wrist centre
         * was behind the base (the equivalent front solution is returned). */
        const float r_orig = model.a2 * cosf(q[1]) + model.a3 * cosf(q[1] + q[2]);
        if (r_orig > 1.0f && fabsf(q[4]) > DEG(2)) {
            comparable++;
            bool same = true;
            for (int i = 0; i < KIN_NUM_JOINTS; i++) {
                same = same && fabsf(kin_wrap_angle(sol[i] - q[i])) < 2e-3f;
            }
            recovered += same ? 1 : 0;
        }
        solved++;
    }
    TEST_ASSERT_GREATER_THAN(tried * 99 / 100, solved);
    TEST_ASSERT_GREATER_THAN(comparable * 95 / 100, recovered);
}

static void test_ik_without_seed_points_down(void)
{
    /* Typical pick pose: in front of the robot, tool pointing down. */
    const kin_pose_t target = {.x = 250.0f, .y = 0.0f, .z = 60.0f, .pitch = DEG(-90), .yaw = 0.0f};
    float q[KIN_NUM_JOINTS];
    TEST_ASSERT_EQUAL(KIN_OK, kin_inverse(&model, &target, NULL, q, NULL));
    kin_pose_t back;
    kin_forward(&model, q, &back);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.0f, pose_distance(&target, &back));
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, DEG(-90), back.pitch);
    TEST_ASSERT_TRUE(q[2] <= 0.0f); /* elbow up by default */
}

static void test_elbow_selection(void)
{
    const kin_pose_t target = {.x = 200.0f, .y = 50.0f, .z = 150.0f, .pitch = 0.0f, .yaw = 0.0f};
    float up[KIN_NUM_JOINTS], down[KIN_NUM_JOINTS];
    const kin_ik_options_t o_up = {.elbow = KIN_ELBOW_UP}, o_down = {.elbow = KIN_ELBOW_DOWN};
    TEST_ASSERT_EQUAL(KIN_OK, kin_inverse(&model, &target, &o_up, up, NULL));
    TEST_ASSERT_EQUAL(KIN_OK, kin_inverse(&model, &target, &o_down, down, NULL));
    TEST_ASSERT_TRUE(up[2] < 0.0f);
    TEST_ASSERT_TRUE(down[2] > 0.0f);
}

static void test_cad_pose_is_solvable_with_seed(void)
{
    /* Wrist centre on the base axis and approach along the forearm: both singular, resolved by the seed. */
    const float q[KIN_NUM_JOINTS] = {DEG(30), DEG(90), 0, DEG(-40), 0};
    kin_pose_t pose;
    kin_forward(&model, q, &pose);
    const kin_ik_options_t opts = {.elbow = KIN_ELBOW_UP, .seed = q};
    float sol[KIN_NUM_JOINTS];
    TEST_ASSERT_EQUAL(KIN_OK, kin_inverse(&model, &pose, &opts, sol, NULL));
    kin_pose_t back;
    kin_forward(&model, sol, &back);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.0f, pose_distance(&pose, &back));
}

static void test_unreachable(void)
{
    float q[KIN_NUM_JOINTS];
    const kin_pose_t far = {.x = 1000.0f, .y = 0, .z = 100.0f, .pitch = 0, .yaw = 0};
    TEST_ASSERT_EQUAL(KIN_ERR_UNREACHABLE, kin_inverse(&model, &far, NULL, q, NULL));
    TEST_ASSERT_EQUAL(KIN_ERR_INVALID_ARG, kin_inverse(&model, NULL, NULL, q, NULL));
}

static void test_joint_limits_reported(void)
{
    model.q_min[2] = DEG(-10); /* elbow can barely bend upwards */
    const kin_pose_t target = {.x = 250.0f, .y = 0.0f, .z = 60.0f, .pitch = DEG(-90), .yaw = 0.0f};
    float q[KIN_NUM_JOINTS];
    int bad = -1;
    TEST_ASSERT_EQUAL(KIN_ERR_JOINT_LIMIT, kin_inverse(&model, &target, NULL, q, &bad));
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
    /* 200 full steps x 16 microsteps x 20:1 = 64000 steps per output revolution. */
    const kin_drive_t drive = {.steps_per_rad = kin_steps_per_rad(200, 16, 20.0f), .invert = false};
    TEST_ASSERT_EQUAL_INT32(16000, kin_joint_to_steps(&drive, DEG(90)));
    TEST_ASSERT_EQUAL_INT32(-16000, kin_joint_to_steps(&drive, DEG(-90)));
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, DEG(90), kin_steps_to_joint(&drive, 16000));

    const kin_drive_t inv = {.steps_per_rad = drive.steps_per_rad, .invert = true};
    TEST_ASSERT_EQUAL_INT32(-16000, kin_joint_to_steps(&inv, DEG(90)));
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, DEG(90), kin_steps_to_joint(&inv, -16000));

    /* Rounding, not truncation. */
    TEST_ASSERT_EQUAL_INT32(1, kin_joint_to_steps(&drive, 0.6f / drive.steps_per_rad));
    TEST_ASSERT_EQUAL_INT32(-1, kin_joint_to_steps(&drive, -0.6f / drive.steps_per_rad));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_model_validation);
    RUN_TEST(test_forward_cad_pose);
    RUN_TEST(test_forward_known_pose);
    RUN_TEST(test_forearm_roll_moves_tool_out_of_plane);
    RUN_TEST(test_round_trip_random);
    RUN_TEST(test_ik_without_seed_points_down);
    RUN_TEST(test_elbow_selection);
    RUN_TEST(test_cad_pose_is_solvable_with_seed);
    RUN_TEST(test_unreachable);
    RUN_TEST(test_joint_limits_reported);
    RUN_TEST(test_wrap_angle);
    RUN_TEST(test_drive_mapping);
    return UNITY_END();
}
