#include <stdlib.h>

#include "stepgen_dda.h"
#include "unity.h"

#define T 40u

static stepgen_dda_t dda;

void setUp(void)
{
    stepgen_dda_init(&dda, 5, T);
}

void tearDown(void) {}

static void run_segment(uint32_t masks[T])
{
    for (uint32_t t = 0; t < T; t++) {
        masks[t] = stepgen_dda_tick(&dda);
    }
    TEST_ASSERT_TRUE(stepgen_dda_segment_done(&dda));
    TEST_ASSERT_EQUAL_UINT32(0, stepgen_dda_tick(&dda)); /* nothing past the end */
}

static void test_exact_step_counts_for_all_n(void)
{
    for (int n = -(int)T; n <= (int)T; n++) {
        stepgen_dda_init(&dda, 1, T);
        const stepgen_segment_t seg = {.steps = {(int16_t)n}};
        TEST_ASSERT_TRUE(stepgen_dda_load(&dda, &seg));
        uint32_t masks[T];
        run_segment(masks);
        int count = 0;
        for (uint32_t t = 0; t < T; t++) {
            count += (int)(masks[t] & 1u);
        }
        TEST_ASSERT_EQUAL_INT(abs(n), count);
        TEST_ASSERT_EQUAL_INT32(n, dda.pos[0]);
    }
}

static void test_steps_are_evenly_spaced(void)
{
    for (int n = 2; n <= (int)T; n++) {
        stepgen_dda_init(&dda, 1, T);
        const stepgen_segment_t seg = {.steps = {(int16_t)n}};
        stepgen_dda_load(&dda, &seg);
        uint32_t masks[T];
        run_segment(masks);
        int last = -1, min_gap = 1000, max_gap = 0;
        for (int t = 0; t < (int)T; t++) {
            if (masks[t] & 1u) {
                if (last >= 0) {
                    const int gap = t - last;
                    min_gap = gap < min_gap ? gap : min_gap;
                    max_gap = gap > max_gap ? gap : max_gap;
                }
                last = t;
            }
        }
        TEST_ASSERT_LESS_OR_EQUAL_INT(1, max_gap - min_gap);
    }
}

static void test_multi_axis_and_direction(void)
{
    const stepgen_segment_t seg = {.steps = {40, -20, 0, 7, -1}};
    TEST_ASSERT_TRUE(stepgen_dda_load(&dda, &seg));
    TEST_ASSERT_EQUAL_HEX32(0x1u | 0x4u | 0x8u, dda.dir_mask & 0x1Fu); /* axis 2 keeps its previous dir */
    uint32_t masks[T];
    run_segment(masks);
    TEST_ASSERT_EQUAL_INT32(40, dda.pos[0]);
    TEST_ASSERT_EQUAL_INT32(-20, dda.pos[1]);
    TEST_ASSERT_EQUAL_INT32(0, dda.pos[2]);
    TEST_ASSERT_EQUAL_INT32(7, dda.pos[3]);
    TEST_ASSERT_EQUAL_INT32(-1, dda.pos[4]);
}

static void test_rejects_too_many_steps(void)
{
    const stepgen_segment_t seg = {.steps = {41}};
    TEST_ASSERT_FALSE(stepgen_dda_load(&dda, &seg));
    TEST_ASSERT_TRUE(stepgen_dda_segment_done(&dda));
}

static void test_long_sequence_accumulates_exactly(void)
{
    int32_t expected = 0;
    srand(1234);
    for (int k = 0; k < 2000; k++) {
        const int n = (rand() % (2 * (int)T + 1)) - (int)T;
        const stepgen_segment_t seg = {.steps = {(int16_t)n}};
        TEST_ASSERT_TRUE(stepgen_dda_load(&dda, &seg));
        uint32_t masks[T];
        run_segment(masks);
        expected += n;
    }
    TEST_ASSERT_EQUAL_INT32(expected, dda.pos[0]);
}

static void test_reset_keeps_position(void)
{
    const stepgen_segment_t seg = {.steps = {40}};
    stepgen_dda_load(&dda, &seg);
    for (int t = 0; t < 10; t++) {
        stepgen_dda_tick(&dda);
    }
    stepgen_dda_reset(&dda);
    TEST_ASSERT_TRUE(stepgen_dda_segment_done(&dda));
    TEST_ASSERT_EQUAL_INT32(10, dda.pos[0]);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_exact_step_counts_for_all_n);
    RUN_TEST(test_steps_are_evenly_spaced);
    RUN_TEST(test_multi_axis_and_direction);
    RUN_TEST(test_rejects_too_many_steps);
    RUN_TEST(test_long_sequence_accumulates_exactly);
    RUN_TEST(test_reset_keeps_position);
    return UNITY_END();
}
