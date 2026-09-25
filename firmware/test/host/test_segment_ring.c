#include "segment_ring.h"
#include "unity.h"

static segment_ring_t ring;

void setUp(void)
{
    segment_ring_init(&ring);
}

void tearDown(void) {}

static void test_empty_ring(void)
{
    stepgen_segment_t s;
    TEST_ASSERT_EQUAL_UINT32(0, segment_ring_used(&ring));
    TEST_ASSERT_EQUAL_UINT32(SEGMENT_RING_CAPACITY - 1, segment_ring_free(&ring));
    TEST_ASSERT_FALSE(segment_ring_pop(&ring, &s));
}

static void test_fifo_order_and_wraparound(void)
{
    int16_t next_in = 0, next_out = 0;
    for (int round = 0; round < 10; round++) {
        for (int i = 0; i < 50; i++) {
            const stepgen_segment_t s = {.steps = {next_in++}};
            TEST_ASSERT_TRUE(segment_ring_push(&ring, &s));
        }
        for (int i = 0; i < 50; i++) {
            stepgen_segment_t s;
            TEST_ASSERT_TRUE(segment_ring_pop(&ring, &s));
            TEST_ASSERT_EQUAL_INT16(next_out++, s.steps[0]);
        }
    }
    TEST_ASSERT_EQUAL_UINT32(0, segment_ring_used(&ring));
}

static void test_full_ring_rejects_push(void)
{
    const stepgen_segment_t s = {.steps = {1}};
    for (uint32_t i = 0; i < SEGMENT_RING_CAPACITY - 1; i++) {
        TEST_ASSERT_TRUE(segment_ring_push(&ring, &s));
    }
    TEST_ASSERT_FALSE(segment_ring_push(&ring, &s));
    TEST_ASSERT_EQUAL_UINT32(0, segment_ring_free(&ring));
}

static void test_flush(void)
{
    const stepgen_segment_t s = {.steps = {1}};
    segment_ring_push(&ring, &s);
    segment_ring_push(&ring, &s);
    segment_ring_flush(&ring);
    TEST_ASSERT_EQUAL_UINT32(0, segment_ring_used(&ring));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_empty_ring);
    RUN_TEST(test_fifo_order_and_wraparound);
    RUN_TEST(test_full_ring_rejects_push);
    RUN_TEST(test_flush);
    return UNITY_END();
}
