/*
 * Lock-free single-producer / single-consumer ring of step segments.
 *
 * Producer: motion task. Consumer: stepgen ISR. Pure C11 atomics, usable from an ISR and on the host.
 * Capacity must be a power of two; one slot is never used, so up to SEGMENT_RING_CAPACITY - 1 entries fit.
 */
#pragma once

#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>

#include "stepgen_segment.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SEGMENT_RING_CAPACITY 64u
_Static_assert((SEGMENT_RING_CAPACITY & (SEGMENT_RING_CAPACITY - 1u)) == 0, "capacity must be a power of 2");

typedef struct {
    stepgen_segment_t slots[SEGMENT_RING_CAPACITY];
    atomic_uint head; /* next slot written by the producer */
    atomic_uint tail; /* next slot read by the consumer */
} segment_ring_t;

static inline void segment_ring_init(segment_ring_t *r)
{
    atomic_init(&r->head, 0u);
    atomic_init(&r->tail, 0u);
}

static inline uint32_t segment_ring_used(segment_ring_t *r)
{
    const unsigned head = atomic_load_explicit(&r->head, memory_order_acquire);
    const unsigned tail = atomic_load_explicit(&r->tail, memory_order_acquire);
    return (head - tail) & (SEGMENT_RING_CAPACITY - 1u);
}

static inline uint32_t segment_ring_free(segment_ring_t *r)
{
    return SEGMENT_RING_CAPACITY - 1u - segment_ring_used(r);
}

/* Producer side. Returns false if full. */
static inline bool segment_ring_push(segment_ring_t *r, const stepgen_segment_t *seg)
{
    const unsigned head = atomic_load_explicit(&r->head, memory_order_relaxed);
    const unsigned tail = atomic_load_explicit(&r->tail, memory_order_acquire);
    const unsigned next = (head + 1u) & (SEGMENT_RING_CAPACITY - 1u);
    if (next == tail) {
        return false;
    }
    r->slots[head] = *seg;
    atomic_store_explicit(&r->head, next, memory_order_release);
    return true;
}

/* Consumer side. Returns false if empty. */
static inline bool segment_ring_pop(segment_ring_t *r, stepgen_segment_t *out)
{
    const unsigned tail = atomic_load_explicit(&r->tail, memory_order_relaxed);
    const unsigned head = atomic_load_explicit(&r->head, memory_order_acquire);
    if (tail == head) {
        return false;
    }
    *out = r->slots[tail];
    atomic_store_explicit(&r->tail, (tail + 1u) & (SEGMENT_RING_CAPACITY - 1u), memory_order_release);
    return true;
}

/* Consumer side: discards everything queued (used on abort). */
static inline void segment_ring_flush(segment_ring_t *r)
{
    atomic_store_explicit(&r->tail, atomic_load_explicit(&r->head, memory_order_acquire), memory_order_release);
}

#ifdef __cplusplus
}
#endif
