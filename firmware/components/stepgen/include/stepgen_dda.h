/*
 * Integer DDA that spreads the steps of a segment evenly over its ticks.
 *
 * Pure C, integer only (safe inside an ISR, no FPU). With T ticks per segment and n <= T steps for an axis,
 * exactly n steps are produced, at most one per tick, evenly spaced (+/- 1 tick).
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "stepgen_segment.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t num_axes;
    uint32_t ticks_per_segment; /* T */
    uint32_t tick;              /* ticks already produced in the current segment */
    bool active;                /* a segment is loaded and not finished */
    uint8_t seg_flags;
    uint32_t dir_mask;          /* bit i set: axis i moves in the positive direction */
    uint16_t n[STEPGEN_MAX_AXES];
    uint32_t acc[STEPGEN_MAX_AXES];
    int32_t pos[STEPGEN_MAX_AXES];
} stepgen_dda_t;

void stepgen_dda_init(stepgen_dda_t *dda, uint32_t num_axes, uint32_t ticks_per_segment);

/*
 * Loads the next segment. Returns false (and loads nothing) if any |steps| exceeds ticks_per_segment.
 * Axes with zero steps keep their previous direction so no DIR edge is generated needlessly.
 */
bool stepgen_dda_load(stepgen_dda_t *dda, const stepgen_segment_t *seg);

/* True when the loaded segment has produced all its ticks (or nothing is loaded). */
static inline bool stepgen_dda_segment_done(const stepgen_dda_t *dda)
{
    return !dda->active || dda->tick >= dda->ticks_per_segment;
}

/* Advances one tick. Returns the mask of axes that step during this tick and updates positions. */
uint32_t stepgen_dda_tick(stepgen_dda_t *dda);

/* Drops the current segment (abort). Positions are kept. */
void stepgen_dda_reset(stepgen_dda_t *dda);

#ifdef __cplusplus
}
#endif
