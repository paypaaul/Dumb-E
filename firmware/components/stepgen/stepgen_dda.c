#include "stepgen_dda.h"

#include <string.h>

void stepgen_dda_init(stepgen_dda_t *dda, uint32_t num_axes, uint32_t ticks_per_segment)
{
    memset(dda, 0, sizeof(*dda));
    dda->num_axes = num_axes > STEPGEN_MAX_AXES ? STEPGEN_MAX_AXES : num_axes;
    dda->ticks_per_segment = ticks_per_segment;
    dda->dir_mask = (1u << dda->num_axes) - 1u;
}

bool stepgen_dda_load(stepgen_dda_t *dda, const stepgen_segment_t *seg)
{
    const int32_t limit = (int32_t)dda->ticks_per_segment;
    for (uint32_t i = 0; i < dda->num_axes; i++) {
        const int32_t s = seg->steps[i];
        if (s > limit || s < -limit) {
            return false;
        }
    }
    const uint32_t half = dda->ticks_per_segment / 2u;
    for (uint32_t i = 0; i < dda->num_axes; i++) {
        const int32_t s = seg->steps[i];
        if (s > 0) {
            dda->dir_mask |= 1u << i;
        } else if (s < 0) {
            dda->dir_mask &= ~(1u << i);
        }
        dda->n[i] = (uint16_t)(s < 0 ? -s : s);
        /* Starting at T/2 centres the steps inside the segment; the accumulator returns to T/2 at the end. */
        dda->acc[i] = half;
    }
    dda->tick = 0;
    dda->seg_flags = seg->flags;
    dda->active = true;
    return true;
}

uint32_t stepgen_dda_tick(stepgen_dda_t *dda)
{
    if (stepgen_dda_segment_done(dda)) {
        return 0;
    }
    uint32_t mask = 0;
    const uint32_t T = dda->ticks_per_segment;
    for (uint32_t i = 0; i < dda->num_axes; i++) {
        dda->acc[i] += dda->n[i];
        if (dda->acc[i] >= T) {
            dda->acc[i] -= T;
            mask |= 1u << i;
            dda->pos[i] += (dda->dir_mask & (1u << i)) ? 1 : -1;
        }
    }
    dda->tick++;
    return mask;
}

void stepgen_dda_reset(stepgen_dda_t *dda)
{
    dda->active = false;
    dda->tick = 0;
    dda->seg_flags = 0;
    memset(dda->n, 0, sizeof(dda->n));
}
