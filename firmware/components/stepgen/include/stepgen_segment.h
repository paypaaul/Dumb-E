#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define STEPGEN_MAX_AXES 6

/* Set on the final segment of a motion: the queue running empty afterwards is not an underrun. */
#define STEPGEN_SEG_FLAG_LAST 0x01u

/* Signed number of steps each axis must perform during one segment period. */
typedef struct {
    int16_t steps[STEPGEN_MAX_AXES];
    uint8_t flags;
} stepgen_segment_t;

#ifdef __cplusplus
}
#endif
