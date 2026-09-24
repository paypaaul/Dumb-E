/*
 * Step pulse generator.
 *
 * A GPTimer interrupt (CONFIG_DUMBE_STEPGEN_TICK_HZ) consumes segments from a lock-free queue and spreads the
 * steps of each segment evenly over CONFIG_DUMBE_STEPGEN_TICKS_PER_SEGMENT ticks with an integer DDA.
 * The ISR is integer-only, lives in IRAM and keeps running while the flash cache is disabled.
 *
 * Threading: stepgen_init() must be called from the task that produces segments, pinned to the core that
 * should service the interrupt (the interrupt is allocated on the calling core). stepgen_push() is
 * single-producer. stepgen_abort() may be called from any task or ISR.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "stepgen_segment.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int step_gpio; /* must be < 32 (all STEP pins are driven with a single register write) */
    int dir_gpio;  /* high = positive direction */
} stepgen_axis_config_t;

typedef struct {
    const stepgen_axis_config_t *axes;
    uint8_t num_axes; /* 1..STEPGEN_MAX_AXES */
} stepgen_config_t;

typedef struct {
    uint32_t ticks;          /* ISR executions (wraps) */
    uint32_t isr_cycles_avg; /* moving average of ISR duration [CPU cycles] */
    uint32_t isr_cycles_max;
    uint32_t segments;       /* segments executed */
    uint32_t underruns;      /* queue ran empty in the middle of a motion */
    uint32_t rejected;       /* segments with too many steps (planner bug) */
    uint32_t aborts;
    uint32_t stalls;         /* times the tick interrupt stopped and was re-armed by stepgen_watchdog() */
} stepgen_stats_t;

esp_err_t stepgen_init(const stepgen_config_t *config, TaskHandle_t consumer_task);

uint32_t stepgen_tick_hz(void);
uint32_t stepgen_ticks_per_segment(void);
/* Maximum step rate of one axis [steps/s] (one step per tick). */
uint32_t stepgen_max_step_rate(void);
/* Duration of one segment [s]. */
float stepgen_segment_period(void);

/* Queues a segment. Returns false if the queue is full. The consumer task is notified after each segment. */
bool stepgen_push(const stepgen_segment_t *seg);
uint32_t stepgen_queue_used(void);
uint32_t stepgen_queue_free(void);

/* True when nothing is queued and no segment is being executed. */
bool stepgen_is_idle(void);

/* Stops pulse generation immediately and drops everything queued. Safe from any context. */
void stepgen_abort(void);
/* Incremented each time an abort has been executed by the ISR. */
uint32_t stepgen_abort_count(void);

/* Current step positions (each axis read atomically). */
void stepgen_get_position(int32_t pos[STEPGEN_MAX_AXES]);
/* Overwrites the step positions. Only allowed while idle. */
esp_err_t stepgen_set_position(const int32_t pos[STEPGEN_MAX_AXES]);

void stepgen_get_stats(stepgen_stats_t *stats);

/*
 * Checks that the tick interrupt is still running and re-arms the timer if it stopped (an alarm is lost if
 * the ISR is delayed by more than one tick period). Call periodically (every few ms) from the consumer task.
 */
void stepgen_watchdog(void);

#ifdef __cplusplus
}
#endif
