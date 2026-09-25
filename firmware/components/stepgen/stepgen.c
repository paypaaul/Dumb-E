#include "stepgen.h"

#include <string.h>

#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "esp_attr.h"
#include "esp_check.h"
#include "esp_cpu.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "hal/gpio_ll.h"
#include "segment_ring.h"
#include "sdkconfig.h"
#include "stepgen_dda.h"

static const char *TAG = "stepgen";

#define TIMER_RESOLUTION_HZ 10000000u
#define TICK_HZ             ((uint32_t)CONFIG_DUMBE_STEPGEN_TICK_HZ)
#define TICKS_PER_SEGMENT   ((uint32_t)CONFIG_DUMBE_STEPGEN_TICKS_PER_SEGMENT)

_Static_assert(TIMER_RESOLUTION_HZ % CONFIG_DUMBE_STEPGEN_TICK_HZ == 0, "tick must divide 10 MHz");

typedef struct {
    stepgen_dda_t dda;
    segment_ring_t ring;
    uint32_t num_axes;
    /* STEP pins are written through two registers: GPIO0-31 (out) and GPIO32-39 (out1). */
    uint32_t step_bit_lo[STEPGEN_MAX_AXES];
    uint32_t step_bit_hi[STEPGEN_MAX_AXES];
    uint32_t dir_gpio[STEPGEN_MAX_AXES];
    uint32_t all_step_lo;
    uint32_t all_step_hi;
    uint32_t pending_steps; /* axis mask raised at the start of the next tick */
    uint32_t min_pulse_cycles;
    TaskHandle_t consumer;
    volatile uint32_t abort_request;
    volatile uint32_t abort_count;
    volatile uint32_t busy; /* a segment is loaded or pulses are pending */
    stepgen_stats_t stats;
} stepgen_state_t;

static DRAM_ATTR stepgen_state_t s_sg;
static gptimer_handle_t s_timer;
static bool s_initialized;
static uint32_t s_wd_last_ticks;
static int64_t s_wd_last_change_us;

/* The tick is considered stalled after this long without any interrupt. */
#define WATCHDOG_TIMEOUT_US 5000

#if CONFIG_DUMBE_STEPGEN_DEBUG_GPIO >= 0
#define DEBUG_PIN_SET(level) gpio_ll_set_level(&GPIO, CONFIG_DUMBE_STEPGEN_DEBUG_GPIO, (level))
#else
#define DEBUG_PIN_SET(level) ((void)0)
#endif

static inline void IRAM_ATTR raise_steps(uint32_t axis_mask)
{
    uint32_t lo = 0, hi = 0;
    for (uint32_t i = 0; i < s_sg.num_axes; i++) {
        if (axis_mask & (1u << i)) {
            lo |= s_sg.step_bit_lo[i];
            hi |= s_sg.step_bit_hi[i];
        }
    }
    if (lo) {
        GPIO.out_w1ts = lo;
    }
    if (hi) {
        GPIO.out1_w1ts.val = hi;
    }
}

static inline void IRAM_ATTR lower_steps(void)
{
    if (s_sg.all_step_lo) {
        GPIO.out_w1tc = s_sg.all_step_lo;
    }
    if (s_sg.all_step_hi) {
        GPIO.out1_w1tc.val = s_sg.all_step_hi;
    }
}

static bool IRAM_ATTR on_tick(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)
{
    (void)user_ctx;
    const uint32_t start = esp_cpu_get_cycle_count();
    (void)timer;
    (void)edata;
    DEBUG_PIN_SET(1);

    /* 1. Rising edges for the steps computed during the previous tick. */
    const uint32_t raised = s_sg.pending_steps;
    if (raised) {
        raise_steps(raised);
    }

    /* 2. Compute the next tick (and load the next segment at a boundary). */
    bool notify = false;
    uint32_t dir_changed = 0;
    if (s_sg.abort_request) {
        s_sg.abort_request = 0;
        segment_ring_flush(&s_sg.ring);
        stepgen_dda_reset(&s_sg.dda);
        s_sg.abort_count++;
        s_sg.stats.aborts++;
        notify = true;
    } else if (stepgen_dda_segment_done(&s_sg.dda)) {
        const bool was_active = s_sg.dda.active;
        const bool was_last = (s_sg.dda.seg_flags & STEPGEN_SEG_FLAG_LAST) != 0;
        if (was_active) {
            s_sg.stats.segments++;
            notify = true;
        }
        stepgen_segment_t seg;
        if (segment_ring_pop(&s_sg.ring, &seg)) {
            const uint32_t old_dir = s_sg.dda.dir_mask;
            if (stepgen_dda_load(&s_sg.dda, &seg)) {
                dir_changed = old_dir ^ s_sg.dda.dir_mask;
            } else {
                s_sg.stats.rejected++;
                stepgen_dda_reset(&s_sg.dda);
            }
        } else {
            if (was_active && !was_last) {
                s_sg.stats.underruns++;
            }
            stepgen_dda_reset(&s_sg.dda);
        }
    }
    /* Steps computed now are raised at the start of the next tick, after the DIR writes below. */
    const uint32_t next = stepgen_dda_tick(&s_sg.dda);

    /* 3. Falling edges after the minimum pulse width. */
    if (raised) {
        while ((uint32_t)(esp_cpu_get_cycle_count() - start) < s_sg.min_pulse_cycles) {
        }
        lower_steps();
    }

    /* 4. Direction pins, written at least one tick before the next rising edge. */
    if (dir_changed) {
        for (uint32_t i = 0; i < s_sg.num_axes; i++) {
            if (dir_changed & (1u << i)) {
                gpio_ll_set_level(&GPIO, s_sg.dir_gpio[i], (s_sg.dda.dir_mask >> i) & 1u);
            }
        }
    }

    s_sg.pending_steps = next;
    s_sg.busy = s_sg.dda.active || next != 0;

    BaseType_t woken = pdFALSE;
    if (notify && s_sg.consumer != NULL) {
        vTaskNotifyGiveFromISR(s_sg.consumer, &woken);
    }

    s_sg.stats.ticks++;
    const uint32_t cycles = esp_cpu_get_cycle_count() - start;
    if (cycles > s_sg.stats.isr_cycles_max) {
        s_sg.stats.isr_cycles_max = cycles;
    }
    s_sg.stats.isr_cycles_avg += (int32_t)(cycles - s_sg.stats.isr_cycles_avg) / 64;
    DEBUG_PIN_SET(0);
    return woken == pdTRUE;
}

esp_err_t stepgen_init(const stepgen_config_t *config, TaskHandle_t consumer_task)
{
    ESP_RETURN_ON_FALSE(!s_initialized, ESP_ERR_INVALID_STATE, TAG, "already initialized");
    ESP_RETURN_ON_FALSE(config != NULL && config->axes != NULL && config->num_axes > 0 &&
                            config->num_axes <= STEPGEN_MAX_AXES,
                        ESP_ERR_INVALID_ARG, TAG, "invalid axis configuration");

    memset(&s_sg, 0, sizeof(s_sg));
    segment_ring_init(&s_sg.ring);
    stepgen_dda_init(&s_sg.dda, config->num_axes, TICKS_PER_SEGMENT);
    s_sg.num_axes = config->num_axes;
    s_sg.consumer = consumer_task;
    s_sg.min_pulse_cycles = (uint32_t)CONFIG_DUMBE_STEPGEN_MIN_PULSE_NS * esp_rom_get_cpu_ticks_per_us() / 1000u;

    uint64_t out_mask = 0;
    for (uint32_t i = 0; i < config->num_axes; i++) {
        const int step = config->axes[i].step_gpio;
        const int dir = config->axes[i].dir_gpio;
        ESP_RETURN_ON_FALSE(GPIO_IS_VALID_OUTPUT_GPIO(step), ESP_ERR_INVALID_ARG, TAG,
                            "axis %" PRIu32 ": STEP gpio %d is not an output", i, step);
        ESP_RETURN_ON_FALSE(GPIO_IS_VALID_OUTPUT_GPIO(dir), ESP_ERR_INVALID_ARG, TAG,
                            "axis %" PRIu32 ": DIR gpio %d is not an output", i, dir);
        if (step < 32) {
            s_sg.step_bit_lo[i] = 1u << step;
        } else {
            s_sg.step_bit_hi[i] = 1u << (step - 32);
        }
        s_sg.all_step_lo |= s_sg.step_bit_lo[i];
        s_sg.all_step_hi |= s_sg.step_bit_hi[i];
        s_sg.dir_gpio[i] = (uint32_t)dir;
        out_mask |= (1ull << step) | (1ull << dir);
    }
#if CONFIG_DUMBE_STEPGEN_DEBUG_GPIO >= 0
    out_mask |= 1ull << CONFIG_DUMBE_STEPGEN_DEBUG_GPIO;
#endif
    const gpio_config_t io = {
        .pin_bit_mask = out_mask,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&io), TAG, "gpio_config");
    for (uint32_t i = 0; i < config->num_axes; i++) {
        gpio_set_level(config->axes[i].step_gpio, 0);
        /* Matches the DDA's initial direction (positive). */
        gpio_set_level(config->axes[i].dir_gpio, 1);
    }

    const gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = TIMER_RESOLUTION_HZ,
        .intr_priority = CONFIG_DUMBE_STEPGEN_INTR_PRIORITY,
    };
    ESP_RETURN_ON_ERROR(gptimer_new_timer(&timer_config, &s_timer), TAG, "gptimer_new_timer");
    const gptimer_event_callbacks_t cbs = {.on_alarm = on_tick};
    /* The interrupt is allocated here, on the calling core. */
    ESP_RETURN_ON_ERROR(gptimer_register_event_callbacks(s_timer, &cbs, NULL), TAG, "register callbacks");
    const gptimer_alarm_config_t alarm = {
        .alarm_count = TIMER_RESOLUTION_HZ / TICK_HZ,
        .reload_count = 0,
        .flags.auto_reload_on_alarm = true,
    };
    ESP_RETURN_ON_ERROR(gptimer_set_alarm_action(s_timer, &alarm), TAG, "set alarm");
    ESP_RETURN_ON_ERROR(gptimer_enable(s_timer), TAG, "enable");
    ESP_RETURN_ON_ERROR(gptimer_start(s_timer), TAG, "start");

    s_initialized = true;
    ESP_LOGI(TAG, "%" PRIu32 " axes, tick %" PRIu32 " Hz, %" PRIu32 " ticks/segment, core %d", s_sg.num_axes,
             TICK_HZ, TICKS_PER_SEGMENT, esp_cpu_get_core_id());
    return ESP_OK;
}

uint32_t stepgen_tick_hz(void)
{
    return TICK_HZ;
}

uint32_t stepgen_ticks_per_segment(void)
{
    return TICKS_PER_SEGMENT;
}

uint32_t stepgen_max_step_rate(void)
{
    return TICK_HZ;
}

float stepgen_segment_period(void)
{
    return (float)TICKS_PER_SEGMENT / (float)TICK_HZ;
}

bool stepgen_push(const stepgen_segment_t *seg)
{
    return segment_ring_push(&s_sg.ring, seg);
}

uint32_t stepgen_queue_used(void)
{
    return segment_ring_used(&s_sg.ring);
}

uint32_t stepgen_queue_free(void)
{
    return segment_ring_free(&s_sg.ring);
}

bool stepgen_is_idle(void)
{
    return segment_ring_used(&s_sg.ring) == 0 && !s_sg.busy;
}

void IRAM_ATTR stepgen_abort(void)
{
    s_sg.abort_request = 1;
}

uint32_t stepgen_abort_count(void)
{
    return s_sg.abort_count;
}

void stepgen_get_position(int32_t pos[STEPGEN_MAX_AXES])
{
    for (uint32_t i = 0; i < STEPGEN_MAX_AXES; i++) {
        pos[i] = s_sg.dda.pos[i];
    }
}

esp_err_t stepgen_set_position(const int32_t pos[STEPGEN_MAX_AXES])
{
    ESP_RETURN_ON_FALSE(stepgen_is_idle(), ESP_ERR_INVALID_STATE, TAG, "cannot set position while moving");
    for (uint32_t i = 0; i < STEPGEN_MAX_AXES; i++) {
        s_sg.dda.pos[i] = pos[i];
    }
    return ESP_OK;
}

void stepgen_get_stats(stepgen_stats_t *stats)
{
    *stats = s_sg.stats;
}

void stepgen_watchdog(void)
{
    if (!s_initialized) {
        return;
    }
    const int64_t now = esp_timer_get_time();
    const uint32_t ticks = s_sg.stats.ticks;
    if (ticks != s_wd_last_ticks) {
        s_wd_last_ticks = ticks;
        s_wd_last_change_us = now;
        return;
    }
    if (now - s_wd_last_change_us < WATCHDOG_TIMEOUT_US) {
        return;
    }
    /* Restart counting from zero and re-enable the alarm. */
    const gptimer_alarm_config_t alarm = {
        .alarm_count = TIMER_RESOLUTION_HZ / TICK_HZ,
        .reload_count = 0,
        .flags.auto_reload_on_alarm = true,
    };
    gptimer_set_raw_count(s_timer, 0);
    gptimer_set_alarm_action(s_timer, &alarm);
    s_sg.stats.stalls++;
    s_wd_last_change_us = now;
    ESP_LOGW(TAG, "tick interrupt stalled, timer re-armed");
}
