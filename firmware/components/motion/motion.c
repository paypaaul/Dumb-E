#include "motion.h"

#include <math.h>
#include <stdatomic.h>
#include <string.h>

#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "motion_profile.h"

static const char *TAG = "motion";

#define MOTION_TASK_CORE      1
#define MOTION_TASK_PRIORITY  (configMAX_PRIORITIES - 2)
#define MOTION_TASK_STACK     4096
#define MOTION_QUEUE_LEN      8
/* Segments kept queued ahead of the ISR (~20 ms at 1 kHz): latency of `stop` vs. tolerance to stalls. */
#define MOTION_FILL_TARGET    20
/* Joint moves smaller than this are ignored [rad]. */
#define MOTION_DQ_EPS         1e-6f
/* Headroom kept below the maximum step rate of the step generator. */
#define MOTION_STEP_RATE_USAGE 0.9f

typedef struct {
    float q_target[MOTION_MAX_JOINTS];
    float speed_scale;
    float accel_scale;
} motion_cmd_t;

typedef struct {
    motion_config_t cfg;
    QueueHandle_t queue;
    TaskHandle_t task;
    SemaphoreHandle_t ready;
    esp_err_t init_result;
    portMUX_TYPE lock;

    /* Owned by the motion task. */
    bool active;
    bool stopping;
    mp_profile_t profile;
    float t;
    float sdd_max;
    float seg_dt;
    float q_start[MOTION_MAX_JOINTS];
    float q_target[MOTION_MAX_JOINTS];
    float dq[MOTION_MAX_JOINTS];
    int32_t last_steps[MOTION_MAX_JOINTS];
    uint32_t seen_aborts;

    /* Shared, protected by `lock`. */
    float q_cmd[MOTION_MAX_JOINTS];

    /* Commands accepted and not yet fully planned (queued + the one being executed). */
    atomic_int pending;

    volatile bool stop_request;
    volatile bool abort_request;
} motion_state_t;

static motion_state_t s_m = {.lock = portMUX_INITIALIZER_UNLOCKED};

static void publish_q_cmd(const float q[MOTION_MAX_JOINTS])
{
    portENTER_CRITICAL(&s_m.lock);
    memcpy(s_m.q_cmd, q, sizeof(s_m.q_cmd));
    portEXIT_CRITICAL(&s_m.lock);
}

/* Re-reads the step generator positions after an abort or a reference change. */
static void resync_from_stepgen(void)
{
    int32_t pos[STEPGEN_MAX_AXES];
    stepgen_get_position(pos);
    float q[MOTION_MAX_JOINTS] = {0};
    for (int i = 0; i < s_m.cfg.num_joints; i++) {
        s_m.last_steps[i] = pos[i];
        q[i] = kin_steps_to_joint(&s_m.cfg.drive[i], pos[i]);
    }
    if (s_m.active) {
        atomic_fetch_sub(&s_m.pending, 1);
    }
    s_m.active = false;
    s_m.stopping = false;
    publish_q_cmd(q);
}

/* Drops all queued commands. */
static void drain_queue(void)
{
    motion_cmd_t cmd;
    while (xQueueReceive(s_m.queue, &cmd, 0) == pdTRUE) {
        atomic_fetch_sub(&s_m.pending, 1);
    }
}

static void wait_stepgen_idle(void)
{
    while (!stepgen_is_idle()) {
        vTaskDelay(1);
    }
}

static void start_move(const motion_cmd_t *cmd)
{
    float v[MOTION_MAX_JOINTS], a[MOTION_MAX_JOINTS];
    for (int i = 0; i < s_m.cfg.num_joints; i++) {
        s_m.q_start[i] = s_m.q_cmd[i];
        s_m.q_target[i] = cmd->q_target[i];
        s_m.dq[i] = cmd->q_target[i] - s_m.q_start[i];
        v[i] = s_m.cfg.v_max[i] * cmd->speed_scale;
        a[i] = s_m.cfg.a_max[i] * cmd->accel_scale;
    }
    float sd_max;
    if (!mp_sync_limits(s_m.dq, v, a, s_m.cfg.num_joints, MOTION_DQ_EPS, &sd_max, &s_m.sdd_max)) {
        atomic_fetch_sub(&s_m.pending, 1); /* already there */
        return;
    }
    if (!mp_profile_trapezoid(&s_m.profile, 1.0f, sd_max, s_m.sdd_max)) {
        ESP_LOGE(TAG, "invalid profile");
        atomic_fetch_sub(&s_m.pending, 1);
        return;
    }
    s_m.t = 0.0f;
    s_m.active = true;
    s_m.stopping = false;
    ESP_LOGD(TAG, "move: %.3f s", (double)s_m.profile.total_time);
}

/* Produces segments until the step generator queue holds MOTION_FILL_TARGET of them. */
static void fill_segments(void)
{
    float q[MOTION_MAX_JOINTS] = {0};
    while (s_m.active && stepgen_queue_used() < MOTION_FILL_TARGET) {
        s_m.t += s_m.seg_dt;
        float s;
        mp_profile_eval(&s_m.profile, s_m.t, &s, NULL);
        const bool done = mp_profile_done(&s_m.profile, s_m.t);
        const bool at_target = done && !s_m.stopping;

        stepgen_segment_t seg = {.flags = done ? STEPGEN_SEG_FLAG_LAST : 0};
        const int32_t limit = (int32_t)stepgen_ticks_per_segment();
        for (int i = 0; i < s_m.cfg.num_joints; i++) {
            q[i] = at_target ? s_m.q_target[i] : s_m.q_start[i] + s_m.dq[i] * s;
            const int32_t steps = kin_joint_to_steps(&s_m.cfg.drive[i], q[i]);
            int32_t delta = steps - s_m.last_steps[i];
            if (delta > limit || delta < -limit) {
                /* Cannot happen with validated limits; clamp rather than lose the step count. */
                ESP_LOGE(TAG, "joint %d: %" PRId32 " steps in one segment", i, delta);
                delta = delta > 0 ? limit : -limit;
            }
            seg.steps[i] = (int16_t)delta;
            s_m.last_steps[i] += delta;
        }
        if (!stepgen_push(&seg)) {
            ESP_LOGE(TAG, "segment queue full");
            break;
        }
        publish_q_cmd(q);
        if (done) {
            s_m.active = false;
            s_m.stopping = false;
            atomic_fetch_sub(&s_m.pending, 1);
        }
    }
}

static void motion_task(void *arg)
{
    (void)arg;
    s_m.init_result = stepgen_init(&s_m.cfg.stepgen, xTaskGetCurrentTaskHandle());
    xSemaphoreGive(s_m.ready);
    if (s_m.init_result != ESP_OK) {
        vTaskDelete(NULL);
        return;
    }
    s_m.seg_dt = stepgen_segment_period();
    s_m.seen_aborts = stepgen_abort_count();
    resync_from_stepgen();

    for (;;) {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(5));
        stepgen_watchdog();

        if (s_m.abort_request) {
            s_m.abort_request = false;
            stepgen_abort();
        }
        if (stepgen_abort_count() != s_m.seen_aborts) {
            /* Abort requested here or directly (e.g. e-stop ISR): drop everything and resync. */
            drain_queue();
            wait_stepgen_idle();
            s_m.seen_aborts = stepgen_abort_count();
            s_m.stop_request = false;
            resync_from_stepgen();
            continue;
        }
        if (s_m.stop_request) {
            s_m.stop_request = false;
            drain_queue();
            if (s_m.active && !s_m.stopping) {
                mp_profile_stop(&s_m.profile, s_m.t, s_m.sdd_max);
                s_m.t = 0.0f;
                s_m.stopping = true;
            }
        }
        if (!s_m.active) {
            motion_cmd_t cmd;
            if (xQueueReceive(s_m.queue, &cmd, 0) == pdTRUE) {
                start_move(&cmd);
            }
        }
        if (s_m.active) {
            fill_segments();
        }
    }
}

esp_err_t motion_init(const motion_config_t *config)
{
    ESP_RETURN_ON_FALSE(s_m.task == NULL, ESP_ERR_INVALID_STATE, TAG, "already initialized");
    ESP_RETURN_ON_FALSE(config != NULL && config->num_joints > 0 && config->num_joints <= MOTION_MAX_JOINTS &&
                            config->stepgen.num_axes == config->num_joints,
                        ESP_ERR_INVALID_ARG, TAG, "invalid configuration");
    s_m.cfg = *config;

    const float max_rate = MOTION_STEP_RATE_USAGE * (float)stepgen_max_step_rate();
    for (int i = 0; i < config->num_joints; i++) {
        ESP_RETURN_ON_FALSE(config->drive[i].steps_per_rad > 0.0f && config->v_max[i] > 0.0f &&
                                config->a_max[i] > 0.0f,
                            ESP_ERR_INVALID_ARG, TAG, "joint %d: invalid drive or limits", i);
        const float v_limit = max_rate / config->drive[i].steps_per_rad;
        if (s_m.cfg.v_max[i] > v_limit) {
            ESP_LOGW(TAG, "joint %d: max speed %.1f deg/s clamped to %.1f deg/s (step rate limit)", i,
                     (double)(s_m.cfg.v_max[i] * 57.29578f), (double)(v_limit * 57.29578f));
            s_m.cfg.v_max[i] = v_limit;
        }
    }

    s_m.queue = xQueueCreate(MOTION_QUEUE_LEN, sizeof(motion_cmd_t));
    s_m.ready = xSemaphoreCreateBinary();
    ESP_RETURN_ON_FALSE(s_m.queue != NULL && s_m.ready != NULL, ESP_ERR_NO_MEM, TAG, "no memory");

    BaseType_t ok = xTaskCreatePinnedToCore(motion_task, "motion", MOTION_TASK_STACK, NULL, MOTION_TASK_PRIORITY,
                                            &s_m.task, MOTION_TASK_CORE);
    ESP_RETURN_ON_FALSE(ok == pdPASS, ESP_ERR_NO_MEM, TAG, "task creation failed");
    xSemaphoreTake(s_m.ready, portMAX_DELAY);
    return s_m.init_result;
}

esp_err_t motion_move_joints(const float q_target[MOTION_MAX_JOINTS], float speed_scale, float accel_scale)
{
    ESP_RETURN_ON_FALSE(s_m.queue != NULL, ESP_ERR_INVALID_STATE, TAG, "not initialized");
    ESP_RETURN_ON_FALSE(speed_scale > 0.0f && speed_scale <= 1.0f && accel_scale > 0.0f && accel_scale <= 1.0f,
                        ESP_ERR_INVALID_ARG, TAG, "scale out of range");
    motion_cmd_t cmd = {.speed_scale = speed_scale, .accel_scale = accel_scale};
    for (int i = 0; i < s_m.cfg.num_joints; i++) {
        ESP_RETURN_ON_FALSE(isfinite(q_target[i]), ESP_ERR_INVALID_ARG, TAG, "invalid target");
        cmd.q_target[i] = q_target[i];
    }
    atomic_fetch_add(&s_m.pending, 1);
    if (xQueueSend(s_m.queue, &cmd, 0) != pdTRUE) {
        atomic_fetch_sub(&s_m.pending, 1);
        return ESP_ERR_NO_MEM;
    }
    xTaskNotifyGive(s_m.task);
    return ESP_OK;
}

esp_err_t motion_stop(void)
{
    ESP_RETURN_ON_FALSE(s_m.task != NULL, ESP_ERR_INVALID_STATE, TAG, "not initialized");
    s_m.stop_request = true;
    xTaskNotifyGive(s_m.task);
    return ESP_OK;
}

esp_err_t motion_abort(void)
{
    ESP_RETURN_ON_FALSE(s_m.task != NULL, ESP_ERR_INVALID_STATE, TAG, "not initialized");
    stepgen_abort();
    s_m.abort_request = true;
    xTaskNotifyGive(s_m.task);
    return ESP_OK;
}

bool motion_is_busy(void)
{
    if (s_m.queue == NULL) {
        return false;
    }
    return atomic_load(&s_m.pending) > 0 || !stepgen_is_idle() || s_m.stop_request || s_m.abort_request ||
           stepgen_abort_count() != s_m.seen_aborts;
}

esp_err_t motion_set_joint_positions(const float q[MOTION_MAX_JOINTS])
{
    ESP_RETURN_ON_FALSE(s_m.task != NULL, ESP_ERR_INVALID_STATE, TAG, "not initialized");
    ESP_RETURN_ON_FALSE(!motion_is_busy(), ESP_ERR_INVALID_STATE, TAG, "busy");
    int32_t pos[STEPGEN_MAX_AXES] = {0};
    float qc[MOTION_MAX_JOINTS] = {0};
    for (int i = 0; i < s_m.cfg.num_joints; i++) {
        pos[i] = kin_joint_to_steps(&s_m.cfg.drive[i], q[i]);
        qc[i] = q[i];
    }
    ESP_RETURN_ON_ERROR(stepgen_set_position(pos), TAG, "set position");
    /* The motion task is idle: updating its step bookkeeping here cannot race with fill_segments(). */
    for (int i = 0; i < s_m.cfg.num_joints; i++) {
        s_m.last_steps[i] = pos[i];
    }
    publish_q_cmd(qc);
    return ESP_OK;
}

void motion_get_status(motion_status_t *status)
{
    memset(status, 0, sizeof(*status));
    portENTER_CRITICAL(&s_m.lock);
    memcpy(status->q_cmd, s_m.q_cmd, sizeof(status->q_cmd));
    portEXIT_CRITICAL(&s_m.lock);
    status->busy = motion_is_busy();
    status->queued = s_m.queue ? uxQueueMessagesWaiting(s_m.queue) : 0;
    stepgen_get_position(status->steps);
    for (int i = 0; i < s_m.cfg.num_joints; i++) {
        status->q_actual[i] = kin_steps_to_joint(&s_m.cfg.drive[i], status->steps[i]);
    }
    stepgen_get_stats(&status->stepgen);
}
