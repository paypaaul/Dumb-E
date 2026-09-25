#include "robot.h"

#include <math.h>
#include <string.h>

#include "board.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "gripper.h"
#include "motion.h"
#include "sdkconfig.h"

static const char *TAG = "robot";

#define DEG_TO_RAD(x) ((x) * 0.017453292519943295f)
#define RAD_TO_DEG(x) ((x) * 57.29577951308232f)
#define LED_PERIOD_MS 100

static struct {
    const robot_config_t *cfg;
    kin_model_t model;
    kin_drive_t drive[KIN_NUM_JOINTS];
    SemaphoreHandle_t mutex;
    esp_timer_handle_t led_timer;
    uint32_t led_tick;
    bool enabled;
    bool referenced;
    volatile bool estop_latched;
} s_r;

/* ---------- e-stop ---------- */

static bool estop_input_active(void)
{
#if CONFIG_DUMBE_ESTOP_ENABLE
    const int level = gpio_get_level(CONFIG_DUMBE_ESTOP_GPIO);
#if CONFIG_DUMBE_ESTOP_ACTIVE_HIGH
    return level == 1;
#else
    return level == 0;
#endif
#else
    return false;
#endif
}

#if CONFIG_DUMBE_ESTOP_ENABLE
static void estop_isr(void *arg)
{
    (void)arg;
#if CONFIG_DUMBE_ESTOP_ACTIVE_HIGH
    const bool active = gpio_get_level(CONFIG_DUMBE_ESTOP_GPIO) == 1;
#else
    const bool active = gpio_get_level(CONFIG_DUMBE_ESTOP_GPIO) == 0;
#endif
    if (active) {
        stepgen_abort();
        s_r.estop_latched = true;
    }
}

static esp_err_t estop_init(void)
{
    const gpio_config_t io = {
        .pin_bit_mask = 1ull << CONFIG_DUMBE_ESTOP_GPIO,
        .mode = GPIO_MODE_INPUT,
        /* GPIO34-39 have no internal pull-up: use an external one on those pins. */
        .pull_up_en = GPIO_IS_VALID_OUTPUT_GPIO(CONFIG_DUMBE_ESTOP_GPIO) ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    esp_err_t err = gpio_config(&io);
    if (err == ESP_OK) {
        err = gpio_install_isr_service(0);
        if (err == ESP_ERR_INVALID_STATE) {
            err = ESP_OK; /* already installed */
        }
    }
    if (err == ESP_OK) {
        err = gpio_isr_handler_add(CONFIG_DUMBE_ESTOP_GPIO, estop_isr, NULL);
    }
    if (err == ESP_OK && estop_input_active()) {
        s_r.estop_latched = true;
    }
    return err;
}
#endif

/* ---------- state ---------- */

static robot_state_t current_state(void)
{
    if (s_r.estop_latched) {
        return ROBOT_ESTOP;
    }
    if (!s_r.enabled) {
        return ROBOT_DISABLED;
    }
    return s_r.referenced ? ROBOT_READY : ROBOT_ENABLED;
}

static void led_cb(void *arg)
{
    (void)arg;
    const uint32_t t = s_r.led_tick++;
    bool on;
    switch (current_state()) {
    case ROBOT_ESTOP:
        on = (t % 2) == 0; /* fast blink */
        break;
    case ROBOT_DISABLED:
        on = (t % 20) == 0; /* short flash every 2 s */
        break;
    case ROBOT_ENABLED:
        on = (t % 10) < 5; /* slow blink */
        break;
    default:
        on = motion_is_busy() ? (t % 4) < 2 : true; /* READY: solid, moving: medium blink */
        break;
    }
    board_led_set(on);
}

static void lock(void)
{
    xSemaphoreTake(s_r.mutex, portMAX_DELAY);
}

static void unlock(void)
{
    xSemaphoreGive(s_r.mutex);
}

static float clamp_pct(float pct)
{
    return fminf(100.0f, fmaxf(0.0f, pct)) / 100.0f;
}

static robot_err_t check_can_move(bool need_reference)
{
    if (s_r.estop_latched) {
        return ROBOT_ERR_ESTOP;
    }
    if (!s_r.enabled) {
        return ROBOT_ERR_STATE;
    }
    if (need_reference && !s_r.referenced) {
        return ROBOT_ERR_NOT_REFERENCED;
    }
    return ROBOT_OK;
}

static robot_err_t queue_move(const float q_rad[KIN_NUM_JOINTS], float speed, float accel)
{
    float q[MOTION_MAX_JOINTS] = {0};
    memcpy(q, q_rad, sizeof(float) * KIN_NUM_JOINTS);
    const esp_err_t err = motion_move_joints(q, speed, accel);
    if (err == ESP_ERR_NO_MEM) {
        return ROBOT_ERR_QUEUE_FULL;
    }
    return err == ESP_OK ? ROBOT_OK : ROBOT_ERR_INTERNAL;
}

/* ---------- public API ---------- */

robot_err_t robot_init(void)
{
    s_r.cfg = &robot_default_config;
    const robot_config_t *c = s_r.cfg;
    s_r.mutex = xSemaphoreCreateMutex();
    if (s_r.mutex == NULL) {
        return ROBOT_ERR_INTERNAL;
    }
    if (board_init() != ESP_OK) {
        return ROBOT_ERR_INTERNAL;
    }

    s_r.model = (kin_model_t){
        .d1 = c->d1_mm,
        .a2 = c->a2_mm,
        .a3 = c->a3_mm,
        .tool_offset = c->tool_offset_mm,
        .tool_length = c->tool_length_mm,
    };
    motion_config_t mc = {.num_joints = KIN_NUM_JOINTS};
    static stepgen_axis_config_t axes[KIN_NUM_JOINTS];
    for (int i = 0; i < KIN_NUM_JOINTS; i++) {
        const robot_joint_config_t *j = &c->joints[i];
        s_r.model.q_min[i] = DEG_TO_RAD(j->min_deg);
        s_r.model.q_max[i] = DEG_TO_RAD(j->max_deg);
        s_r.drive[i] = (kin_drive_t){
            .steps_per_rad = kin_steps_per_rad(j->full_steps_per_rev, j->microsteps, j->gear_ratio),
            .invert = j->invert,
        };
        mc.drive[i] = s_r.drive[i];
        mc.v_max[i] = DEG_TO_RAD(j->v_max_deg_s);
        mc.a_max[i] = DEG_TO_RAD(j->a_max_deg_s2);
        axes[i] = (stepgen_axis_config_t){.step_gpio = board_axis_pins[i].step, .dir_gpio = board_axis_pins[i].dir};
    }
    if (!kin_model_is_valid(&s_r.model)) {
        ESP_LOGE(TAG, "invalid robot configuration");
        return ROBOT_ERR_INTERNAL;
    }
    mc.stepgen = (stepgen_config_t){.axes = axes, .num_axes = KIN_NUM_JOINTS};
    if (motion_init(&mc) != ESP_OK) {
        return ROBOT_ERR_INTERNAL;
    }

    const gripper_config_t gc = {
        .gpio = BOARD_PIN_SERVO,
        .pulse_min_us = c->gripper_pulse_min_us,
        .pulse_max_us = c->gripper_pulse_max_us,
        .speed_pct_per_s = c->gripper_speed_pct_s,
    };
    if (gripper_init(&gc) != ESP_OK) {
        return ROBOT_ERR_INTERNAL;
    }

#if CONFIG_DUMBE_ESTOP_ENABLE
    if (estop_init() != ESP_OK) {
        return ROBOT_ERR_INTERNAL;
    }
#endif

    const esp_timer_create_args_t led_args = {.callback = led_cb, .name = "status_led"};
    if (esp_timer_create(&led_args, &s_r.led_timer) != ESP_OK ||
        esp_timer_start_periodic(s_r.led_timer, LED_PERIOD_MS * 1000) != ESP_OK) {
        return ROBOT_ERR_INTERNAL;
    }
    ESP_LOGI(TAG, "ready: drivers disabled, not referenced");
    return ROBOT_OK;
}

robot_err_t robot_enable(void)
{
    lock();
    robot_err_t err = ROBOT_OK;
    if (s_r.estop_latched) {
        err = ROBOT_ERR_ESTOP;
    } else if (!s_r.enabled) {
        board_drivers_enable(true);
        s_r.enabled = true;
        s_r.referenced = false;
    }
    unlock();
    return err;
}

robot_err_t robot_disable(void)
{
    lock();
    robot_err_t err = ROBOT_OK;
    if (motion_is_busy()) {
        err = ROBOT_ERR_BUSY;
    } else {
        board_drivers_enable(false);
        s_r.enabled = false;
        s_r.referenced = false; /* the arm can move freely while unpowered */
    }
    unlock();
    return err;
}

robot_err_t robot_zero(const float *q_deg)
{
    float q[MOTION_MAX_JOINTS] = {0};
    for (int i = 0; i < KIN_NUM_JOINTS; i++) {
        const float deg = q_deg ? q_deg[i] : s_r.cfg->joints[i].park_deg;
        if (!isfinite(deg)) {
            return ROBOT_ERR_ARG;
        }
        q[i] = DEG_TO_RAD(deg);
    }
    lock();
    robot_err_t err = check_can_move(false);
    if (err == ROBOT_OK && motion_is_busy()) {
        err = ROBOT_ERR_BUSY;
    }
    if (err == ROBOT_OK && kin_check_limits(&s_r.model, q) >= 0) {
        err = ROBOT_ERR_LIMIT;
    }
    if (err == ROBOT_OK) {
        if (motion_set_joint_positions(q) == ESP_OK) {
            s_r.referenced = true;
        } else {
            err = ROBOT_ERR_INTERNAL;
        }
    }
    unlock();
    return err;
}

robot_err_t robot_move_joints(const float q_deg[KIN_NUM_JOINTS], float speed_pct, float accel_pct)
{
    float q[KIN_NUM_JOINTS];
    for (int i = 0; i < KIN_NUM_JOINTS; i++) {
        if (!isfinite(q_deg[i])) {
            return ROBOT_ERR_ARG;
        }
        q[i] = DEG_TO_RAD(q_deg[i]);
    }
    const float speed = clamp_pct(speed_pct), accel = clamp_pct(accel_pct);
    if (speed <= 0.0f || accel <= 0.0f) {
        return ROBOT_ERR_ARG;
    }
    lock();
    robot_err_t err = check_can_move(true);
    if (err == ROBOT_OK && kin_check_limits(&s_r.model, q) >= 0) {
        err = ROBOT_ERR_LIMIT;
    }
    if (err == ROBOT_OK) {
        err = queue_move(q, speed, accel);
    }
    unlock();
    return err;
}

robot_err_t robot_move_pose(const kin_pose_t *pose, kin_elbow_t elbow, float speed_pct, float accel_pct,
                            int *bad_joint)
{
    /* Seed with the commanded position: picks the solution closest to where the arm is. */
    motion_status_t ms;
    motion_get_status(&ms);
    const kin_ik_options_t opts = {.elbow = elbow, .seed = ms.q_cmd};
    float q[KIN_NUM_JOINTS];
    const kin_status_t ks = kin_inverse(&s_r.model, pose, &opts, q, bad_joint);
    switch (ks) {
    case KIN_OK:
        break;
    case KIN_ERR_JOINT_LIMIT:
        return ROBOT_ERR_LIMIT;
    case KIN_ERR_UNREACHABLE:
        return ROBOT_ERR_UNREACHABLE;
    case KIN_ERR_NO_CONVERGENCE:
        return ROBOT_ERR_SINGULAR;
    default:
        return ROBOT_ERR_ARG;
    }
    float q_deg[KIN_NUM_JOINTS];
    for (int i = 0; i < KIN_NUM_JOINTS; i++) {
        q_deg[i] = RAD_TO_DEG(q[i]);
    }
    return robot_move_joints(q_deg, speed_pct, accel_pct);
}

robot_err_t robot_jog(int joint, float delta_deg, float speed_pct)
{
    if (joint < 0 || joint >= KIN_NUM_JOINTS || !isfinite(delta_deg)) {
        return ROBOT_ERR_ARG;
    }
    float speed = clamp_pct(speed_pct);
    if (speed <= 0.0f) {
        return ROBOT_ERR_ARG;
    }
    lock();
    robot_err_t err = check_can_move(false);
    if (err == ROBOT_OK && motion_is_busy()) {
        err = ROBOT_ERR_BUSY;
    }
    if (err == ROBOT_OK) {
        motion_status_t ms;
        motion_get_status(&ms);
        float q[KIN_NUM_JOINTS];
        memcpy(q, ms.q_cmd, sizeof(q));
        q[joint] += DEG_TO_RAD(delta_deg);
        if (s_r.referenced) {
            if (kin_check_limits(&s_r.model, q) >= 0) {
                err = ROBOT_ERR_LIMIT;
            }
        } else {
            speed = fminf(speed, s_r.cfg->unreferenced_speed_scale);
        }
        if (err == ROBOT_OK) {
            err = queue_move(q, speed, speed);
        }
    }
    unlock();
    return err;
}

robot_err_t robot_park(float speed_pct)
{
    float q_deg[KIN_NUM_JOINTS];
    for (int i = 0; i < KIN_NUM_JOINTS; i++) {
        q_deg[i] = s_r.cfg->joints[i].park_deg;
    }
    return robot_move_joints(q_deg, speed_pct, speed_pct);
}

robot_err_t robot_stop(void)
{
    return motion_stop() == ESP_OK ? ROBOT_OK : ROBOT_ERR_INTERNAL;
}

robot_err_t robot_estop(void)
{
    s_r.estop_latched = true;
    return motion_abort() == ESP_OK ? ROBOT_OK : ROBOT_ERR_INTERNAL;
}

robot_err_t robot_reset(void)
{
    lock();
    robot_err_t err = ROBOT_OK;
    if (estop_input_active()) {
        err = ROBOT_ERR_ESTOP;
    } else {
        /* The reference is kept: after an abort at speed verify the position before absolute moves. */
        s_r.estop_latched = false;
    }
    unlock();
    return err;
}

robot_err_t robot_gripper_set(float percent)
{
    if (!(percent >= 0.0f && percent <= 100.0f)) {
        return ROBOT_ERR_ARG;
    }
    if (s_r.estop_latched) {
        return ROBOT_ERR_ESTOP;
    }
    return gripper_set(percent) == ESP_OK ? ROBOT_OK : ROBOT_ERR_INTERNAL;
}

robot_err_t robot_gripper_open(void)
{
    return robot_gripper_set(s_r.cfg->gripper_open_pct);
}

robot_err_t robot_gripper_close(void)
{
    return robot_gripper_set(s_r.cfg->gripper_close_pct);
}

robot_err_t robot_gripper_off(void)
{
    return gripper_off() == ESP_OK ? ROBOT_OK : ROBOT_ERR_INTERNAL;
}

void robot_get_status(robot_status_t *st)
{
    memset(st, 0, sizeof(*st));
    motion_status_t ms;
    motion_get_status(&ms);
    st->state = current_state();
    st->enabled = s_r.enabled;
    st->referenced = s_r.referenced;
    st->moving = ms.busy;
    st->estop_input = estop_input_active();
    st->queued = ms.queued;
    float q[KIN_NUM_JOINTS];
    for (int i = 0; i < KIN_NUM_JOINTS; i++) {
        q[i] = ms.q_cmd[i];
        st->q_deg[i] = RAD_TO_DEG(ms.q_cmd[i]);
        st->steps[i] = ms.steps[i];
    }
    st->pose_valid = s_r.referenced && kin_forward(&s_r.model, q, &st->pose) == KIN_OK;
    st->gripper_on = gripper_is_on();
    st->gripper_pct = gripper_position();
    st->stepgen = ms.stepgen;
}

const robot_config_t *robot_get_config(void)
{
    return s_r.cfg;
}

const kin_model_t *robot_get_model(void)
{
    return &s_r.model;
}

const char *robot_state_str(robot_state_t state)
{
    switch (state) {
    case ROBOT_DISABLED:
        return "DISABLED";
    case ROBOT_ENABLED:
        return "ENABLED";
    case ROBOT_READY:
        return "READY";
    case ROBOT_ESTOP:
        return "ESTOP";
    }
    return "UNKNOWN";
}

const char *robot_err_str(robot_err_t err)
{
    switch (err) {
    case ROBOT_OK:
        return "ok";
    case ROBOT_ERR_ARG:
        return "invalid argument";
    case ROBOT_ERR_STATE:
        return "not allowed in this state (enable first)";
    case ROBOT_ERR_NOT_REFERENCED:
        return "not referenced (use zero)";
    case ROBOT_ERR_BUSY:
        return "busy (moving)";
    case ROBOT_ERR_LIMIT:
        return "joint limit";
    case ROBOT_ERR_UNREACHABLE:
        return "unreachable";
    case ROBOT_ERR_SINGULAR:
        return "no solution near a singularity";
    case ROBOT_ERR_ESTOP:
        return "e-stop active (reset)";
    case ROBOT_ERR_QUEUE_FULL:
        return "command queue full";
    case ROBOT_ERR_INTERNAL:
        return "internal error";
    }
    return "unknown";
}
