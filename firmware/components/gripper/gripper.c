#include "gripper.h"

#include <math.h>

#include "driver/ledc.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"

static const char *TAG = "gripper";

#define GRIPPER_LEDC_MODE    LEDC_LOW_SPEED_MODE
#define GRIPPER_LEDC_TIMER   LEDC_TIMER_0
#define GRIPPER_LEDC_CHANNEL LEDC_CHANNEL_0
#define GRIPPER_PWM_HZ       50
#define GRIPPER_PERIOD_US    (1000000 / GRIPPER_PWM_HZ)
#define GRIPPER_DUTY_BITS    14
#define GRIPPER_RAMP_MS      20

static struct {
    gripper_config_t cfg;
    esp_timer_handle_t ramp_timer;
    portMUX_TYPE lock;
    bool initialized;
    bool on;
    float position; /* percent */
    float target;   /* percent */
} s_g = {.lock = portMUX_INITIALIZER_UNLOCKED};

static void apply_position(float percent)
{
    const float pulse_us =
        s_g.cfg.pulse_min_us + (s_g.cfg.pulse_max_us - s_g.cfg.pulse_min_us) * (percent / 100.0f);
    const uint32_t duty = (uint32_t)lroundf(pulse_us * (float)(1u << GRIPPER_DUTY_BITS) / GRIPPER_PERIOD_US);
    ledc_set_duty(GRIPPER_LEDC_MODE, GRIPPER_LEDC_CHANNEL, duty);
    ledc_update_duty(GRIPPER_LEDC_MODE, GRIPPER_LEDC_CHANNEL);
}

static void ramp_cb(void *arg)
{
    (void)arg;
    const float step = s_g.cfg.speed_pct_per_s * (GRIPPER_RAMP_MS / 1000.0f);
    portENTER_CRITICAL(&s_g.lock);
    const bool on = s_g.on;
    const float target = s_g.target;
    float pos = s_g.position;
    portEXIT_CRITICAL(&s_g.lock);
    if (!on || pos == target) {
        return;
    }
    if (fabsf(target - pos) <= step) {
        pos = target;
    } else {
        pos += target > pos ? step : -step;
    }
    apply_position(pos);
    portENTER_CRITICAL(&s_g.lock);
    s_g.position = pos;
    portEXIT_CRITICAL(&s_g.lock);
}

esp_err_t gripper_init(const gripper_config_t *config)
{
    ESP_RETURN_ON_FALSE(!s_g.initialized, ESP_ERR_INVALID_STATE, TAG, "already initialized");
    ESP_RETURN_ON_FALSE(config != NULL && config->pulse_min_us > 0 && config->pulse_max_us > 0 &&
                            config->pulse_min_us < GRIPPER_PERIOD_US && config->pulse_max_us < GRIPPER_PERIOD_US,
                        ESP_ERR_INVALID_ARG, TAG, "invalid pulse range");
    s_g.cfg = *config;

    const ledc_timer_config_t timer = {
        .speed_mode = GRIPPER_LEDC_MODE,
        .duty_resolution = GRIPPER_DUTY_BITS,
        .timer_num = GRIPPER_LEDC_TIMER,
        .freq_hz = GRIPPER_PWM_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_RETURN_ON_ERROR(ledc_timer_config(&timer), TAG, "ledc timer");
    const ledc_channel_config_t channel = {
        .gpio_num = config->gpio,
        .speed_mode = GRIPPER_LEDC_MODE,
        .channel = GRIPPER_LEDC_CHANNEL,
        .timer_sel = GRIPPER_LEDC_TIMER,
        .duty = 0, /* no pulses until the first command */
        .hpoint = 0,
    };
    ESP_RETURN_ON_ERROR(ledc_channel_config(&channel), TAG, "ledc channel");

    const esp_timer_create_args_t args = {.callback = ramp_cb, .name = "gripper"};
    ESP_RETURN_ON_ERROR(esp_timer_create(&args, &s_g.ramp_timer), TAG, "timer");
    ESP_RETURN_ON_ERROR(esp_timer_start_periodic(s_g.ramp_timer, GRIPPER_RAMP_MS * 1000), TAG, "timer start");
    s_g.initialized = true;
    return ESP_OK;
}

esp_err_t gripper_set(float percent)
{
    ESP_RETURN_ON_FALSE(s_g.initialized, ESP_ERR_INVALID_STATE, TAG, "not initialized");
    ESP_RETURN_ON_FALSE(percent >= 0.0f && percent <= 100.0f, ESP_ERR_INVALID_ARG, TAG, "out of range");
    portENTER_CRITICAL(&s_g.lock);
    const bool jump = !s_g.on || s_g.cfg.speed_pct_per_s <= 0.0f;
    s_g.target = percent;
    if (jump) {
        s_g.position = percent;
    }
    s_g.on = true;
    portEXIT_CRITICAL(&s_g.lock);
    if (jump) {
        apply_position(percent);
    }
    return ESP_OK;
}

esp_err_t gripper_off(void)
{
    ESP_RETURN_ON_FALSE(s_g.initialized, ESP_ERR_INVALID_STATE, TAG, "not initialized");
    portENTER_CRITICAL(&s_g.lock);
    s_g.on = false;
    portEXIT_CRITICAL(&s_g.lock);
    ledc_set_duty(GRIPPER_LEDC_MODE, GRIPPER_LEDC_CHANNEL, 0);
    ledc_update_duty(GRIPPER_LEDC_MODE, GRIPPER_LEDC_CHANNEL);
    return ESP_OK;
}

bool gripper_is_on(void)
{
    return s_g.on;
}

float gripper_position(void)
{
    return s_g.position;
}

float gripper_target(void)
{
    return s_g.target;
}
