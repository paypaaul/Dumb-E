#include "stepper_driver.h"
#include "driver/gptimer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <math.h>
#include <string.h>

#define MAX_MOTORS 6
#define TIMER_RESOLUTION_HZ 1000000 // 1MHz resolution
#define TIMER_ALARM_HZ 20000        // 20kHz loop di controllo step (Regolare in base alla velocità max)

typedef struct {
    stepper_config_t config;
    int32_t current_step_pos;
    int32_t target_step_pos;
    float current_speed;     // steps/sec
    float target_speed;      // steps/sec
    float acceleration;      // steps/sec^2
    float accumulator;       // Per generare i passi dal timer
    bool is_moving;
} motor_ctx_t;

static motor_ctx_t motors[MAX_MOTORS];
static int motor_count = 0;
static int global_enable_pin = -1;
static gptimer_handle_t gptimer = NULL;

// Callback del Timer (ISR - Deve essere velocissima)
// Qui generiamo i pulse STEP
static bool IRAM_ATTR on_timer_alarm(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data) {
    bool wake_task = false;
    
    for (int i = 0; i < motor_count; i++) {
        if (motors[i].is_moving) {
            // Aggiorna accumulatore velocità
            motors[i].accumulator += fabsf(motors[i].current_speed);
            
            // Se accumulatore supera frequenza timer, fai un passo
            if (motors[i].accumulator >= TIMER_ALARM_HZ) {
                motors[i].accumulator -= TIMER_ALARM_HZ;
                
                // Imposta direzione
                if (motors[i].target_step_pos > motors[i].current_step_pos) {
                    gpio_set_level(motors[i].config.dir_pin, motors[i].config.reverse_dir ? 0 : 1);
                    motors[i].current_step_pos++;
                } else {
                    gpio_set_level(motors[i].config.dir_pin, motors[i].config.reverse_dir ? 1 : 0);
                    motors[i].current_step_pos--;
                }
                
                // Pulse Step
                gpio_set_level(motors[i].config.step_pin, 1);
                // Breve delay in asm per pulse width (pochi cicli clock)
                for(volatile int j=0; j<10; j++); 
                gpio_set_level(motors[i].config.step_pin, 0);

                // Check arrivo
                if (motors[i].current_step_pos == motors[i].target_step_pos) {
                    motors[i].is_moving = false;
                    motors[i].current_speed = 0;
                }
            }
        }
    }
    return wake_task;
}

// Task Core 1: Calcolo profilo velocità (S-Curve / Trapezoidale)
// Aggiorna motors[i].current_speed
void motion_planner_task(void *arg) {
    const float dt = 0.01f; // 10ms update rate per la velocità
    
    while (1) {
        bool system_moving = false;
        
        for (int i = 0; i < motor_count; i++) {
            if (!motors[i].is_moving && motors[i].current_step_pos == motors[i].target_step_pos) continue;
            
            system_moving = true;
            motors[i].is_moving = true;

            // Calcolo distanza rimanente
            int32_t steps_remaining = abs(motors[i].target_step_pos - motors[i].current_step_pos);
            // Distanza per fermarsi alla velocità attuale: v^2 / 2a
            float stop_dist = (motors[i].current_speed * motors[i].current_speed) / (2.0f * motors[i].acceleration);

            if (steps_remaining <= stop_dist) {
                // Decelerazione (Trapezoidale per semplicità, S-Curve richiede gestione Jerk qui)
                if (motors[i].current_speed > 10.0f) // min speed
                    motors[i].current_speed -= motors[i].acceleration * dt;
                else
                     motors[i].current_speed = 10.0f; // crawl speed finale
            } else {
                // Accelerazione
                if (motors[i].current_speed < motors[i].target_speed) {
                    motors[i].current_speed += motors[i].acceleration * dt;
                    if (motors[i].current_speed > motors[i].target_speed) 
                        motors[i].current_speed = motors[i].target_speed;
                }
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10)); // 100Hz planning loop
    }
}

void stepper_system_init(int enable_pin) {
    global_enable_pin = enable_pin;
    
    // Configura Enable Pin
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << enable_pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    stepper_enable_all(false); // Disabilita all'inizio

    // Crea Timer
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = TIMER_RESOLUTION_HZ,
    };
    gptimer_new_timer(&timer_config, &gptimer);

    gptimer_event_callbacks_t cbs = { .on_alarm = on_timer_alarm };
    gptimer_register_event_callbacks(gptimer, &cbs, NULL);

    gptimer_alarm_config_t alarm_config = {
        .alarm_count = TIMER_RESOLUTION_HZ / TIMER_ALARM_HZ, 
        .reload_count = 0,
        .flags.auto_reload_on_alarm = true,
    };
    gptimer_set_alarm_action(gptimer, &alarm_config);
    gptimer_enable(gptimer);
    gptimer_start(gptimer);

    // Crea Task su Core 1
    xTaskCreatePinnedToCore(motion_planner_task, "MotionCore1", 4096, NULL, 5, NULL, 1);
}

void stepper_add_motor(stepper_config_t config) {
    if (motor_count >= MAX_MOTORS) return;
    
    motors[motor_count].config = config;
    motors[motor_count].current_step_pos = 0;
    motors[motor_count].target_step_pos = 0;
    motors[motor_count].current_speed = 0;
    motors[motor_count].is_moving = false;

    // Config GPIO
    gpio_set_direction(config.step_pin, GPIO_MODE_OUTPUT);
    gpio_set_direction(config.dir_pin, GPIO_MODE_OUTPUT);
    
    motor_count++;
}

void stepper_enable_all(bool enable) {
    // LOW solitamente abilita i driver stepper standard
    gpio_set_level(global_enable_pin, enable ? 0 : 1);
}

void stepper_move_absolute(int motor_id, float target_angle_deg, float max_speed, float acceleration) {
    if (motor_id >= motor_count) return;
    
    // Conversione Angolo -> Step
    // 32000 step = 90 gradi -> ratio = 355.55
    int32_t target_steps = (int32_t)(target_angle_deg * motors[motor_id].config.steps_per_degree);
    
    motors[motor_id].target_step_pos = target_steps;
    motors[motor_id].target_speed = max_speed * motors[motor_id].config.steps_per_degree; // deg/s -> step/s
    motors[motor_id].acceleration = acceleration * motors[motor_id].config.steps_per_degree;
    // Il task Motion Planner rileverà la differenza e inizierà a muovere
}

void stepper_move_linear_interpolated(float *target_angles, float speed_linear, float acceleration) {
    // 1. Trova l'asse che deve fare più passi (Dominant Axis)
    int32_t max_delta_steps = 0;
    int dominant_axis = 0;
    int32_t deltas[MAX_MOTORS];

    for(int i=0; i<motor_count; i++) {
        int32_t target = (int32_t)(target_angles[i] * motors[i].config.steps_per_degree);
        deltas[i] = abs(target - motors[i].current_step_pos);
        if (deltas[i] > max_delta_steps) {
            max_delta_steps = deltas[i];
            dominant_axis = i;
        }
    }

    if (max_delta_steps == 0) return;

    // 2. Calcola tempo basato sull'asse dominante
    // Velocità e Accel vengono scalate per gli altri assi
    float dominant_speed_steps = speed_linear * motors[dominant_axis].config.steps_per_degree;
    
    for(int i=0; i<motor_count; i++) {
        float ratio = (float)deltas[i] / (float)max_delta_steps;
        
        int32_t target = (int32_t)(target_angles[i] * motors[i].config.steps_per_degree);
        motors[i].target_step_pos = target;
        
        // Scaliamo velocità e accelerazione in base al rapporto di distanza
        // Così tutti iniziano e finiscono insieme
        motors[i].target_speed = dominant_speed_steps * ratio;
        motors[i].acceleration = (acceleration * motors[i].config.steps_per_degree) * ratio;
    }
}