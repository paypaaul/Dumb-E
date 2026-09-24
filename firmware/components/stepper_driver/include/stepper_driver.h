#pragma once
#include <stdint.h>
#include "driver/gpio.h"

// Definizione struttura configurazione singolo motore
typedef struct {
    int step_pin;
    int dir_pin;
    int motor_id;
    float steps_per_degree; // Es. 32000 / 90.0
    bool reverse_dir;
} stepper_config_t;

// Inizializza il sistema (Timer, Task su Core 1, GPIO Enable)
void stepper_system_init(int enable_pin);

// Aggiunge un motore al sistema
void stepper_add_motor(stepper_config_t config);

// Abilita/Disabilita tutti i motori
void stepper_enable_all(bool enable);

// Muove un singolo motore (angoli assoluti) - Non bloccante
void stepper_move_absolute(int motor_id, float target_angle_deg, float max_speed, float acceleration);

// Muove tutti i motori interpolati linearmente (tutti arrivano insieme)
// angles array deve avere dimensione pari al numero di motori registrati
void stepper_move_linear_interpolated(float *target_angles, float speed_linear, float acceleration);

// Controlla se il robot si sta muovendo
bool stepper_is_moving();