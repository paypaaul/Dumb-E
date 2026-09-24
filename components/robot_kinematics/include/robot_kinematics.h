#pragma once

typedef struct {
    float theta1; // Base
    float theta2; // Spalla
    float theta3; // Gomito
    float theta4; // Polso
} joint_angles_t;

typedef struct {
    float x;
    float y;
    float z;
    float phi; // Angolo finale end-effector rispetto al piano
} coords_t;

// Inizializza parametri link (Lunghezze in mm)
void ik_init(float l1, float l2, float l3);

// Calcola angoli target dato XYZ + phi
// Ritorna 0 su successo, -1 se irraggiungibile
int ik_calculate_inverse(coords_t target, joint_angles_t *out_angles);