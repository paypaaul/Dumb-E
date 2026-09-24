#include "robot_kinematics.h"
#include <math.h>

static float L_BASE = 0;
static float L_HUMERUS = 0;
static float L_FOREARM = 0;

void ik_init(float l1, float l2, float l3) {
    L_BASE = l1;    // Altezza base da terra al giunto 2
    L_HUMERUS = l2; // Lunghezza braccio 1
    L_FOREARM = l3; // Lunghezza braccio 2
}

int ik_calculate_inverse(coords_t target, joint_angles_t *out_angles) {
    // 1. Theta 1 (Base Rotation) - Semplice atan2 di Y/X
    out_angles->theta1 = atan2f(target.y, target.x);

    // Coordinate cilindriche proiettate sul piano del braccio
    float r = sqrtf(target.x*target.x + target.y*target.y);
    float z_offset = target.z - L_BASE;

    // Qui va inserita la geometrica per un braccio 3-link planare (o 2 link + polso)
    // Esempio semplificato per spalla-gomito (2DOF planare su R-Z)
    
    // Distanza dall'origine spalla al polso
    float D = (r*r + z_offset*z_offset - L_HUMERUS*L_HUMERUS - L_FOREARM*L_FOREARM) / (2 * L_HUMERUS * L_FOREARM);
    
    if (D < -1.0 || D > 1.0) return -1; // Irraggiungibile

    // Theta 3 (Gomito)
    out_angles->theta3 = atan2f(sqrtf(1 - D*D), D); // Gomito in su

    // Theta 2 (Spalla)
    out_angles->theta2 = atan2f(z_offset, r) - atan2f(L_FOREARM * sinf(out_angles->theta3), L_HUMERUS + L_FOREARM * cosf(out_angles->theta3));

    // Theta 4 (Polso) - Per mantenere l'utensile ad angolo phi rispetto al suolo
    // Theta2 + Theta3 + Theta4 = Phi
    out_angles->theta4 = target.phi - out_angles->theta2 - out_angles->theta3;

    // Converti tutto in gradi
    out_angles->theta1 = out_angles->theta1 * 180.0 / M_PI;
    out_angles->theta2 = out_angles->theta2 * 180.0 / M_PI;
    out_angles->theta3 = out_angles->theta3 * 180.0 / M_PI;
    out_angles->theta4 = out_angles->theta4 * 180.0 / M_PI;

    return 0;
}