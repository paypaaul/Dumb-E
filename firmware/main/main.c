#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_log.h"

#include "wifi_connection.h"
#include "stepper_driver.h"
#include "robot_kinematics.h"

// Pin Configuration (Esempio)
#define PIN_ENABLE 25
#define PIN_STEP_1 12
#define PIN_DIR_1  13
#define PIN_STEP_2 26
#define PIN_DIR_2  27             
// ... altri pin

void app_main(void)
{
    // 1. Init NVS (necessario per WiFi)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 2. Init WiFi (Core 0 background)
    wifi_init_sta("MioSSID", "MiaPassword");

    // 3. Init Kinematics
    ik_init(100.0, 150.0, 150.0); // Esempio lunghezze mm

    // 4. Init Steppers
    stepper_system_init(PIN_ENABLE);

    // Configura i 4 motori
    float steps_per_deg_ratio = 32000.0f / 90.0f; // 355.55
    
    stepper_config_t m1 = { .step_pin=PIN_STEP_1, .dir_pin=PIN_DIR_1, .steps_per_degree=steps_per_deg_ratio, .reverse_dir=false };
    stepper_add_motor(m1); // ID 0

    stepper_config_t m2 = { .step_pin=PIN_STEP_2, .dir_pin=PIN_DIR_2, .steps_per_degree=steps_per_deg_ratio, .reverse_dir=false };
    stepper_add_motor(m2); // ID 1
    
    // Aggiungi m3, m4...

    stepper_enable_all(true);
    vTaskDelay(pdMS_TO_TICKS(1000));

    // 5. Loop Logica (Core 0)
    // Esempio: Muovi a coordinata XYZ
    joint_angles_t joints;

    float move_cmds[2];
    move_cmds[0] = 90;
    move_cmds[1] = 45;

        // Muovi linearmente (interpolato)
    stepper_move_linear_interpolated(move_cmds, 400.0, 100.0); // Speed 20 deg/s, Accel 10 deg/s^2

    while (1) {
        // Qui potrai aggiungere logica per ricevere comandi via WiFi (Socket/MQTT)
        // e passarli alla funzione stepper_move_...
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}