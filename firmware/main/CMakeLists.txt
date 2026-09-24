idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS ""
                       REQUIRES stepper_driver wifi_connection robot_kinematics nvs_flash)