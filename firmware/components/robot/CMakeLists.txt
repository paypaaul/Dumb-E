idf_component_register(SRCS "robot.c" "robot_config.c"
                       INCLUDE_DIRS "include"
                       REQUIRES kinematics stepgen
                       PRIV_REQUIRES board motion gripper esp_driver_gpio esp_timer freertos log)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror)
