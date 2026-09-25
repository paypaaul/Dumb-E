idf_component_register(SRCS "gripper.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_common
                       PRIV_REQUIRES esp_driver_ledc esp_timer freertos log)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror)
