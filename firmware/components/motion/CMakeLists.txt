idf_component_register(SRCS "motion.c" "motion_profile.c"
                       INCLUDE_DIRS "include"
                       REQUIRES kinematics stepgen esp_common freertos
                       PRIV_REQUIRES log)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror)
