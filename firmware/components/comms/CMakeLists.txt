idf_component_register(SRCS "comms.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_common
                       PRIV_REQUIRES console robot net kinematics esp_app_format esp_rom log)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror)
