idf_component_register(SRCS "board.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_common
                       PRIV_REQUIRES esp_driver_gpio log)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror)
