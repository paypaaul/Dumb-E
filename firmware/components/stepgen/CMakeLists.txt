idf_component_register(SRCS "stepgen.c" "stepgen_dda.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_common freertos
                       PRIV_REQUIRES esp_driver_gpio esp_driver_gptimer esp_hal_gpio esp_hw_support esp_rom esp_timer log
                       LDFRAGMENTS "linker.lf")
target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror)
