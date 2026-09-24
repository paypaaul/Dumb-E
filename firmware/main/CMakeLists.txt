idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS ""
                       PRIV_REQUIRES robot net comms nvs_flash esp_event log)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror)
