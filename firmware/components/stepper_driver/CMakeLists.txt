idf_component_register(SRCS "stepper_driver.c"
                       INCLUDE_DIRS "include"
                       REQUIRES driver freertos esp_driver_gpio esp_driver_gptimer)