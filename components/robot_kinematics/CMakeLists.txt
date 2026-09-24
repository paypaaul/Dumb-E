idf_component_register(SRCS "robot_kinematics.c"
                       INCLUDE_DIRS "include"
                       REQUIRES stepper_driver) # Dipende dal componente stepper