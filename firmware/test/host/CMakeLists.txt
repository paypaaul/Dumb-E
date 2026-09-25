# Host-side unit tests for the pure-C parts of the firmware (no ESP-IDF needed).
#   cmake -S firmware/test/host -B build-host && cmake --build build-host && ctest --test-dir build-host
cmake_minimum_required(VERSION 3.22)
project(dumb_e_host_tests C)

set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)

include(FetchContent)
FetchContent_Declare(unity
    GIT_REPOSITORY https://github.com/ThrowTheSwitch/Unity.git
    GIT_TAG        v2.6.1
    GIT_SHALLOW    TRUE)
FetchContent_MakeAvailable(unity)
target_compile_definitions(unity PUBLIC UNITY_INCLUDE_DOUBLE)

set(COMPONENTS_DIR ${CMAKE_CURRENT_LIST_DIR}/../../components)
set(WARNINGS -Wall -Wextra -Werror -Wshadow -Wdouble-promotion)

add_library(pure_libs STATIC
    ${COMPONENTS_DIR}/kinematics/kinematics.c
    ${COMPONENTS_DIR}/motion/motion_profile.c
    ${COMPONENTS_DIR}/stepgen/stepgen_dda.c)
target_include_directories(pure_libs PUBLIC
    ${COMPONENTS_DIR}/kinematics/include
    ${COMPONENTS_DIR}/motion/include
    ${COMPONENTS_DIR}/stepgen/include)
target_compile_options(pure_libs PRIVATE ${WARNINGS})
target_link_libraries(pure_libs PUBLIC m)

enable_testing()
foreach(test_name test_kinematics test_motion_profile test_stepgen_dda test_segment_ring)
    add_executable(${test_name} ${test_name}.c)
    target_compile_options(${test_name} PRIVATE ${WARNINGS})
    target_link_libraries(${test_name} PRIVATE pure_libs unity)
    add_test(NAME ${test_name} COMMAND ${test_name})
endforeach()
