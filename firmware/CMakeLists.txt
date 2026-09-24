# Dumb-E firmware (ESP-IDF v6.0.x)
cmake_minimum_required(VERSION 3.22)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
# Only build main and the components it (transitively) requires.
idf_build_set_property(MINIMAL_BUILD ON)
project(dumb_e)
