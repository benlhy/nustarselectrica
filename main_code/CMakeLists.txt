cmake_minimum_required(VERSION 3.8)
project(main_code)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES
        main_code.ino
        pid.h
        radio_functions.h
        sensors.cpp
        sensors.h)

add_executable(main_code ${SOURCE_FILES})