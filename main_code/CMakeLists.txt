cmake_minimum_required(VERSION 3.8)
project(main_code)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES
        main_code.ino
        sensors.cpp
        sensors.h
        storage.cpp
        storage.h
        pid.cpp
        pid.h
        radio.cpp
        radio.h
        )

add_executable(main_code ${SOURCE_FILES})