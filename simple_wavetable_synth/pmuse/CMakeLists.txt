cmake_minimum_required(VERSION 3.12)

if (NOT EXISTS $ENV{PICO_SDK_PATH})
    message(FATAL_ERROR "Pico SDK Path not found, please set correct environment variable PICO_SDK_PATH")
endif ()

include($ENV{PICO_SDK_PATH}/pico_sdk_init.cmake)

project(pmuse)

pico_sdk_init()

add_library(pmuse waveform.cpp)

target_link_libraries(pmuse
    pico_stdlib)