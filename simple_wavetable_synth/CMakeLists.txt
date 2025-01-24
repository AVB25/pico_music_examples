cmake_minimum_required(VERSION 3.12)

if (NOT EXISTS $ENV{PICO_SDK_PATH})
    message(FATAL_ERROR "Pico SDK Path not found, please set correct environment variable PICO_SDK_PATH")
endif ()

include($ENV{PICO_SDK_PATH}/pico_sdk_init.cmake)

project(simple_wavetable_synth)

pico_sdk_init()

add_subdirectory(pico_hardware_utils)

add_executable(simple_wavetable_synth
    simple_wavetable_synth.cpp
)

target_link_libraries(simple_wavetable_synth
    pico_stdlib
    pico_time
    pico_hardware_utils
)
target_include_directories(simple_wavetable_synth PUBLIC "${CMAKE_SOURCE_DIR}/pico_hardware_utils")

pico_enable_stdio_usb(simple_wavetable_synth 1)
pico_enable_stdio_uart(simple_wavetable_synth 0)

pico_add_extra_outputs(simple_wavetable_synth)