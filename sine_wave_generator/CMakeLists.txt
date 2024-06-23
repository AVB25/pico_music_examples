cmake_minimum_required(VERSION 3.12)

include(pico_sdk_import.cmake)

project(sine_wave_generator)

pico_sdk_init()

add_executable(sine_wave_generator
    sine_wave_generator.cpp
)

target_link_libraries(sine_wave_generator
    pico_stdlib
    pico_time
    hardware_pwm
    # pico_cyw43_arch_none
)

pico_enable_stdio_usb(sine_wave_generator 1)
pico_enable_stdio_uart(sine_wave_generator 0)

pico_add_extra_outputs(sine_wave_generator)