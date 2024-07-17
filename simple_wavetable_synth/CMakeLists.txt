cmake_minimum_required(VERSION 3.12)

include(pico_sdk_import.cmake)

project(simple_wavetable_synth)

pico_sdk_init()

add_executable(simple_wavetable_synth
    simple_wavetable_synth.cpp
)

target_link_libraries(simple_wavetable_synth
    pico_stdlib
    pico_time
    hardware_pwm
    hardware_adc
    hardware_interp
)

pico_enable_stdio_usb(simple_wavetable_synth 1)
pico_enable_stdio_uart(simple_wavetable_synth 0)

pico_add_extra_outputs(simple_wavetable_synth)