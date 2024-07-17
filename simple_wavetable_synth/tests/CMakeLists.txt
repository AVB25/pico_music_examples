cmake_minimum_required(VERSION 3.12)

include(pico_sdk_import.cmake)

project(test_every_step)

pico_sdk_init()

add_executable(test_every_step
    test_every_step.cpp
)

target_link_libraries(test_every_step
    pico_stdlib
    pico_time
    hardware_pwm
    hardware_adc
    hardware_interp
)

pico_enable_stdio_usb(test_every_step 1)
pico_enable_stdio_uart(test_every_step 0)

pico_add_extra_outputs(test_every_step)


add_executable(test_time
    test_time.cpp
)

target_link_libraries(test_time
    pico_stdlib
    pico_time
    hardware_pwm
    hardware_adc
    hardware_interp
)

pico_enable_stdio_usb(test_time 1)
pico_enable_stdio_uart(test_time 0)

pico_add_extra_outputs(test_time)