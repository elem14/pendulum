cmake_minimum_required(VERSION 3.13)
include($ENV{PICO_SDK_PATH}/pico_sdk_init.cmake)
project(stepper_project)
pico_sdk_init()

add_executable(stepper stepper.c)
pico_add_extra_outputs(stepper)
target_link_libraries(stepper pico_stdlib)

pico_enable_stdio_usb(stepper 1)
pico_enable_stdio_uart(stepper 0)
