cmake_minimum_required(VERSION 3.13)
include($ENV{PICO_SDK_PATH}/pico_sdk_init.cmake)
project(encoder_project)
pico_sdk_init()

add_executable(encoder encoder.c)
pico_add_extra_outputs(encoder)
target_link_libraries(encoder pico_stdlib)

pico_enable_stdio_usb(encoder 1)
pico_enable_stdio_uart(encoder 0)
