cmake_minimum_required(VERSION 3.13)
include($ENV{PICO_SDK_PATH}/pico_sdk_init.cmake)
project(blink_project)
pico_sdk_init()
add_executable(blink blink.c)
pico_add_extra_outputs(blink)
target_link_libraries(blink pico_stdlib)

pico_enable_stdio_usb(blink 1)
pico_enable_stdio_uart(blink 0)

