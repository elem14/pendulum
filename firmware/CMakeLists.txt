#makes program called pendulum_firmware from main.cpp
add_executable(pendulum_firmware
    src/main.cpp
    src/encoder.cpp
    src/velocity_estimator.cpp
)

target_include_directories(pendulum_firmware
    PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/include
)

#gives pendulum_firmware access to the pico libraries
target_link_libraries(pendulum_firmware
    pico_stdlib
)

pico_enable_stdio_usb(pendulum_firmware 1)
pico_enable_stdio_usb(pendulum_firmware 0)


# lets CMake generate pendulum_firmware.uf2, .elf, .bin, and .hex files for flashing
pico_add_extra_outputs(pendulum_firmware)

