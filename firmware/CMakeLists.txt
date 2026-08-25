#makes program called pendulum_firmware from main.cpp
add_executable(pendulum_firmware
    src/main.cpp
)
#gives pendulum_firmware access to the pico libraries
target_link_libraries(pendulum_firmware
    pico_stdlib
)
# lets CMake generate pendulum_firmware.uf2, .elf, .bin, and .hex files for flashing
pico_add_extra_outputs(pendulum_firmware)

