#include "pico/stdlib.h"
#include "pendulum/motor.hpp"
#include <stdio.h>

int main() {
    stdio_init_all();

    motor_init();

    sleep_ms(2000);

    printf("Motor test starting\n");

    motor_enable();

    sleep_ms(100);

    while (true) {
        printf("Forward\n");

        motor_move_steps(200, 250);

        sleep_ms(1000);

        printf("Reverse\n");

        motor_move_steps(-200, 1000);

        sleep_ms(1000);
    }
}