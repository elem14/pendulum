#include "pico/stdlib.h"

#include "pendulum/motor.hpp"
#include "pendulum/state_estimator.hpp"
#include "pendulum/tmc2209_uart.hpp"

#include <stdio.h>


int main() {

    stdio_init_all();

    motor_init();

    state_estimator_init(
        0.02f
    );

    sleep_ms(2000);

    printf("\nConfiguring TMC2209...\n");

    if (!tmc2209_init_spreadcycle()) {

        printf("ERROR: TMC2209 UART configuration failed\n");

        motor_disable();

        while (true) {
            sleep_ms(1000);
        }
    }

    printf(
        "TMC2209 ready — SpreadCycle enabled\n"
    );

    motor_set_limits(
        //non completely tested max limits
        100.0f,  
        100.0f
    );


    motor_disable();

    printf("\nPlace cart at center and hold pendulum upright\n");

    printf("State will zero in 3 seconds...\n");

    sleep_ms(3000);


    state_estimator_zero();


    printf("State zeroed.\n");

    printf("x = 0 m, theta = 0 rad\n\n");


    // lock cart at 0
    motor_enable();

    motor_set_target_angular_velocity(0.0f);


    // times setup
    uint64_t previous_update_us = time_us_64();

    uint64_t previous_print_us = previous_update_us;


    // main state estimation loop
    while (true) {

        uint64_t current_time_us = time_us_64();

        uint64_t delta_time_us = current_time_us - previous_update_us;

        previous_update_us = current_time_us;

        float dt_seconds = static_cast<float>(delta_time_us) * 1.0e-6f;


        // update motor model
        motor_update(dt_seconds);


        // builds latest complete state vector
        state_estimator_update();


        // prints at 10 Hz
        if (current_time_us - previous_print_us >= 100000) {

            PendulumState state = state_estimator_get();

            printf(
                "x: %+8.4f m | "
                "xdot: %+8.4f m/s | "
                "theta: %+8.4f rad | "
                "thetadot: %+8.4f rad/s\n",

                state.cart_position_m,
                state.cart_velocity_m_s,
                state.pendulum_angle_rad,
                state.pendulum_angular_velocity_rad_s
            );

            previous_print_us = current_time_us;

        }

        sleep_ms(1);
    }
}