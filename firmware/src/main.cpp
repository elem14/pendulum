#include "pico/stdlib.h"

#include "pendulum/motor.hpp"

#include <stdio.h>

void run_motor_for(float target_velocity_rad_s, uint32_t duration_ms) {
    motor_set_target_angular_velocity(target_velocity_rad_s);

    uint64_t start_time_us = time_us_64();
    uint64_t previous_time_us = start_time_us;
    uint64_t previous_print_time_us = start_time_us;

    while (time_us_64() - start_time_us < static_cast<uint64_t>(duration_ms) * 1000) {
        uint64_t current_time_us = time_us_64();
        uint64_t delta_time_us = current_time_us - previous_time_us;

        previous_time_us = current_time_us;

        float dt_seconds = static_cast<float>(delta_time_us) * 1.0e-6f;

        motor_update(dt_seconds);

        if (current_time_us - previous_print_time_us >= 10000) {
            printf(
                "target: %6.3f | "
                "est omega: %6.3f | "
                "est theta: %8.4f | "
                "STEP: %8.2f Hz\n",

                motor_get_target_angular_velocity(),
                motor_get_estimated_velocity_rad_s(),
                motor_get_estimated_position_rad(),
                motor_get_step_frequency()
            );

            previous_print_time_us = current_time_us;
        }

        sleep_ms(1);
    }
}

void test_speed(
    float speed_rad_s,
    uint32_t hold_ms
) {
    printf(
        "\n=== Testing %.2f rad/s ===\n",
        speed_rad_s
    );

    // Forward
    run_motor_for(
        speed_rad_s,
        hold_ms
    );

    // Ramp to zero and remain stopped
    run_motor_for(
        0.0f,
        1500
    );

    // Reverse
    run_motor_for(
        -speed_rad_s,
        hold_ms
    );

    // Ramp back to zero
    run_motor_for(
        0.0f,
        1500
    );

    printf(
        "Estimated position after round trip: %.4f rad\n",
        motor_get_estimated_position_rad()
    );
}


int main() {
    /*stdio_init_all();
    motor_init();
    motor_set_limits(6.0f, 10.0f); //max v, max a

    sleep_ms(2000);
    printf("\nMotor model test\n");

    motor_enable();

    sleep_ms(100);

    motor_set_zero_position(); //current shaft orientation is theta_m = 0

    printf("\nStarting position: %.4f rad\n", motor_get_estimated_position_rad());

    printf("\nCommanding +2 rad/s for 3 seconds\n");

    run_motor_for(10.0f, 5000);

    printf("\nCommanding stop\n");

    run_motor_for(0.0f, 1000);


    printf("\nAfter forward test:\n");

    printf("Estimated position: %.4f rad\n", motor_get_estimated_position_rad());

    //reverse same motion

    printf("\nCommanding -2 rad/s for 3 seconds\n");

    run_motor_for(-10.0f, 5000);

    printf("\nCommanding stop\n");

    run_motor_for(0.0f, 1000);

    printf("\nFinal estimated position: %.4f rad\n", motor_get_estimated_position_rad());

    motor_disable();

    while (true) {
        sleep_ms(1000);
    }
    */

    motor_enable();
    motor_set_zero_position();

    test_speed(2.0f, 3000);
}