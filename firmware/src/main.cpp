#include "pico/stdlib.h"

#include "pendulum/motor.hpp"

#include <stdio.h>


void run_motor_for(
    float target_velocity_rad_s,
    uint32_t duration_ms
) {
    motor_set_target_angular_velocity(
        target_velocity_rad_s
    );

    uint64_t start_time_us =
        time_us_64();

    uint64_t previous_time_us =
        start_time_us;

    while (
        time_us_64() - start_time_us
        <
        static_cast<uint64_t>(duration_ms) * 1000
    ) {
        uint64_t current_time_us =
            time_us_64();

        uint64_t delta_time_us =
            current_time_us
            - previous_time_us;

        previous_time_us =
            current_time_us;

        float dt_seconds =
            static_cast<float>(
                delta_time_us
            )
            * 1.0e-6f;

        motor_update(
            dt_seconds
        );

        sleep_ms(1);
    }
}


void test_speed(
    float speed_rad_s,
    uint32_t hold_ms
) {
    printf(
        "\nTesting %.2f rad/s\n",
        speed_rad_s
    );

    // Forward
    run_motor_for(
        speed_rad_s,
        hold_ms
    );

    // Ramp to zero
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
    stdio_init_all();

    motor_init();

    motor_set_limits(
        10.0f,   // max velocity for this test
        5.0f    // gentle acceleration
    );

    sleep_ms(2000);

    printf(
        "\ncart displacement test starting\n"
    );

    motor_enable();

    sleep_ms(100);

    motor_set_zero_position();

    printf("Moving exactly 2 motor revolutions...\n");

    motor_move_steps_blocking(
        3200,   // 1600 steps/rev × 2 rev
        1000    // safe half-period in microseconds
    );

    printf(
        "Estimated motor position: %.4f rad\n",
        motor_get_estimated_position_rad()
    );

    printf(
        "Estimated cart position: %.4f m\n",
        motor_get_estimated_cart_position_m()
    );

    printf("Movement finished\n");

    printf("Returning exactly 2 motor revolutions...\n");

    motor_move_steps_blocking(
        -3200,   // exactly 2 revolutions backward
        1000
    );

    printf(
        "Final estimated motor position: %.4f rad\n",
        motor_get_estimated_position_rad()
    );

    printf(
        "Final estimated cart position: %.4f m\n",
        motor_get_estimated_cart_position_m()
    );

    printf("Return movement finished\n");

    // actual test call
    /*test_speed(
        8.0f,
        10000
    );

    printf(
        "\nTest finished\n"
    );*/

    motor_disable();

    while (true) {
        sleep_ms(1000);
    }
}