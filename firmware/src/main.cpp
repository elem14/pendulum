#include "pico/stdlib.h"

#include "pendulum/motor.hpp"
#include "pendulum/tmc2209_uart.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>


namespace {

constexpr float TWO_PI =
    6.28318530717958647692f;


// =================================================
// Belt transmission
//
// 36T GT2 pulley
// 36 teeth * 2 mm = 72 mm/rev
// =================================================

constexpr float CART_METERS_PER_MOTOR_REV =
    0.072f;

constexpr float CART_METERS_PER_MOTOR_RAD =
    CART_METERS_PER_MOTOR_REV
    /
    TWO_PI;


// =================================================
// Test geometry
// =================================================

// Oscillate between +/- 6 cm around track center.
constexpr float POSITION_SETPOINT_M =
    0.060f;


// Emergency software bound.
// The normal test should never get near this.
constexpr float SAFETY_LIMIT_M =
    0.150f;


// =================================================
// Motion settings
// =================================================

// Keep velocity modest.
//
// This test is about acceleration,
// not maximum motor speed.
constexpr float CRUISE_SPEED_RAD_S =
    75.0f;


// 1 mm position tolerance.
constexpr float POSITION_TOLERANCE_M =
    0.001f;


// Consider cart stopped below ~5 mm/s.
constexpr float STOPPED_CART_VELOCITY_M_S =
    0.005f;


// Two full back-and-forth cycles
// for each acceleration setting.
constexpr int CYCLES_PER_ACCELERATION =
    2;


// Prevent an unexpected condition from
// commanding motion indefinitely.
constexpr uint32_t MOVE_TIMEOUT_MS =
    5000;


// =================================================
// Acceleration sweep
// =================================================

constexpr float ACCELERATION_TESTS[] = {
    500.0f,
    750.0f,
    1000.0f,
    1500.0f,
    2000.0f,
};


constexpr int NUM_ACCELERATION_TESTS =
    sizeof(ACCELERATION_TESTS)
    /
    sizeof(ACCELERATION_TESTS[0]);

}


// =================================================
// Update motor using real elapsed time
// =================================================

void update_motor(
    uint64_t& previous_time_us
) {

    uint64_t now_us =
        time_us_64();


    float dt_seconds =
        static_cast<float>(
            now_us
            -
            previous_time_us
        )
        *
        1.0e-6f;


    previous_time_us =
        now_us;


    motor_update(
        dt_seconds
    );
}


// =================================================
// Move cart to one absolute position.
//
// Instead of moving for a fixed amount of time,
// calculate the maximum velocity from which the
// cart can still stop at the target.
//
// Motor-space kinematics:
//
//     omega^2 = 2 * alpha * delta_theta
//
// Therefore:
//
//     omega_stop = sqrt(
//         2 * alpha * remaining_motor_angle
//     )
// =================================================

bool move_to_position(
    float target_position_m,
    float acceleration_rad_s2
) {

    uint64_t start_time_us =
        time_us_64();


    uint64_t previous_time_us =
        start_time_us;


    while (true) {

        update_motor(
            previous_time_us
        );


        float position_m =
            motor_get_estimated_cart_position_m();


        float velocity_m_s =
            motor_get_estimated_cart_velocity_m_s();


        // -----------------------------------------
        // Emergency travel protection
        // -----------------------------------------

        if (
            std::fabs(position_m)
            >
            SAFETY_LIMIT_M
        ) {

            printf(
                "\nABORT: software travel limit exceeded.\n"
            );

            printf(
                "Estimated x = %.4f m\n",
                position_m
            );


            motor_set_target_angular_velocity(
                0.0f
            );


            motor_disable();


            return false;
        }


        // -----------------------------------------
        // Timeout protection
        // -----------------------------------------

        uint64_t elapsed_us =
            time_us_64()
            -
            start_time_us;


        if (
            elapsed_us
            >
            static_cast<uint64_t>(
                MOVE_TIMEOUT_MS
            )
            *
            1000
        ) {

            printf(
                "\nABORT: movement timed out.\n"
            );


            motor_set_target_angular_velocity(
                0.0f
            );


            motor_disable();


            return false;
        }


        // -----------------------------------------
        // Position error
        // -----------------------------------------

        float error_m =
            target_position_m
            -
            position_m;


        // -----------------------------------------
        // Target reached?
        // -----------------------------------------

        if (
            std::fabs(error_m)
            <=
            POSITION_TOLERANCE_M
            &&
            std::fabs(velocity_m_s)
            <=
            STOPPED_CART_VELOCITY_M_S
        ) {

            motor_set_target_angular_velocity(
                0.0f
            );


            printf(
                "Reached target %+.3f m"
                " | estimated x = %+.4f m\n",

                target_position_m,

                position_m
            );


            return true;
        }


        // -----------------------------------------
        // Remaining linear distance
        //        ↓
        // remaining motor angle
        //
        // x = k_x * theta_m
        // -----------------------------------------

        float remaining_motor_angle_rad =
            std::fabs(error_m)
            /
            CART_METERS_PER_MOTOR_RAD;


        // -----------------------------------------
        // How fast can we travel RIGHT NOW
        // and still stop at the destination?
        //
        // omega^2 = 2 alpha delta_theta
        // -----------------------------------------

        float stopping_velocity_rad_s =
            std::sqrt(
                2.0f
                *
                acceleration_rad_s2
                *
                remaining_motor_angle_rad
            );


        // Don't exceed our chosen cruise speed.
        float velocity_magnitude_rad_s =
            std::min(
                CRUISE_SPEED_RAD_S,
                stopping_velocity_rad_s
            );


        float direction =
            error_m >= 0.0f
            ? 1.0f
            : -1.0f;


        motor_set_target_angular_velocity(
            direction
            *
            velocity_magnitude_rad_s
        );


        sleep_ms(1);
    }
}


// =================================================
// One acceleration level
// =================================================

bool run_acceleration_stage(
    float acceleration_rad_s2
) {

    printf("\n");
    printf("========================================\n");
    printf(
        "ACCELERATION TEST: %.1f rad/s^2\n",
        acceleration_rad_s2
    );
    printf(
        "Cruise speed:      %.1f rad/s\n",
        CRUISE_SPEED_RAD_S
    );
    printf(
        "Setpoints:         +/- %.3f m\n",
        POSITION_SETPOINT_M
    );
    printf("========================================\n");


    motor_set_limits(
        CRUISE_SPEED_RAD_S,
        acceleration_rad_s2
    );


    for (
        int cycle = 0;
        cycle < CYCLES_PER_ACCELERATION;
        ++cycle
    ) {

        printf(
            "\nCycle %d/%d\n",
            cycle + 1,
            CYCLES_PER_ACCELERATION
        );


        // Center -> right
        if (
            !move_to_position(
                POSITION_SETPOINT_M,
                acceleration_rad_s2
            )
        ) {
            return false;
        }


        // Right -> left
        if (
            !move_to_position(
                -POSITION_SETPOINT_M,
                acceleration_rad_s2
            )
        ) {
            return false;
        }


        // Left -> right
        if (
            !move_to_position(
                POSITION_SETPOINT_M,
                acceleration_rad_s2
            )
        ) {
            return false;
        }
    }


    printf(
        "\nReturning to center...\n"
    );


    if (
        !move_to_position(
            0.0f,
            acceleration_rad_s2
        )
    ) {
        return false;
    }


    printf(
        "Stage %.1f rad/s^2 complete.\n",
        acceleration_rad_s2
    );


    // Let you physically inspect the center mark
    // before the next acceleration begins.
    sleep_ms(2000);


    return true;
}


// =================================================
// MAIN
// =================================================

int main() {

    stdio_init_all();


    motor_init();


    sleep_ms(2000);


    printf("\n");
    printf("========================================\n");
    printf("MOTOR ACCELERATION CAPABILITY TEST\n");
    printf("========================================\n");


    // =================================================
    // TMC2209
    // =================================================

    printf(
        "Configuring TMC2209...\n"
    );


    if (
        !tmc2209_init_spreadcycle()
    ) {

        printf(
            "ERROR: TMC2209 initialization failed.\n"
        );


        motor_disable();


        while (true) {
            sleep_ms(1000);
        }
    }


    printf(
        "SpreadCycle enabled.\n"
    );


    // =================================================
    // Establish physical cart center
    // =================================================

    motor_disable();


    printf("\n");
    printf(
        "Clamp pendulum straight DOWN.\n"
    );

    printf(
        "Place cart at physical CENTER mark.\n"
    );

    printf(
        "Test begins in 5 seconds...\n"
    );


    sleep_ms(5000);


    motor_set_zero_position();


    motor_enable();


    motor_set_target_angular_velocity(
        0.0f
    );


    sleep_ms(500);


    // =================================================
    // Run sweep
    // =================================================

    for (
        int i = 0;
        i < NUM_ACCELERATION_TESTS;
        ++i
    ) {

        if (
            !run_acceleration_stage(
                ACCELERATION_TESTS[i]
            )
        ) {

            printf("\n");
            printf("TEST ABORTED.\n");


            while (true) {
                sleep_ms(1000);
            }
        }
    }


    // =================================================
    // Finished
    // =================================================

    motor_set_target_angular_velocity(
        0.0f
    );


    sleep_ms(1000);


    motor_disable();


    printf("\n");
    printf("========================================\n");
    printf("ACCELERATION SWEEP COMPLETE\n");
    printf("========================================\n");


    while (true) {
        sleep_ms(1000);
    }
}