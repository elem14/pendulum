#include "pico/stdlib.h"

#include "pendulum/motor.hpp"
#include "pendulum/tmc2209_uart.hpp"

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

    uint64_t previous_diagnostic_time_us = start_time_us;

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

        if (
            current_time_us
            - previous_diagnostic_time_us
            >= 250000
        ) {
            uint32_t tstep = 0;
            uint32_t drv_status = 0;

            if (
                tmc2209_read_tstep(tstep)
                &&
                tmc2209_read_drv_status(
                    drv_status
                )
            ) {
                uint32_t cs_actual =
                    (drv_status >> 16)
                    & 0x1F;

                printf(
                    "target: %.2f | "
                    "TSTEP: %lu | "
                    "CS_ACTUAL: %lu | "
                    "STEP freq: %.1f Hz\n",

                    target_velocity_rad_s,

                    static_cast<unsigned long>(
                        tstep
                    ),

                    static_cast<unsigned long>(
                        cs_actual
                    ),

                    motor_get_step_frequency()
                );
            }

            previous_diagnostic_time_us =
                current_time_us;
        }

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


void test_loaded_acceleration(float acceleration_rad_s2) {
    constexpr float TEST_SPEED_RAD_S = 20.0f;

    constexpr uint32_t COMMAND_TIME_MS = 3000;

    motor_set_limits(TEST_SPEED_RAD_S, acceleration_rad_s2);

    motor_set_zero_position();

    uint32_t stop_time_ms = static_cast<uint32_t>(
        (TEST_SPEED_RAD_S / acceleration_rad_s2) * 1000.0f
    ) + 500;

    printf("\n=== Loaded acceleration test ===\n");

    printf("Acceleration: %.2f rad/s^2\n", acceleration_rad_s2);

    //forward

    run_motor_for(
        TEST_SPEED_RAD_S,
        COMMAND_TIME_MS
    );

    run_motor_for(
        0.0f,
        stop_time_ms
    );

    printf(
        "Forward position: %.4f m\n",
        motor_get_estimated_cart_position_m()
    );

    //reverse

    run_motor_for(
        -TEST_SPEED_RAD_S,
        COMMAND_TIME_MS
    );

    run_motor_for(
        0.0f,
        stop_time_ms
    );

    printf(
        "Final position: %.4f m\n",
        motor_get_estimated_cart_position_m()
    );
}




int main() {
    stdio_init_all();

    motor_init();

    sleep_ms(10000);

    printf(
        "\nConfiguring TMC2209 UART...\n"
    );

    bool tmc_uart_ok =
        tmc2209_init_spreadcycle();


    if (!tmc_uart_ok) {

        printf(
            "ERROR: TMC2209 UART configuration failed\n"
        );

        motor_disable();

        while (true) {
            sleep_ms(1000);
        }
    }


    printf(
        "TMC2209 UART communication successful\n"
    );

    printf(
        "SpreadCycle enabled: %s\n",
        tmc2209_is_spreadcycle_enabled()
            ? "YES"
            : "NO"
    );


    uint32_t chopconf = 0;

    if (tmc2209_read_chopconf(chopconf)) {
        uint32_t toff = chopconf & 0x0F;
        printf("CHOPCONF: 0x%08lx\n", static_cast<unsigned long>(chopconf));
        printf("TOFF: %lu\n", static_cast<unsigned long>(toff));
    }
    else {
        printf("ERROR: Could not read CHOPCONF\n");
    }


    motor_set_limits(
        20.0f,   // max velocity
        500.0f    // max acceleration
    );

    sleep_ms(2000);

    printf(
        "\ncart displacement test starting\n"
    );

    motor_enable();

    sleep_ms(100);

    printf(
        "\n--- Driver diagnostics after motor_enable() ---\n"
    );

    // Pico view of EN

    printf(
        "Pico GP4 level: %d\n",
        gpio_get(4)
    );

    // TMC actual input pins

    uint32_t ioin = 0;

    if (tmc2209_read_ioin(ioin)) {

        uint32_t tmc_enn =
            ioin & 0x01;

        uint32_t version =
            (ioin >> 24) & 0xFF;

        printf(
            "IOIN: 0x%08lx\n",
            static_cast<unsigned long>(ioin)
        );

        printf(
            "TMC ENN input: %lu (%s)\n",
            static_cast<unsigned long>(tmc_enn),
            tmc_enn == 0
                ? "ENABLED"
                : "DISABLED"
        );

        printf(
            "TMC version: 0x%02lx\n",
            static_cast<unsigned long>(version)
        );
    }
    else {
        printf(
            "ERROR: Could not read IOIN\n"
        );
    }

    // Global faults

    uint32_t gstat = 0;

    if (tmc2209_read_gstat(gstat)) {

        bool driver_error =
            (gstat & (1u << 1)) != 0;

        bool undervoltage =
            (gstat & (1u << 2)) != 0;

        printf(
            "GSTAT: 0x%08lx\n",
            static_cast<unsigned long>(gstat)
        );

        printf(
            "Driver error: %s\n",
            driver_error ? "YES" : "NO"
        );

        printf(
            "Charge-pump undervoltage: %s\n",
            undervoltage ? "YES" : "NO"
        );
    }
    else {
        printf(
            "ERROR: Could not read GSTAT\n"
        );
    }

    // Actual driver status

    uint32_t drv_status = 0;

    if (tmc2209_read_drv_status(
            drv_status
        )) {

        bool stealth =
            (drv_status & (1u << 30)) != 0;

        uint32_t cs_actual =
            (drv_status >> 16) & 0x1F;

        uint32_t short_flags =
            (drv_status >> 2) & 0x0F;

        bool overtemperature =
            (drv_status & (1u << 1)) != 0;

        printf(
            "DRV_STATUS: 0x%08lx\n",
            static_cast<unsigned long>(
                drv_status
            )
        );

        printf(
            "Actual chopper mode: %s\n",
            stealth
                ? "StealthChop"
                : "SpreadCycle"
        );

        printf(
            "CS_ACTUAL: %lu\n",
            static_cast<unsigned long>(
                cs_actual
            )
        );

        printf(
            "Short-circuit flags: 0x%lx\n",
            static_cast<unsigned long>(
                short_flags
            )
        );

        printf(
            "Overtemperature shutdown: %s\n",
            overtemperature
                ? "YES"
                : "NO"
        );
    }
    else {
        printf(
            "ERROR: Could not read DRV_STATUS\n"
        );
    }

    test_loaded_acceleration(500.0f); 

    printf("\nAcceleration test complete\n");

    motor_disable();


   /* motor_set_zero_position();

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

    while (true) {
        sleep_ms(1000);
    }
}