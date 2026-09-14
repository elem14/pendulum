#include "pico/stdlib.h"

#include "pendulum/encoder.hpp"

#include <stdio.h>


int main() {

    stdio_init_all();

    encoder_init();

    // Give USB serial time to connect.
    sleep_ms(2000);


    printf("\n=== Encoder angle test ===\n");

    printf(
        "Hold the pendulum at the position "
        "you want to call zero.\n"
    );

    printf("Zeroing in 3 seconds...\n");

    sleep_ms(3000);


    encoder_set_zero();

    printf("Encoder zeroed.\n\n");


    while (true) {

        int32_t count =
            encoder_get_count();

        float angle_continuous =
            encoder_get_angle_continuous();

        float angle_wrapped =
            encoder_get_angle_wrapped();


        printf(
            "count: %ld | "
            "continuous: %+8.4f rad | "
            "wrapped: %+8.4f rad\n",

            static_cast<long>(count),

            angle_continuous,

            angle_wrapped
        );


        sleep_ms(100);
    }
}