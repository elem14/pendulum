#include "pico/stdlib.h"

#include "pendulum/velocity_estimator.hpp"
#include "pendulum/encoder.hpp"

#include <stdio.h>

int main() {
    stdio_init_all();

    encoder_init();
    velocity_estimator_init(0.02f);

    sleep_ms(2000);

    encoder_set_zero();
    
    while (true) {
        int32_t count = encoder_get_count();
        float continuous_angle = encoder_get_angle_continuous();
        float wrapped_angle = encoder_get_angle_wrapped();

        printf(
            "Count: %ld | Continuous: %.4f rad | Wrapped: %.4f rad\n",
            count,
            continuous_angle,
            wrapped_angle
        );

        sleep_ms(200);
    }
}