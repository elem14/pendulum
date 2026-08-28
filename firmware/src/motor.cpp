#include "pendulum/motor.hpp"

#include "pico/stdlib.h"

namespace {

    constexpr uint STEP_PIN = 2;
    constexpr uint DIR_PIN = 3;
    constexpr uint EN_PIN = 4;

}

void motor_init() {
    gpio_init(STEP_PIN);
    gpio_set_dir(STEP_PIN, GPIO_OUT);
    gpio_put(STEP_PIN, 0);

    gpio_init(DIR_PIN);
    gpio_set_dir(DIR_PIN, GPIO_OUT);
    gpio_put(DIR_PIN, 0);

    gpio_init(EN_PIN);
    gpio_set_dir(EN_PIN, GPIO_OUT);

    motor_disable();
}

void motor_enable() {
    gpio_put(EN_PIN, 0);
}

void motor_disable() {
    gpio_put(EN_PIN, 1);
}

void motor_set_direction(bool forward) {
    gpio_put(DIR_PIN, forward ? 1 : 0);
}

void motor_step_once(uint32_t half_period_us) {
    gpio_put(STEP_PIN, 1);
    sleep_us(half_period_us);

    gpio_put(STEP_PIN, 0);
    sleep_us(half_period_us);
}

//can move the motor n-steps where n>0 => forward & n<0 => reverse, |n| = # of step pulses
void motor_move_steps(int32_t steps, uint32_t half_period_us) {
    if (steps == 0) {
        return;
    }

    bool forward = steps > 0;
    motor_set_direction(forward);

    uint32_t step_count = steps > 0 ? static_cast<uint32_t>(steps) : static_cast<uint32_t>(-steps);

    for (uint32_t i = 0; i < step_count; i++) {
        motor_step_once(half_period_us);
    }
} 