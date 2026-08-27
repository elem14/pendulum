#include "pendulum/velocity_estimator.hpp"

#include "pico/stdlib.h"

namespace {

constexpr float COUNTS_PER_REV = 2400.0f;
constexpr float TWO_PI = 6.28318530717958647692f;
constexpr float SECONDS_PER_MICROSECOND = 1.0e-6f;


int32_t previous_count = 0;
uint64_t previous_time_us = 0.0f;

float raw_velocity = 0.0f;
float filtered_velocity = 0.0f;

float filter_tau = 0.02f;

bool initialized = false;

}

void velocity_estimator_init(float time_constant_seconds) {
    filter_tau = time_constant_seconds;

    raw_velocity = 0.0f;
    filtered_velocity = 0.0f;

    initialized = false;
}

void velocity_estimator_update(int32_t current_count) {
    uint64_t current_time_us = time_us_64();

    if (!initialized) {
        previous_count = current_count;
        previous_time_us = current_time_us;
        initialized = true;
        return;
    }

    int32_t delta_count = current_count - previous_count;

    uint64_t delta_time_us = current_time_us - previous_time_us;

    float delta_time = delta_time_us * SECONDS_PER_MICROSECOND;
    
    if (delta_time <= 0.0f) {
        return;
    }

    float delta_angle = delta_count * TWO_PI / COUNTS_PER_REV;

    raw_velocity = delta_angle / delta_time;

    float alpha = delta_time / (filter_tau + delta_time);

    filtered_velocity = filtered_velocity + alpha * (raw_velocity - filtered_velocity);

    previous_count = current_count;
    previous_time_us = current_time_us;
}

float velocity_estimator_get_raw() {
    return raw_velocity;
}

float velocity_estimator_get_filtered() {
    return filtered_velocity;
}