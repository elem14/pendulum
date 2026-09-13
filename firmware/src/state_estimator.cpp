#include "pendulum/state_estimator.hpp"

#include "pendulum/encoder.hpp"
#include "pendulum/velocity_estimator.hpp"
#include "pendulum/motor.hpp"

namespace {

PendulumState current_state{};

float velocity_filter_time_constant_s = 0.02f;

// Signs: +x = positive cart direction/right
//        +theta = pendulum tilting toward +x

constexpr float CART_SIGN = 1.0f;

constexpr float PENDULUM_SIGN = 1.0f;

}


void state_estimator_init(float pendulum_velocity_time_constant_s) {
    velocity_filter_time_constant_s = pendulum_velocity_time_constant_s;

    encoder_init();

    velocity_estimator_init(velocity_filter_time_constant_s);

    current_state = {};
}

void state_estimator_zero() {
    encoder_set_zero();

    motor_set_zero_position();

    //reset velocity estimator too
    velocity_estimator_init(velocity_filter_time_constant_s);

    current_state = {};
}

void state_estimator_update() {
    int32_t encoder_count = encoder_get_count();
    velocity_estimator_update(encoder_count);

    current_state.cart_position_m = CART_SIGN * motor_get_estimated_cart_position_m();
    current_state.cart_velocity_m_s = CART_SIGN * motor_get_estimated_cart_velocity_m_s();
    current_state.pendulum_angle_rad = PENDULUM_SIGN * encoder_get_angle_continuous();
    current_state.pendulum_angular_velocity_rad_s = PENDULUM_SIGN * velocity_estimator_get_filtered();
}

PendulumState state_estimator_get() {
    return current_state;
}

