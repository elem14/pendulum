#include "pendulum/state_estimator.hpp"

#include "pendulum/encoder.hpp"
#include "pendulum/velocity_estimator.hpp"
#include "pendulum/motor.hpp"


namespace {

PendulumState current_state{};

float velocity_filter_time_constant_s =
    0.02f;

constexpr float CART_SIGN =
    1.0f;

constexpr float PENDULUM_SIGN =
    1.0f;

}


// init state estimator

void state_estimator_init(
    float pendulum_velocity_time_constant_s
) {

    velocity_filter_time_constant_s =
        pendulum_velocity_time_constant_s;


    encoder_init();


    velocity_estimator_init(
        velocity_filter_time_constant_s
    );


    current_state = {};
}


// current physical config defined as:
//
// x         = 0
// x_dot     = 0
// theta     = 0
// theta_dot = 0
//
// Call this while
// cart = centered 
// pendulum = upright and stationary

void state_estimator_zero() {

    encoder_set_zero();

    motor_set_zero_position();

    // Reset the velocity estimator so an old encoder difference doesn't create a fake
    // angular velocity spike after zeroing
    velocity_estimator_init(
        velocity_filter_time_constant_s
    );


    current_state = {};
}


// Update complete state vector

void state_estimator_update() {

    int32_t encoder_count =
        encoder_get_count();


    velocity_estimator_update(
        encoder_count
    );


    current_state.pendulum_angle_rad =
        PENDULUM_SIGN
        *
        encoder_get_angle_continuous();


    current_state.pendulum_angular_velocity_rad_s =
        PENDULUM_SIGN
        *
        velocity_estimator_get_filtered();


    // Cart open loop estimates

    current_state.cart_position_m =
        CART_SIGN
        *
        motor_get_estimated_cart_position_m();


    current_state.cart_velocity_m_s =
        CART_SIGN
        *
        motor_get_estimated_cart_velocity_m_s();
}


// Return latest state
PendulumState state_estimator_get() {

    return current_state;
}