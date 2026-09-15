#pragma once


struct PendulumState {
    float cart_position_m;
    float cart_velocity_m_s;
    float pendulum_angle_rad;
    float pendulum_angular_velocity_rad_s;
};


void state_estimator_init(
    float pendulum_velocity_time_constant_s
);

void state_estimator_zero();

void state_estimator_update();

PendulumState state_estimator_get();