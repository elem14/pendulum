#pragma once
#include <cstdint>

void motor_init();

void motor_enable();
void motor_disable();

void motor_stop();

void motor_move_steps_blocking(int32_t steps, uint32_t half_period_us);

void motor_set_limits(float max_velocity_rad_s, float max_acceleration_rad_s2);

void motor_set_target_angular_velocity(float omega_rad_s);

void motor_update(float dt_seconds);

void motor_set_zero_position();

float motor_get_target_angular_velocity(); //what the controller wants
float motor_get_commanded_angular_velocity(); //what the a limiter lets us do
float motor_get_step_frequency();

float motor_get_estimated_position_rad();
float motor_get_estimated_velocity_rad_s();

float motor_get_estimated_cart_position_m();
float motor_get_estimated_cart_velocity_m_s();



bool motor_is_enabled();