#pragma once

void motor_init();

void motor_enable();
void motor_disable();

void motor_stop();

void motor_set_limits(float max_velocity_rad_s, float max_acceleration_rad_s2);

void motor_set_target_angular_velocity(float omega_rad_s);

void motor_update(float dt_seconds);

void motor_set_zero_position();

float motor_get_target_angular_velocity(); //what the controller wants
float motor_get_commanded_angular_velocity(); //what the a limiter lets us do
float motor_get_step_frequency();

float motor_get_estimated_position_rad();
float motor_get_estimated_velocity_rad_s();



bool motor_is_enabled();