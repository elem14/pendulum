#pragma once

#include <cstdint>

void motor_init();

void motor_enable();
void motor_disable();

void motor_set_direction(bool forward);

void motor_step_once(uint32_t half_period_us);

void motor_move_steps(int32_t steps, uint32_t half_period_us);