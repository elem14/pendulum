#pragma once

#include <cstdint>

void encoder_init();

int32_t encoder_get_count();

void encoder_set_zero();

float encoder_get_angle_continous();

float encoder_get_angle_wrapped();