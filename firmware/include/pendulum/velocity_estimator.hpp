#pragma once

#include <cstdint>

void velocity_estimator_init(float time_constant_seconds);

void velocity_estimator_update(int32_t current_count);

float velocity_estimator_get_raw();

float velocity_estimator_get_filtered();


