#pragma once

#include <cstdint>

bool tmc2209_init_spreadcycle();

bool tmc2209_is_spreadcycle_enabled();

bool tmc2209_read_chopconf(uint32_t& value);
