#pragma once

#include "pendulum/state_estimator.hpp"

//just wanna turn state vector into u

float lqr_compute_force_n(const PendulumState& state);

