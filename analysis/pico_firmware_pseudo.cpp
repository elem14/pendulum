// all PC side 
{/*
        constexpr float K[4] = {-13.33f, -10.78f, -39.79f, -5.14f};
        constexpr float m_p = /* measured */, l_c = /* measured */, I_p = /* measured */, g = 9.81f;
        constexpr float E_target = m_p * g * l_c;
        constexpr float THETA_CAPTURE = 0.2f, THETADOT_CAPTURE = 1.0f;

        enum class Mode { SWING_UP, BALANCE };
        Mode mode = Mode::SWING_UP;

        // runs every control tick ( eg 500 Hz off a hardware timer) 
        float control_step(State s /* {x, x_dot, theta, theta_dot}, from sensors */) {
            bool in_capture_region = std::abs(s.theta) < THETA_CAPTURE
                                    && std::abs(s.theta_dot) < THETADOT_CAPTURE;
            if (in_capture_region) mode = Mode::BALANCE;
            // (no BALANCE -> SWING_UP transition yet, farther tuning decision)

            if (mode == Mode::BALANCE) {
                return -(K[0]*s.x + K[1]*s.x_dot + K[2]*s.theta + K[3]*s.theta_dot);
            } else {
                // energy-based swing-up -- NOT yet derived in the doc. Sketch of the shape it'll take, not the final law:
                float T = 0.5f*(m_c+m_p)*s.x_dot*s.x_dot
                        + m_p*l_c*std::cos(s.theta)*s.x_dot*s.theta_dot
                        + 0.5f*I_p*s.theta_dot*s.theta_dot;
                float V = m_p*g*l_c*std::cos(s.theta);
                float E = T + V;
                float E_error = E_target - E;
                return /* some k_swing * E_error * (sign related to theta_dot*cos(theta)) */;
            }
        }
*/}

