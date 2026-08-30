#include "pendulum/motor.hpp"

#include "pico/stdlib.h"

#include "hardware/clocks.h"
#include "hardware/pwm.h"

#include <cmath>

namespace {

constexpr uint STEP_PIN = 2;
constexpr uint DIR_PIN = 3;
constexpr uint EN_PIN = 4;

constexpr float STEPS_PER_REV = 1600.0f;

constexpr float TWO_PI = 6.28318530717958647692f;

//slow pwm counter way way down from the sys clock
//clock sys 125 MHz -- 125 / 250 = 500 kHz counterclock
constexpr float PWM_CLKDIV = 250.0f; //Clock D

//rpi pwm counter = 16 bit so max period = 65536 counter ticks
constexpr uint32_t MAX_PERIOD_COUNTS = 65536;

constexpr uint32_t MIN_PERIOD_COUNTS = 2; //need high and low

constexpr uint32_t DIR_SETTLE_US = 5; //time for DIR to settle before STEP starts

//starting limits - optimize later
float max_velocity_rad_s = 6.0f;
float max_acceleration_rad_s2 = 10.0f;

float target_velocity_rad_s = 0.0f;
float commanded_velocity_rad_s = 0.0f;
float actual_step_frequency_hz = 0.0f;
float estimated_position_rad = 0.0f;
float estimated_velocity_rad_s = 0.0f;

bool driver_enabled = false;
bool step_signal_running = false;

bool current_direction_forward = true;

uint pwm_slice = 0;
uint pwm_channel = 0;

float pwm_counter_frequency_hz = 0.0f;

float clamp_float(float value, float minimum, float maximum) {
    if (value < minimum) {
        return minimum;
    }

    if (value > maximum) {
        return maximum;
    }

    return value;
}

//forward angularv to fstep relation
float velocity_to_step_frequency(float omega_rad_s) {
    return (std::fabs(omega_rad_s) * STEPS_PER_REV / TWO_PI);
}

float step_frequency_to_velocity(float frequency_hz, bool forward) {
    float velocity_magnitude = frequency_hz * TWO_PI / STEPS_PER_REV;

    return forward ? velocity_magnitude : -velocity_magnitude; //restore sign
}

void stop_step_signal() {
    //stop pwm count
    pwm_set_enabled(pwm_slice, false);

    //gives STEP back to normal gpio control
    gpio_set_function(STEP_PIN, GPIO_FUNC_SIO);

    gpio_set_dir(STEP_PIN, GPIO_OUT);

    //guarantees STEP is on LOW at stop
    gpio_put(STEP_PIN, 0);
    step_signal_running = false;
    actual_step_frequency_hz = 0.0f;
}

bool configure_step_frequency(float requested_frequency_hz) {
    if(requested_frequency_hz <= 0.0f) {
        return false;
    }

    float period_counts_float = pwm_counter_frequency_hz / requested_frequency_hz;

    //since too slow
    if (period_counts_float > static_cast<float>(MAX_PERIOD_COUNTS)) {
        return false;
    }

    //clamp on too high of freqs
    if (period_counts_float < static_cast<float>(MIN_PERIOD_COUNTS)) {
        period_counts_float = static_cast<float>(MIN_PERIOD_COUNTS);
    }

    uint32_t period_counts = static_cast<uint32_t>(std::round(period_counts_float));

    if (period_counts < MIN_PERIOD_COUNTS) {
        period_counts = MIN_PERIOD_COUNTS;
    }

    if (period_counts > MAX_PERIOD_COUNTS) {
        period_counts = MAX_PERIOD_COUNTS;
    }

    uint16_t wrap = static_cast<uint16_t>(period_counts - 1);
    uint16_t level = static_cast<uint16_t>(period_counts / 2);

    pwm_set_wrap(pwm_slice, wrap);
    pwm_set_chan_level(pwm_slice, pwm_channel, level);

    actual_step_frequency_hz = pwm_counter_frequency_hz / static_cast<float>(period_counts);
     
    return true;
}

void start_or_update_step_signal(float omega_rad_s) {
    float requested_frequency_hz = velocity_to_step_frequency(omega_rad_s);

    //if requested velocity is to tiny then stop
    float minimum_frequency_hz = pwm_counter_frequency_hz / static_cast<float>(MAX_PERIOD_COUNTS);

    if (requested_frequency_hz < minimum_frequency_hz) {
        stop_step_signal();
        return;
    }

    bool forward = omega_rad_s > 0.0f;

    //if already in same dir, change pwm freq.
    //can keep pulsing if executing other stuff
    if (step_signal_running && forward == current_direction_forward) {
        configure_step_frequency(requested_frequency_hz);
        return;
    }

    //starting from rest or changing dir -> stop STEP first so DIR doesn;t change in the middle of active pulsing
    stop_step_signal();

    current_direction_forward = forward;

    gpio_put(DIR_PIN, forward ? 1: 0);

    //let DIR settle
    sleep_us(DIR_SETTLE_US);

    if (!configure_step_frequency(requested_frequency_hz)) {
        return;
    }

    //start each new pulse train at counter = 0
    pwm_set_counter(pwm_slice, 0);

    //give physical gp2 to pwm peripheral
    gpio_set_function(STEP_PIN, GPIO_FUNC_PWM);

    pwm_set_enabled(pwm_slice, true);

    step_signal_running = true;
}

void apply_commanded_velocity() {
    if (!driver_enabled) {
        stop_step_signal();
        return;
    }

    if (std::fabs(commanded_velocity_rad_s) < 0.001f) {
        stop_step_signal();
        return;
    }

    start_or_update_step_signal(commanded_velocity_rad_s);
}

} //namespace




void motor_init() {
    //dir
    gpio_init(DIR_PIN);
    gpio_set_dir(DIR_PIN, GPIO_OUT);
    gpio_put(DIR_PIN, 0);

    //enable
    gpio_init(EN_PIN);
    gpio_set_dir(EN_PIN, GPIO_OUT);

    //active low enable, high = disabled
    gpio_put(EN_PIN, 1);

    //step
    gpio_init(STEP_PIN);
    gpio_set_dir(STEP_PIN, GPIO_OUT);
    gpio_put(STEP_PIN, 0);

    //choose gp2 pwm hardware
    pwm_slice = pwm_gpio_to_slice_num(STEP_PIN);

    pwm_channel = pwm_gpio_to_channel(STEP_PIN);

    //read REAL pico sys clock
    float system_clock_hz = static_cast<float>(clock_get_hz(clk_sys));

    pwm_counter_frequency_hz = system_clock_hz / PWM_CLKDIV;

    //create initial pwm config
    pwm_config config = pwm_get_default_config();

    pwm_config_set_clkdiv(&config, PWM_CLKDIV);

    //random safe initial TOP
    pwm_config_set_wrap(&config, 999);

    pwm_init(pwm_slice, &config, false);

    pwm_set_chan_level(pwm_slice, pwm_channel, 0);

    target_velocity_rad_s = 0.0f;
    commanded_velocity_rad_s = 0.0f;
    actual_step_frequency_hz = 0.0f;
    estimated_position_rad = 0.0f;
    estimated_velocity_rad_s = 0.0f;
    driver_enabled = false;
    step_signal_running = false;
}

void motor_enable() {
    //tmc2209 EN is active-low
    gpio_put(EN_PIN, 0);

    driver_enabled = true;
}

void motor_disable() {
    target_velocity_rad_s = 0.0f;
    commanded_velocity_rad_s = 0.0f;

    stop_step_signal();

    estimated_velocity_rad_s = 0.0f;

    //HIGH disables tmc outputs
    gpio_put(EN_PIN, 1);
    driver_enabled = false;
}

void motor_stop() {
    //immediate STEP stop
    //keeps driver enabled so motor keeps holding torque
    target_velocity_rad_s = 0.0f;
    commanded_velocity_rad_s = 0.0f;

    stop_step_signal();

    estimated_velocity_rad_s = 0.0f;
}

void motor_set_limits(float new_max_velocity_rad_s, float new_max_acceleration_rad_s2) {
    if (new_max_velocity_rad_s > 0.0f) {
        max_velocity_rad_s = new_max_velocity_rad_s;
    }

    if(new_max_acceleration_rad_s2 > 0.0f) {
        max_acceleration_rad_s2 = new_max_acceleration_rad_s2;
    }

    target_velocity_rad_s = clamp_float(target_velocity_rad_s, -max_velocity_rad_s, max_velocity_rad_s);

    commanded_velocity_rad_s = clamp_float(commanded_velocity_rad_s, -max_velocity_rad_s, max_velocity_rad_s);
}

void motor_set_target_angular_velocity(float omega_rad_s) {
    target_velocity_rad_s = clamp_float(omega_rad_s, -max_velocity_rad_s, max_velocity_rad_s);
}

void motor_update(float dt_seconds) {
    if (!driver_enabled) {
        return;
    }

    if (dt_seconds <= 0.0f) {
        return;
    }

    float desired_velocity = target_velocity_rad_s;

    //if reveresing direction, first ramp to zero, then accelerate into oppo dir
    if (commanded_velocity_rad_s > 0.0f && target_velocity_rad_s < 0.0f) {
        desired_velocity = 0.0f;
    }

    if (commanded_velocity_rad_s < 0.0f && target_velocity_rad_s > 0.0f) {
        desired_velocity = 0.0f;
    }

    
    float velocity_error = desired_velocity - commanded_velocity_rad_s;

    float max_velocity_change = max_acceleration_rad_s2 * dt_seconds;


    if (std::fabs(velocity_error) <= max_velocity_change) {
        commanded_velocity_rad_s = desired_velocity;
    }
    else if (velocity_error > 0.0f) {
        commanded_velocity_rad_s += max_velocity_change;
    }
    else {
        commanded_velocity_rad_s -= max_velocity_change;
    }

    //clean floating point tiny residue
    if (std::fabs(commanded_velocity_rad_s) < 0.0001f) {
        commanded_velocity_rad_s = 0.0f;
    }

    float previous_estimated_velocity_rad_s = estimated_velocity_rad_s;

    apply_commanded_velocity();

    if (step_signal_running) {
        estimated_velocity_rad_s = step_frequency_to_velocity(actual_step_frequency_hz, current_direction_forward);
    }
    else {
        estimated_velocity_rad_s = 0.0f;
    }

    //numerical integration trapezoid rule
    float average_velocity_rad_s = 0.5f * (previous_estimated_velocity_rad_s + estimated_velocity_rad_s);

    estimated_position_rad += average_velocity_rad_s * dt_seconds;

}

float motor_get_target_angular_velocity() {
    return target_velocity_rad_s;
}

float motor_get_commanded_angular_velocity() {
    return commanded_velocity_rad_s;
}

float motor_get_step_frequency() {
    return actual_step_frequency_hz;
}

void motor_set_zero_position() {
    estimated_position_rad = 0.0f;
}

float motor_get_estimated_position_rad() {
    return estimated_position_rad;
}

float motor_get_estimated_velocity_rad_s() {
    return estimated_velocity_rad_s;
}

bool motor_is_enabled() {
    return driver_enabled;
}

