#include "pendulum/encoder.hpp"
#include "pico/stdlib.h"
#include "hardware/gpio.h"

//anonymous namespace because everything here is private to this file
//only exposes the functions below the namespace
namespace {

constexpr uint ENC_A = 26;
constexpr uint ENC_B = 27;

constexpr float COUNTS_PER_REV = 2400.0f;
constexpr float TWO_PI = 6.28318530717958647692f;
constexpr float PI = 3.14159265358979323846f;

volatile int32_t encoder_count = 0;
volatile uint8_t last_state = 0;

int32_t zero_count = 0;

constexpr int8_t qtable[16] = {
    0, -1, +1, 0,
    +1, 0, 0, -1,
    -1, 0, 0, +1,
    0, +1, -1, 0
};

void encoder_callback(uint gpio, uint32_t events) {
    uint8_t a = gpio_get(ENC_A);
    uint8_t b = gpio_get(ENC_B);

    uint8_t current_state = (a << 1) | b;
    uint8_t index = (last_state << 2) | current_state;

    encoder_count += qtable[index];
    last_state = current_state;
}

}

void encoder_init() {
    gpio_init(ENC_A);
    gpio_set_dir(ENC_A, GPIO_IN);

    gpio_init(ENC_B);
    gpio_set_dir(ENC_B, GPIO_IN);

    last_state = (gpio_get(ENC_A) << 1) | gpio_get(ENC_B);

    gpio_set_irq_enabled_with_callback(
        ENC_A,
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
        true,
        &encoder_callback
    );

    gpio_set_irq_enabled(
        ENC_B,
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
        true
    );
}

int32_t encoder_get_count() {
    return encoder_count;
}

void encoder_set_zero() {
    zero_count = encoder_count;
}

float encoder_get_angle_continuous() {
    int32_t relative_count = encoder_count - zero_count; //C_rel = C_raw - C_0
    
    return relative_count * TWO_PI / COUNTS_PER_REV;
}

float encoder_get_angle_wrapped() {
    float angle = encoder_get_angle_continuous();

    while (angle >= PI) {
        angle -= TWO_PI;
    }

    while(angle < -PI) {
        angle += TWO_PI;
    }

    return angle;
}





