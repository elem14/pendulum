#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include <stdio.h>

#define ENC_A 26   // GP26, pin 31
#define ENC_B 27   // GP27, pin 32

volatile int32_t encoder_count = 0;
volatile uint8_t last_state = 0;

// Quadrature state table: indexed by (last_AB << 2 | current_AB)
static const int8_t qtable[16] = {
     0, -1, +1,  0,
    +1,  0,  0, -1,
    -1,  0,  0, +1,
     0, +1, -1,  0
};

void encoder_callback(uint gpio, uint32_t events) {
    uint8_t a = gpio_get(ENC_A);
    uint8_t b = gpio_get(ENC_B);
    uint8_t current_state = (a << 1) | b;
    uint8_t index = (last_state << 2) | current_state;
    encoder_count += qtable[index];
    last_state = current_state;
}

int main() {
    stdio_init_all();

    gpio_init(ENC_A);
    gpio_set_dir(ENC_A, GPIO_IN);

    gpio_init(ENC_B);
    gpio_set_dir(ENC_B, GPIO_IN);

    // external 10k pull-ups already in place, no internal pull-up needed

    last_state = (gpio_get(ENC_A) << 1) | gpio_get(ENC_B);

    gpio_set_irq_enabled_with_callback(ENC_A, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &encoder_callback);
    gpio_set_irq_enabled(ENC_B, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);

    while (true) {
        printf("Count: %d\n", encoder_count);
        sleep_ms(200);
    }
}
