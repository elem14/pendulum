#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include <stdio.h>

#define ENC_A 26   // GP26, pin 31
#define ENC_B 27   // GP27, pin 32

//volatile because the variable can change unexpectedly. Always read from
//memory not a stale cache
volatile int32_t encoder_count = 0;
volatile uint8_t last_state = 0;

// Quadrature state table: indexed by (last_AB << 2 | current_AB)
//lookuptable for transition values. 
static const int8_t qtable[16] = {
     0, -1, +1,  0,
    +1,  0,  0, -1,
    -1,  0,  0, +1,
     0, +1, -1,  0
};

//for interrupts, callback rereads A and B when an encoder edge occurs
void encoder_callback(uint gpio, uint32_t events) {
    uint8_t a = gpio_get(ENC_A);
    uint8_t b = gpio_get(ENC_B);
    //a=1, a << 1 = bitshift left = decimal 2, b=0, current state = 2, b=1, current state = 3, a=0, b=1, current state = 1, a=0, b=0, current state = 0 <-- does so through the a << 1 | b, it packs two bool signals into one 2-bit number AB
    uint8_t current_state = (a << 1) | b; 
    //need to rep previous AB and current AB as a 4-bit #. the 4 bit made by the or statement corresponds to the correct transition in the qtable.
    uint8_t index = (last_state << 2) | current_state;
    encoder_count += qtable[index];
    last_state = current_state;
}

int main() {
    stdio_init_all();

    gpio_init(ENC_A);
    gpio_set_dir(ENC_A, GPIO_IN); //makes GP26 an input pin, so it can read the encoder A signal

    gpio_init(ENC_B);
    gpio_set_dir(ENC_B, GPIO_IN); //makes GP27 an input pin, so it can read the encoder B signal

    // external 10k pull-ups already in place, no internal pull-up needed

    //reads actual instead of assuming last_state on boot is 00
    last_state = (gpio_get(ENC_A) << 1) | gpio_get(ENC_B);
    //configure interrupts on gpio26, trigger interrupt on rising & falling edges, enable the interrupt, by calling encoder_callback. 
    gpio_set_irq_enabled_with_callback(ENC_A, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true/*enable interrupt*/, &encoder_callback);
    //doesn't have callback because the interrupt system uses the callback that's already registered
    gpio_set_irq_enabled(ENC_B, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);

    while (true) {
        printf("Count: %d\n", encoder_count);
        sleep_ms(200);
    }
}
