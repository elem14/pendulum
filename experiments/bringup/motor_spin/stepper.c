#include "pico/stdlib.h"
#include <stdio.h>

#define STEP_PIN 2   // GP2
#define DIR_PIN  3   // GP3
#define EN_PIN   4   // GP4

// EN is active-low: driving it LOW enables the driver
void stepper_enable() {
    gpio_put(EN_PIN, 0);
}

void stepper_disable() {
    gpio_put(EN_PIN, 1);
}

void stepper_set_dir(bool forward) {
    gpio_put(DIR_PIN, forward ? 1 : 0);
}

// One pulse = one microstep (8 microsteps/full step, per MS1/MS2 = GND/GND)
void stepper_step_once(uint32_t pulse_width_us) {
    gpio_put(STEP_PIN, 1);
    sleep_us(pulse_width_us);
    gpio_put(STEP_PIN, 0);
    sleep_us(pulse_width_us);
}

int main() {
    stdio_init_all();
    sleep_ms(2000);   // give USB serial time to enumerate before first prints

    gpio_init(STEP_PIN);
    gpio_set_dir(STEP_PIN, GPIO_OUT);
    gpio_put(STEP_PIN, 0);

    gpio_init(DIR_PIN);
    gpio_set_dir(DIR_PIN, GPIO_OUT);

    gpio_init(EN_PIN);
    gpio_set_dir(EN_PIN, GPIO_OUT);
    stepper_disable();   // start disabled, safe default

    printf("Stepper test starting...\n");
    sleep_ms(500);

    stepper_enable();
    sleep_ms(100);   // small settle time after enabling

    while (true) {
        printf("Stepping forward 200 steps (1 full rev at full-step, or 1/8 rev at 8 microstep)\n");
        stepper_set_dir(true);
        for (int i = 0; i < 200; i++) {
            stepper_step_once(1000);   // 1000us pulse width = slow, safe starting speed
        }

        sleep_ms(1000);

        printf("Stepping backward 200 steps\n");
        stepper_set_dir(false);
        for (int i = 0; i < 200; i++) {
            stepper_step_once(1000);
        }

        sleep_ms(1000);
    }
}
