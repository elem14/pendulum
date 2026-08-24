#include "pico/stdlib.h"

#define LED_PIN     PICO_DEFAULT_LED_PIN   // onboard LED (GPIO 25 on most Pico boards)
#define BUTTON_PIN  15                     // change to whatever GPIO you wire your switch to

int main() {
    // LED setup
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    // Button setup (active-low, using internal pull-up)
    gpio_init(BUTTON_PIN);
    gpio_set_dir(BUTTON_PIN, GPIO_IN);
    gpio_pull_up(BUTTON_PIN);

    while (true) {
        bool pressed = !gpio_get(BUTTON_PIN);  // pressed == pulled low

        if (pressed) {
            gpio_put(LED_PIN, 1);   // solid on while held
        } else {
            gpio_put(LED_PIN, 1);
            sleep_ms(100);
            gpio_put(LED_PIN, 0);
            sleep_ms(400);          // slow blink when idle
        }
    }
}

