blink.c just makes the Pico blink

encoder.c turns on the encoder and displays encoder values to the connected computer on a terminal
encoder values be seen /dev/tty* (one of your tty terminals) at 115200 baud
Example: /usr/bin/screen tty.usbmodem3101 115200

stepper.c turns on the stepper motor turning it forward, then backward and forward repeatedly
