// file rundown:
// init UART - > readGCONF -> preserve existing bits -> set bit 6: say UART owns PDN_UART
// -> set bit 2: SpreadCycle -> write GCONF -> read back -> verify

#include "pendulum/tmc2209_uart.hpp"

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/uart.h"

#include <cstdint>

namespace {

uart_inst_t* const TMC_UART = uart1;

constexpr uint UART_TX_PIN = 8;
constexpr uint UART_RX_PIN = 9;

constexpr uint32_t UART_BAUD = 115200;

constexpr uint8_t TMC_ADDRESS = 0;


//TMC2209 register address
constexpr uint8_t REG_GCONF = 0x00;

// Chop config address
constexpr uint8_t REG_CHOPCONF = 0x6C;

//GCONF bits
constexpr uint32_t GCONF_EN_SPREADCYCLE = (1u << 2);

constexpr uint32_t GCONF_PDN_DISABLE = (1u << 6);

bool spreadcycle_enabled = false;



//CRC

uint8_t calculate_crc(const uint8_t* data, uint32_t length) {
    uint8_t crc = 0;

    for (uint32_t i = 0; i < length; i++) {
        uint8_t current_byte = data[i];

        for (uint8_t bit = 0; bit < 8; bit++) {

            if(((crc >> 7) ^ (current_byte & 0x01)) != 0) {
                crc = static_cast<uint8_t>(
                    (crc << 1) ^ 0x07
                );
            }

            else {
                crc = static_cast<uint8_t>(crc << 1);
            }

            current_byte >>= 1;
        }
    }

    return crc;
}


//flush anything currently waiting uart RX

void clear_uart_rx() {
    while (uart_is_readable(TMC_UART)) {
        uart_getc(TMC_UART);
    }
}


// write one tmc2209 register

void write_register(uint8_t register_address, uint32_t value) {
    uint8_t packet[8];

    packet[0] = 0x05;
    packet[1] = TMC_ADDRESS;

    // Bit 7 = 1 means WRITE
    packet[2] = register_address | 0x80;

    //tmc2209 will expect 32 bit register, data most significant byte first
    packet[3] = static_cast<uint8_t>( 
        (value >> 24) & 0xFF
    );
    packet[4] = static_cast<uint8_t>(
        (value >> 16) & 0xFF
    );
    packet[5] = static_cast<uint8_t>(
        (value >> 8) & 0xFF
    );
    packet[6] = static_cast<uint8_t>(
        value & 0xFF
    );

    packet[7] = calculate_crc(packet, 7);

    clear_uart_rx();

    uart_write_blocking(TMC_UART, packet, sizeof(packet));

    uart_tx_wait_blocking(TMC_UART);

}


// read one tmc2209 register

bool read_register(uint8_t register_address, uint32_t& value) {
    clear_uart_rx();

    uint8_t request[4];

    request[0] = 0x05;
    request[1] = TMC_ADDRESS;

    //bit 7 = 0 means READ
    request[2] = register_address & 0x7F;

    request[3] = calculate_crc(request, 3);

    uart_write_blocking(TMC_UART, request, sizeof(request));

    uart_tx_wait_blocking(TMC_UART);

    //tx and rx share the uart bus so pico can receive its own request
    //gather everything and look for real tmc response packet

    uint8_t buffer[20];
    
    uint32_t received = 0;

    uint64_t start_us = time_us_64();

    while (time_us_64() - start_us < 10000) {
        while (uart_is_readable(TMC_UART) && received < sizeof(buffer)) {
            buffer[received++] = uart_getc(TMC_UART);
        }
    }

    // tmc read response looks like:
    // 0x05
    // 0xFF
    // register
    // data byte 3
    // byte 2
    // byte 1
    // byte 0
    // CRC

    for (uint32_t i = 0; i + 7 < received; i++) {
        if (buffer[i] == 0x05 && buffer[i + 1] == 0xFF && (buffer[i + 2] & 0x7F) == register_address) {
            
            uint8_t expected_crc = calculate_crc(&buffer[i], 7);

            if (expected_crc != buffer[i + 7]) {
                continue;
            }

            value =
                (
                    static_cast<uint32_t>(buffer[i + 3]) << 24
                )
                |
                (
                    static_cast<uint32_t>(buffer[i + 4]) << 16
                )
                |
                (
                    static_cast<uint32_t>(buffer[i + 5]) << 8
                )
                |
                static_cast<uint32_t>(buffer[i + 6]);

            return true;
        }
    }

    return false;

}

}


// Public init

bool tmc2209_init_spreadcycle() {
    uart_init(TMC_UART, UART_BAUD);

    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);

    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    uart_set_format(TMC_UART, 8, 1, UART_PARITY_NONE);

    uart_set_fifo_enabled(TMC_UART, false);

    sleep_ms(100);


    // first read existing GCONF register
    // dont destory unrelated existing stuff

    uint32_t gconf = 0;

    if (!read_register(REG_GCONF, gconf)) {
        spreadcycle_enabled = false;
        return false;
    }


    // Tell PDN_UART to behave as UART instead of standalone power down control

    gconf |=
        GCONF_PDN_DISABLE;


    // enable SpreadCycle

    gconf |=
        GCONF_EN_SPREADCYCLE;


    write_register(REG_GCONF, gconf);

    sleep_ms(10);


    // Read back so I know the command
    // actually reached the TMC2209

    uint32_t verification_gconf = 0;

    if (!read_register(REG_GCONF,verification_gconf)) {
        spreadcycle_enabled = false;
        return false;
    }


    spreadcycle_enabled = (verification_gconf & GCONF_EN_SPREADCYCLE) != 0;

    bool pdn_uart_enabled = (verification_gconf & GCONF_PDN_DISABLE) != 0;

    return (spreadcycle_enabled && pdn_uart_enabled);
}


bool tmc2209_is_spreadcycle_enabled() {
    return spreadcycle_enabled;
}

bool tmc2209_read_chopconf(uint32_t& value) {
    return read_register(REG_CHOPCONF, value);
}

