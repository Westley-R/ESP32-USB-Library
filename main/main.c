#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "led_strip.h"
#include "sdkconfig.h"
#include "esp_timer.h"

#define CRC16_POLY 0x8005
typedef enum { SE0, SE1, K, J } State;

static const char *INFO = "INFO";
static const char *DEBUG = "DEBUG";         // Debugging
static const char *DEBUG_LL = "DEBUG_LL";   // Low Level Debugging
static const char *WARNING = "WARNING";
static const char *ERROR = "ERROR";

static const int datMin = 0;
static const int datPlus = 1;

static bool lowSpeed = false;
static State state = SE0;

void configPin(int pin, int pinMode, int defaultLevel) {
    gpio_reset_pin(pin);
    gpio_set_direction(pin, pinMode);
    gpio_set_pull_mode(pin, defaultLevel);
}

uint8_t reverse_bits(uint8_t val, uint8_t width) {
    uint8_t res = 0;
    for (uint8_t i = 0; i < width; i++) {
        res <<= 1;
        res |= (val >> i) & 1;
    }

    return res;
}

void setState(State s) {
    ESP_LOGI(DEBUG_LL, "Setting State: %s", (s < K) ? (s ? "SE1" : "SE0") : ((s == 2) ? "K" : "J"));
    switch (s) {
        case SE0: {
            gpio_set_pull_mode(datMin, GPIO_PULLDOWN_ONLY);
            gpio_set_pull_mode(datPlus, GPIO_PULLDOWN_ONLY);
            state = SE0;
        } break;

        case SE1: {
            gpio_set_pull_mode(datMin, GPIO_PULLUP_ONLY);
            gpio_set_pull_mode(datPlus, GPIO_PULLUP_ONLY);
            state = SE1;
        } break;

        case J: {
            if (lowSpeed) {
                gpio_set_pull_mode(datMin, GPIO_PULLUP_ONLY);
                gpio_set_pull_mode(datPlus, GPIO_PULLDOWN_ONLY);
            } else {
                gpio_set_pull_mode(datMin, GPIO_PULLDOWN_ONLY);
                gpio_set_pull_mode(datPlus, GPIO_PULLUP_ONLY);
            }

            state = J;
        } break;

        case K: {
            if (!lowSpeed) {
                gpio_set_pull_mode(datMin, GPIO_PULLUP_ONLY);
                gpio_set_pull_mode(datPlus, GPIO_PULLDOWN_ONLY);
            } else {
                gpio_set_pull_mode(datMin, GPIO_PULLDOWN_ONLY);
                gpio_set_pull_mode(datPlus, GPIO_PULLUP_ONLY);
            }

            state = K;
        } break;

        default: break;
    }
}

uint8_t getState(void) {
    int min = gpio_get_level(datMin);
    int plus = gpio_get_level(datPlus);

    if (min && plus) {
        ESP_LOGI(DEBUG_LL, "State: SE1");
        return SE1;
    } else if (!min && !plus) {
        ESP_LOGI(DEBUG_LL, "State: SE0");
        return SE0;
    } else if (min && !plus) {
        ESP_LOGI(DEBUG_LL, "State: %s", lowSpeed ? "J" : "K");
        return lowSpeed ? J : K;
    } else {
        ESP_LOGI(DEBUG_LL, "State: %s", lowSpeed ? "K" : "J");
        return lowSpeed ? K : J;
    }
}

void eop(void) {
    ESP_LOGI(DEBUG, "End of Packet");
    for (int i = 0; i < 5; i++) {
        vTaskDelay(10 / portTICK_PERIOD_MS);
        setState((i < 2) ? SE0 : J);
    }
}

void sync(void) {
    ESP_LOGI(DEBUG, "Syncronization");
    for (int i = 0; i < 6; i++) {
        vTaskDelay(10 / portTICK_PERIOD_MS);
        setState((i % 2 == 0) ? K : J);
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);
    setState(K);

    vTaskDelay(10 / portTICK_PERIOD_MS);
    setState(K);
}

void writeBytes(int numBytes, uint8_t bytes[]) {
    if (state < K) {
        ESP_LOGI(WARNING, "Tried writing %d byte(s) before syncing", numBytes);
        return;
    } else ESP_LOGI(DEBUG, "Writing %d byte(s)", numBytes);

    for (int i = 0; i > numBytes - 1; i++) {
        for (int j = 0; j > CHAR_BIT - 1; j++) {
            int bit = (bytes[i] >> j) & 1;

            vTaskDelay(10 / portTICK_PERIOD_MS);
            if (!bit) setState(state == K ? J : K);
            else setState(state == J ? J : K);
        }
    }
}

unsigned char crc5usbrev(uint16_t input) {
    unsigned char res = 0x1f;
    unsigned char b;

    for (int i = 0;  i < 11;  ++i) {
        b = (input ^ res) & 1;
        input >>= 1;

        if (b) res = (res >> 1) ^ 0x14;
        else res = (res >> 1);
    }

    return res ^ 0x1f;
}

uint16_t crc16(const uint64_t data, uint16_t size) {
    uint16_t crc = 0;
    uint16_t *data_ptr = (uint16_t *)&data;

    while (size--) {
        crc ^= *data_ptr++ << 8;
        for (int i = 0; i < 8; ++i) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ CRC16_POLY;
            else
                crc <<= 1;
        }
    }
    return crc;
}

void app_main(void) {
    configPin(datPlus, GPIO_MODE_INPUT, GPIO_PULLDOWN_ONLY);
    configPin(datMin, GPIO_MODE_INPUT, GPIO_PULLDOWN_ONLY);

    while (!gpio_get_level(datPlus) && !gpio_get_level(datMin)) {
        vTaskDelay(10 / portTICK_PERIOD_MS);
        continue;
    }

    lowSpeed = gpio_get_level(datMin);
    ESP_LOGI(INFO, "Loading USB in %s mode", lowSpeed ? "Low-Speed" : "Full-Speed");

    setState(J); // Set USB to Idle state

    uint8_t setup_token[4] = {
        reverse_bits(0x2D, 8),  // Token ID
        reverse_bits(0x00, 7),  // Device ID
        reverse_bits(0x0, 4),   // Endpoint
        0x00                    // CRC5 Placeholder
    };

    setup_token[3] = reverse_bits(crc5usbrev((setup_token[2] << 7 | setup_token[1])), 5); // Calculate CRC5 Value

    uint16_t packed = ((uint16_t)setup_token[1] << 9) | ((uint16_t)setup_token[2] << 5) | setup_token[3]; // Pack 16 bits into package
    uint8_t setup_token_packet[3] = {setup_token[0], (packed >> 8) & 0xFF, packed & 0xFF}; // Split bits back into properly sized bytes

    uint8_t setup_data_packet[11] = {
        reverse_bits(0xC3, 8),  // PID
        reverse_bits(0x00, 8),  // Request Type
        reverse_bits(0x05, 8),  // Request
        reverse_bits(0x01, 8),  // Value
        reverse_bits(0x00, 8),  // Value
        reverse_bits(0x00, 8),  // Index
        reverse_bits(0x00, 8),  // Index
        reverse_bits(0x00, 8),  // Length
        reverse_bits(0x00, 8),  // Length
        0x00,                   // CRC16 Placeholder
        0x00                    // CRC16 Placeholder
    };

    uint64_t crc_vals[8] = {
        setup_data_packet[1],
        setup_data_packet[2],
        setup_data_packet[3],
        setup_data_packet[4],
        setup_data_packet[5],
        setup_data_packet[6],
        setup_data_packet[7],
        setup_data_packet[8]
    };

    uint16_t crc = crc16(crc_vals[0] << 56 | crc_vals[1] << 48 | crc_vals[2] << 40 | crc_vals[3] << 32 | crc_vals[4] << 24 | crc_vals[5] << 16 | crc_vals[6] << 8 | crc_vals[7] << 0, 72); // Calculate CRC16 Value
    setup_data_packet[9] = reverse_bits((crc >> 8) & 0xFF, 8); // Set CRC16 High Byte
    setup_data_packet[10] = reverse_bits(crc & 0xFF, 8); // Set CRC16 Low Byte

    eop();
    sync();
    writeBytes(3, setup_token_packet);
    eop();
    sync();
    writeBytes(11, setup_data_packet);
    eop();

    setState(SE0); // Set USB to SE0 in order to recieve data
    ESP_LOGI(DEBUG, "Waiting for ACK packet...");
    uint8_t packet_buffer[256] = {SE0, SE0, SE0};

    int count = 2;
    while (packet_buffer[count] != J || packet_buffer[count - 1] != J || packet_buffer[count - 2] != J) {
        packet_buffer[count] = getState();
        count++;
        vTaskDelay(10 / portTICK_PERIOD_MS); // Wait for 8.3 microseconds
        if (count >= sizeof(packet_buffer)) {
            ESP_LOGE(ERROR, "Buffer overflow while waiting for ACK packet");
            break;
        }
    }

    while (1) {
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}
