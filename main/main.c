#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "led_strip.h"
#include "sdkconfig.h"

typedef enum { SE0, SE1, K, J } State;

static const char *INFO = "INFO";
static const char *DEBUG = "DEBUG";
static const char *WARNING = "WARNING";
// static const char *ERROR = "ERROR";

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
    ESP_LOGI(DEBUG, "Setting State: %s", (s < K) ? (s ? "SE1" : "SE0") : ((s == 2) ? "K" : "J"));
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

    //                                      Token ID               Device ID             End Pnt  CRC5
    uint8_t setup_packet[4] = {reverse_bits(0x2D, 8), reverse_bits(0x1, 7), reverse_bits(0x0, 4), 0x0};
    setup_packet[3] = reverse_bits(crc5usbrev((setup_packet[2] << 7 | setup_packet[1])), 5); // Calculate CRC5 Value

    uint16_t packed = ((uint16_t)setup_packet[1] << 9) | ((uint16_t)setup_packet[2] << 5) | setup_packet[3]; // Pack 16 bits into package
    uint8_t packet[3] = {setup_packet[0], (packed >> 8) & 0xFF, packed & 0xFF}; // Split bits back into properly sized bytes

    uint8_t setup_transaction[2] = {0b11000011};

    eop();
    sync();
    writeBytes(3, packet);
    eop();

    while (1) {
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}
