#include "board.h"

#include "driver/gpio.h"
#include "esp_check.h"

static const char *TAG = "board";

const board_axis_pins_t board_axis_pins[BOARD_NUM_AXES] = {
    {.step = 25, .dir = 33}, /* J1 base */
    {.step = 26, .dir = 32}, /* J2 shoulder */
    {.step = 27, .dir = 14}, /* J3 elbow */
    {.step = 19, .dir = 18}, /* J4 wrist pitch */
    {.step = 23, .dir = 4},  /* J5 wrist roll */
};

esp_err_t board_init(void)
{
    /* Drive EN high (drivers off) before the pin becomes an output, so it never glitches low. */
    gpio_set_level(BOARD_PIN_DRIVER_EN, 1);
    gpio_set_level(BOARD_PIN_LED, 0);
    const gpio_config_t io = {
        .pin_bit_mask = (1ull << BOARD_PIN_DRIVER_EN) | (1ull << BOARD_PIN_LED),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&io), TAG, "gpio_config");
    board_drivers_enable(false);
    return ESP_OK;
}

void board_drivers_enable(bool enable)
{
    gpio_set_level(BOARD_PIN_DRIVER_EN, enable ? 0 : 1);
}

void board_led_set(bool on)
{
    gpio_set_level(BOARD_PIN_LED, on ? 1 : 0);
}
