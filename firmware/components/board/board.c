#include "board.h"

#include "driver/gpio.h"
#include "esp_check.h"

static const char *TAG = "board";

/* Joint -> driver socket on the PCB. Change here if the motors are wired to other sockets. */
const board_axis_pins_t board_axis_pins[BOARD_NUM_AXES] = {
    {.step = 26, .dir = 27}, /* J1 base         <- U1 (motor H1) */
    {.step = 12, .dir = 13}, /* J2 shoulder     <- U2 (motor H2); GPIO12 is a strapping pin, see REVIEW.md */
    {.step = 32, .dir = 33}, /* J3 elbow        <- U3 (motor H3) */
    {.step = 4, .dir = 16},  /* J4 wrist pitch  <- U4 (motor H4) */
    {.step = 18, .dir = 19}, /* J5 wrist roll   <- U5 (motor H5) */
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
