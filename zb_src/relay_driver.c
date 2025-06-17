#include "driver/gpio.h"
#include "light_driver.h"

#define DATA_PIN 14
#define CLOCK_PIN 13
#define LATCH_PIN 12
#define OE_PIN 5

static uint16_t shift_state = 0;

static void write_shift_register(void)
{
    gpio_set_level(LATCH_PIN, 0);
    for (int i = 15; i >= 0; --i) {
        gpio_set_level(CLOCK_PIN, 0);
        gpio_set_level(DATA_PIN, (shift_state >> i) & 1);
        gpio_set_level(CLOCK_PIN, 1);
    }
    gpio_set_level(LATCH_PIN, 1);
}

static void set_relay(int index, bool active)
{
    if (active) shift_state |= (1 << index);
    else shift_state &= ~(1 << index);
    write_shift_register();
}

void light_driver_init(bool power)
{
    gpio_reset_pin(DATA_PIN);
    gpio_set_direction(DATA_PIN, GPIO_MODE_OUTPUT);
    gpio_reset_pin(CLOCK_PIN);
    gpio_set_direction(CLOCK_PIN, GPIO_MODE_OUTPUT);
    gpio_reset_pin(LATCH_PIN);
    gpio_set_direction(LATCH_PIN, GPIO_MODE_OUTPUT);
    gpio_reset_pin(OE_PIN);
    gpio_set_direction(OE_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(OE_PIN, 0);

    shift_state = 0;
    write_shift_register();
    light_driver_set_power(power);
}

void light_driver_set_power(bool power)
{
    set_relay(0, power); // control first zone for now
}
