#include "hal/hal.h"
#include "hal_gpio_stm32l4.h"

enum {
    LOW = 0,
    HIGH,
    INVAILD
};

/*
* @ function comments
* @ input para: bool(1/0) which means led active level
* @ return para: void
*/
void board_drv_led_ctrl(uint8_t port, bool active)
{
    uint8_t gpio_set = 1;
    uint8_t gpio_reset = 0;
    gpio_dev_t gpio_led1 = {port, OUTPUT_PUSH_PULL, &gpio_set};
    if (HIGH == active)
        hal_gpio_output_low(&gpio_led1);
    if (LOW == active)
        hal_gpio_output_high(&gpio_led1);
}

aos_timer_t led_timer;
uint8_t led_port;
static void led_off(void)
{
    board_drv_led_ctrl(led_port, 0);
}

void led_notify(uint8_t port, int durations)
{
    board_drv_led_ctrl(port, 1);
    led_port = port;
    aos_timer_new(&led_timer, led_off, NULL, durations, 0);
}