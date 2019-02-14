
LED_RED_1, LED_RED_2, LED_RED_3

/*
* @ function comments
* stm32l476rg-nucleo privide one LED for users to control,
* here is the interface to turn on/off this LED inside the board
* @ input para: bool(1/0) which means gpio active level
* @ return para: void
*/
void board_drv_led_ctrl(uint8_t port, bool active);

void led_notify(uint8_t port, int durations);