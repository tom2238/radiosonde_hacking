# RS41 project
* Some C projects for Vaisala RS41 radiosonde with stm32f100c8 MCU
* Using `QtCreator` as IDE and `Makefile` file for compilling, 
* Developen with 'LibOpenCM3' firmware library http://libopencm3.org/
* For erase and write code is `st-flash` used

## Projects
* blink - Blinking with two onboard leds using stupid delay loop
* fancyblink - Blinking with two onboard leds using timer and millis counter
* si4032_cw - ISM band CW transmitter using Si4032 onboard chip
* si4032_fifo - ISM band transmitter using Si4032 onboard chip, use FIFO mode and GFSK modulation
