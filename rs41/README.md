# RS41 project
* Some C projects for Vaisala RS41 radiosonde with stm32f100c8 MCU
* Using `QtCreator` as IDE and `Makefile` file for compilling, 
* Developen with 'LibOpenCM3' firmware library http://libopencm3.org/
* For erase and write code is `st-flash` used https://github.com/stlink-org/stlink

## Projects
* blink - Blinking with two onboard leds using stupid delay loop
* fancyblink - Blinking with two onboard leds using timer and millis counter
* si4032_cw - ISM band CW transmitter using Si4032 onboard chip
* si4032_fifo - ISM band transmitter using Si4032 onboard chip, use FIFO mode and GFSK modulation, short packet transmission
* adc_uart - Measure MCU voltage and send it over UART
* si4032_long_packet - ISM band transmitter using Si4032 onboard chip, use FIFO mode and GFSK modulation, long packet transmission
* si4032_infinite_packet - ISM band transmitter using Si4032 onboard chip, use FIFO mode and GFSK modulation, packet length over 255 bytes, tx packet handler is off
* si4032_infinite_manchester - ISM band transmitter using Si4032 onboard chip, use FIFO mode and GFSK modulation, continuous packet transmission with Manchester coding, , tx packet handler is off
* si4032_aprs - APRS tracker, use FIFO mode and GFSK modulation
* si4032_rtl433 - Sending temperature and humidity to weather station
* si4032_marker - Channel marker, use FIFO mode with baudrate changing

## Si4032 datasheet
* Si4030/31/32 ISM TRANSMITTER - https://www.silabs.com/documents/public/data-sheets/Si4030-31-32.pdf
* Si4030/31/32 REGISTER DESCRIPTIONS - https://www.silabs.com/documents/public/application-notes/AN466.pdf
* FIFO MODE, DIRECT MODE, AND PACKET HANDLEROPERATIONFOR EZRADIOPRO - https://www.silabs.com/documents/public/application-notes/AN537.pdf
* EZRADIOPRO® PROGRAMMING GUIDE - https://www.silabs.com/documents/public/application-notes/AN415.pdf
