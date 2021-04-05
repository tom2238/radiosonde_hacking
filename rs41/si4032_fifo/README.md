# RS41 si4032_fifo

RS41 project with Si4032 ISM TRANSMITTER, FIFO mode example.
* First inicialize all components (clock, gpio, spi ...)
* Invert onboard leds state
* Default transmitting frequency is 433.125 MHz
* Output power is 1 dBm (1.26 mW)
* GFSK modulation, FIFO mode
* Transmit packet cca every second
* Measure some voltages and MCU temperature

## Packet setup
* 1200 baud rate
* 4800 Hz deviation
* 40 nibble preamble
* 8 byte head length, 20 byte data length, 2 byte CRC length
* Decoding with https://github.com/tom2238/nrz-audio-modem

## Decoder setup
* nrz-audio-modem is used
* NRZ coding, 20 byte packet length, 1200 baud rate, STM32 print mode (-P 1)
* 'sox -t pulseaudio default -t wav - 2>/dev/null | ./decoder -i - -L 20 -b 1200 -P 1'

### UART vs radio transmission comparasion
![UART vs radio](si4032_fifo_radio_uart.png?raw=true "UART vs radio")

### Signal spectrum in Gqrx
![Gqrx spectra](si4032_fifo_frame.png?raw=true "Gqrx spectra")
