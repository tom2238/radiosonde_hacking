# RS41 si4032_infinite_manchester

RS41 project with Si4032 ISM TRANSMITTER, FIFO mode example with infinite manchester packet
* First inicialize all components (clock, gpio, spi ...)
* Invert onboard leds state
* Default transmitting frequency is 433.125 MHz
* Output power is 1 dBm (1.26 mW)
* GFSK modulation, FIFO mode
* Transmit infinite packet (over 255 bytes), Packet Handler Disabled
* Packet transmission still over without break
* Measure some voltages and MCU temperature
* Added watchdog timer

## Packet setup
* 2500 baud rate
* 2500 Hz deviation
* 8 byte head length, 100 byte data length, 2 byte CRC length
* Manchester coding is used: RAW 8 bytes header, 200 bytes data, 4 bytes CRC = 212 bytes total in Manchester coding
* Decoding with https://github.com/tom2238/nrz-audio-modem

## Decoder setup
* nrz-audio-modem is used
* Manchester coding (-M), 100 byte packet length, 2500 baud rate, STM32 print mode (-P 1)
* 'sox -t pulseaudio default -t wav - 2>/dev/null | ./decoder -i - -L 100 -b 2500 -P 1 -M'

### Signal spectrum in Gqrx
![Gqrx spectra](si4032_infinite_man_code.png?raw=true "Gqrx spectra")
