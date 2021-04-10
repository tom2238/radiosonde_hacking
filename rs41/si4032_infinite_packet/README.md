# RS41 si4032_infinite_packet

RS41 project with Si4032 ISM TRANSMITTER, FIFO mode example with infinite packet
* First inicialize all components (clock, gpio, spi ...)
* Invert onboard leds state
* Default transmitting frequency is 433.125 MHz
* Output power is 1 dBm (1.26 mW)
* GFSK modulation, FIFO mode
* Transmit infinite packet (over 255 bytes), 40 bytes preamble + 320 bytes content
* Transmit packet cca every second
* Measure some voltages and MCU temperature
* Added watchdog timer

## Packet setup
* 4800 baud rate
* 2400 Hz deviation
* 8 byte head length, 310 byte data length, 2 byte CRC length
* Decoding with https://github.com/tom2238/nrz-audio-modem

## Decoder setup
* nrz-audio-modem is used
* NRZ coding, 310 byte packet length, 4800 baud rate, STM32 print mode (-P 1)
* 'sox -t pulseaudio default -t wav - 2>/dev/null | ./decoder -i - -L 310 -b 4800 -P 1'
