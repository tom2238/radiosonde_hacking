# RS41 si4032_long_packet

RS41 project with Si4032 ISM TRANSMITTER, FIFO mode example with long packet
* First inicialize all components (clock, gpio, spi ...)
* Invert onboard leds state
* Default transmitting frequency is 433.125 MHz
* Output power is 1 dBm (1.26 mW)
* GFSK modulation, FIFO mode
* Transmit long packet (over 64 bytes), 128 bytes packet size
* Transmit packet cca every second
* Measure some voltages and MCU temperature
* Added watchdog timer (because freezing in loop :-( )

## Packet setup
* 4800 baud rate
* 2400 Hz deviation
* 80 nibble preamble
* 8 byte head length, 126 byte data length, 2 byte CRC length
* Decoding with https://github.com/tom2238/nrz-audio-modem

## Decoder setup
* nrz-audio-modem is used
* NRZ coding, 126 byte packet length, 4800 baud rate, STM32 print mode (-P 1)
* 'sox -t pulseaudio default -t wav - 2>/dev/null | ./decoder -i - -L 126 -b 4800 -P 1'

## TODO
* Fix while loop freezing
```
// Wait for empty FIFO
while(!Si4032_IsFIFOEmpty()); // Sometimes freeze in loop, add some timeout !!
```

### Signal spectrum in Gqrx
![Gqrx spectra](si4032_fifo_long_frame.png?raw=true "Gqrx spectra")
