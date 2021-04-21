# RS41 si4032_aprs

RS41 project with Si4032 ISM TRANSMITTER, APRS transmitter with FIFO mode
* First inicialize all components (clock, gpio, spi ...)
* Invert onboard leds state
* Default transmitting frequency is 433.125 MHz
* Output power is 1 dBm (1.26 mW)
* GFSK modulation, FIFO mode
* Transmit infinite packet (over 255 bytes), Packet Handler Disabled

## Theory
* Idea - Use Si4032 as 1bit DAC with 13200 Hz sample rate.
* Frequency of tone mark is 1200 Hz -> 13200/1200 = 11 samples per period.
* Frequency of tone space is 2200 Hz -> 13200/2200 = 6 samples per period.
* 1) Generate AFSK1200 signal internally with 13200 Hz sample rate.
* 2) Round up this sine signal to square.
* 3) Now we have binary baseband signal stream.
* 4) Convert bits into bytes.
* 5) Fill bytes into 32 bytes buffer.
* 6) If buffer is full, then write buffer into Si4032 FIFO and wait for Si4032 FIFO empty interrupt.
* 7) All packet data is transmitted in 32 bytes RAW blocks.
* No timer configuration is not needed.
* No SPI init and deinit is not needed.
* All timing is done in Si4032 radio.

## Decoder setup
* Some AX.25, APRS decoder

## Signal spectrum comparasion
### My own APRS implementation with FIFO usage
![APRS from FIFO](si4032_aprs_fifo.png?raw=true "APRS from FIFO")

### APRS implementation using MCU timers in RS41FOX, RS41HUP etc projects
![APRS with timer](si4032_aprs_timer.png?raw=true "APRS with timer")
