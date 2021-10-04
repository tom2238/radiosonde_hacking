# RS41 si4032_marker

RS41 project with Si4032 ISM TRANSMITTER, Channel marker, beep, beep ...
* First inicialize all components (clock, gpio, spi ...)
* Invert onboard leds state
* Default transmitting frequency is 433.120 MHz
* GFSK modulation, FIFO mode
* Transmit infinite packet (over 255 bytes), Packet Handler Disabled
* Packet transmission still over without break

## Channel Marker
* Generating three tones
* - 1300 Hz for 520 ms, 1940 Hz for 520 ms, 2590 Hz for 1020 ms
* Every tone has its own output power level
* 1 dBm, 5 dBm and 11 dBm for 2590 Hz

### Signal spectrum in Gqrx
![Gqrx spectra](si4032_marker.png?raw=true "Gqrx spectra")
