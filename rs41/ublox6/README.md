# RS41 Ublox G6010-ST GPS

RS41 project with Si4032 and Ublox G6010 GPS
* Blinking with onboard green led
* Send C to turn on red led or send S to turn off red led (on XDATA UASRT)
* XDATA USART3 baud rate is 38400 bps
* GPS USART1 baud rate is 38400 bps
* Default transmitting frequency is 433.125 MHz
* Output power is 1 dBm (1.26 mW)
* GFSK modulation, FIFO mode
* Transmit short packet (<= 64 bytes)
* Send radio packet every second

## GPS configuration
* Baud rate is 38400 bps, using binary UBX protocol
* 1 Hz navigation rate
* NAV-SOL, NAV-TIMEUTC, NAV-POSLLH and NAV-VELNED messages are received.

## Serial print
* Printing some GPS data on XDATA USART
* Baud rate is 38400 bps, 8N1
* Eg. Time only fix with bad position data:
```
2021.9.21. 18:25:22, Fix:0, SVs:0, Lat:552401552, Lon:-268201759, Alt:14331509, H:0, gS:0, S:0
2021.9.21. 18:25:24, Fix:0, SVs:0, Lat:552401552, Lon:-268201759, Alt:14331509, H:0, gS:0, S:0
2021.9.21. 18:25:25, Fix:0, SVs:0, Lat:552401552, Lon:-268201759, Alt:14331509, H:0, gS:0, S:0
2021.9.21. 18:25:26, Fix:0, SVs:0, Lat:552401552, Lon:-268201759, Alt:14331509, H:0, gS:0, S:0
```

## Radio packet
* Frequency is 433.125 MHz
* Baud rate is 4800 bit/s
* 2400 Hz deviation
* 40 bytes preamble, 8 bytes head, 55 bytes payload, 2 byte CRC
* Decoding with https://github.com/tom2238/nrz-audio-modem
* Eg. Decoded output from ground station:
```
[2538] (RSR00002) Th 2021-09-21 15:02:36.001 (W 2176)  lat: 48.XXXX968  lon: 17.YYYY328  alt: 170.56  vH: 0.0  D: 343.4  vV: 0.0  numSV: 8
[2539] (RSR00002) Th 2021-09-21 15:02:37.001 (W 2176)  lat: 48.XXXX936  lon: 17.YYYY328  alt: 170.05  vH: 0.1  D: 343.4  vV: 0.0  numSV: 8
[2540] (RSR00002) Th 2021-09-21 15:02:38.001 (W 2176)  lat: 48.XXXX904  lon: 17.YYYY248  alt: 170.51  vH: 1.6  D: 277.2  vV: 0.0  numSV: 8
[2541] (RSR00002) Th 2021-09-21 15:02:39.001 (W 2176)  lat: 48.XXXX840  lon: 17.YYYY200  alt: 170.30  vH: 0.3  D: 277.2  vV: 0.0  numSV: 8
[2542] (RSR00002) Th 2021-09-21 15:02:40.001 (W 2176)  lat: 48.XXXX872  lon: 17.YYYY232  alt: 170.28  vH: 0.3  D: 277.2  vV: 0.0  numSV: 8
[2543] (RSR00002) Th 2021-09-21 15:02:41.001 (W 2176)  lat: 48.XXXX904  lon: 17.YYYY248  alt: 170.59  vH: 0.3  D: 277.2  vV: 0.0  numSV: 8
[2544] (RSR00002) Th 2021-09-21 15:02:42.001 (W 2176)  lat: 48.XXXX904  lon: 17.YYYY264  alt: 170.47  vH: 0.2  D: 277.2  vV: 0.0  numSV: 8
```

## Decoder setup
* nrz-audio-modem is used
* NRZ coding, 55 byte packet length, 4800 baud rate, Vaisala RS41 print mode (-P 2)
* 'sox -t pulseaudio default -t wav - 2>/dev/null | ./decoder -i - -L 55 -b 4800 -P 2'

## TODO
* Lock printing and sending on GPS.
