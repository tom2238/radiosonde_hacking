# RS41 Sounding Sonde

RS41 project with Si4032 + Ublox G6010 GPS + PTU + battery measure
* Blinking with onboard green LED if waiting for GPS 3D fix, if fix is OK then turn on.
* XDATA USART3 baud rate is 38400 bps
* GPS USART1 baud rate is 38400 bps
* Default transmitting frequency is 433.125 MHz
* Output power is 1 dBm (1.26 mW)
* GFSK modulation, FIFO mode
* Transmit short packet (<= 64 bytes)
* Send radio packet every second
* TX sending is locked on GPS (under test)

## GPS configuration
* Baud rate is 38400 bps, using binary UBX protocol
* 1 Hz navigation rate
* NAV-SOL, NAV-TIMEUTC, NAV-POSLLH and NAV-VELNED messages are received.

## Radio packet
* Frequency is 433.125 MHz
* Baud rate is 4800 bit/s
* 2400 Hz deviation
* 40 bytes preamble, 8 bytes head, 62 bytes payload, 2 byte CRC
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

### Packet payload
* **GPS** 
* Year(4), Month(1), Day(1), Hour(1), Min(1), Sec(1), gpsFix(1), numSV(1), lon(4), lat(4), hMSL(4), speed(4), groundspeed(4), heading(4), iTOW(4), week(2)
* GPS total: 41 bytes
* **PTU**
* MainTemp(4)
* PTU total: 4
* **Battery**
* Voltage(2)
* Battery total: 2
* **Sonde info**
* ID(8), framecount(2)
* Sonde info total: 10
* Sum up: 57 bytes

## Decoder setup
* nrz-audio-modem is used
* NRZ coding, 62 byte packet length, 4800 baud rate, Vaisala RS41 print mode (-P 2)
* 'sox -t pulseaudio default -t wav - 2>/dev/null | ./decoder -i - -L 62 -b 4800 -P 2'

