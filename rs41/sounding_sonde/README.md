# RS41 Sounding Sonde

RS41 project with Si4032 + Ublox G6010 GPS + PTU + battery measure
* Blinking with onboard green LED if waiting for GPS 3D fix, if fix is OK then turn on.
* XDATA USART3 baud rate is 38400 bps
* GPS USART1 baud rate is 38400 bps
* Default transmitting frequency is 433.120 MHz
* Output power is 11 dBm (12.60 mW)
* GFSK modulation, FIFO mode
* Transmit short packet (<= 64 bytes)
* Send radio packet every second
* TX sending is locked on GPS (under test), maybe works.
* Turn off by button

## GPS configuration
* Baud rate is 38400 bps, using binary UBX protocol
* 1 Hz navigation rate
* NAV-SOL, NAV-TIMEUTC, NAV-POSLLH and NAV-VELNED messages are received.

## Radio packet
* Frequency is 433.120 MHz
* Baud rate is 4800 bit/s
* 2400 Hz deviation
* 40 bytes preamble, 8 bytes head, 62 bytes payload, 2 byte CRC
* Decoding with https://github.com/tom2238/nrz-audio-modem
* Eg. Decoded output from ground station:
```
[846] (RH1312D2) Sa 2022-06-25 12:17:42.001 (W 2215)  lat: 48.9091392  lon: 17.0386480  alt: 195.58  vH: 0.0  v3D: 0.0  D: 190.3  vV: 0.0  numSV: 8  Tm: 21.1  vBat: 4.9  freq: 433.120  txPower: 8
[847] (RH1312D2) Sa 2022-06-25 12:17:43.001 (W 2215)  lat: 48.9091392  lon: 17.0386480  alt: 195.56  vH: 0.0  v3D: 0.0  D: 190.3  vV: 0.0  numSV: 8  Tm: 21.0  vBat: 4.9  freq: 433.120  txPower: 8
[848] (RH1312D2) Sa 2022-06-25 12:17:44.001 (W 2215)  lat: 48.9091392  lon: 17.0386480  alt: 195.55  vH: 0.0  v3D: 0.0  D: 190.3  vV: 0.0  numSV: 8  Tm: 21.2  vBat: 4.9  freq: 433.120  txPower: 8
[849] (RH1312D2) Sa 2022-06-25 12:17:45.001 (W 2215)  lat: 48.9091392  lon: 17.0386480  alt: 195.52  vH: 0.0  v3D: 0.1  D: 190.3  vV: 0.0  numSV: 8  Tm: 21.1  vBat: 4.9  freq: 433.120  txPower: 8
[850] (RH1312D2) Sa 2022-06-25 12:17:46.001 (W 2215)  lat: 48.9091392  lon: 17.0386496  alt: 195.51  vH: 0.0  v3D: 0.0  D: 190.3  vV: 0.0  numSV: 8  Tm: 21.4  vBat: 4.9  freq: 433.120  txPower: 8
```

* For habitat:
```
$$RH1312D2,1345,13:22:36,48.9089312,17.0381536,150,1.6,-100.0,2.9,183,3.2,5,433.120 MHz https://github.com/tom2238/radiosonde_hacking/tree/main/rs41/sounding_sonde*f56b
```

### Packet payload
* **GPS** 
* Year(4), Month(1), Day(1), Hour(1), Min(1), Sec(1), gpsFix(1), numSV(1), lon(4), lat(4), hMSL(4), speed(4), gSpeed(4), heading(4), iTOW(4), week(2)
* GPS total: 41 bytes
* **PTU**
* MainTemp(4)
* PTU total: 4
* **Battery**
* Voltage(1)
* Battery total: 1
* **Sonde info**
* ID(8), framecount(2), frequency(2), txpower(1)
* Sonde info total: 13
* **Empty**
* 3
* **Sum up:** 62 bytes

### Bytes
* One frame
```
::: 0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F
 0: 86 35 F4 40 93 DF 1A 60 00 00 07 E6 06 19 0C 11 
 1: 2A 03 08 0A 27 E4 2B 1D 26 F1 47 00 02 FB FD 00 
 2: 00 00 03 00 00 00 01 01 22 62 20 21 89 D0 C0 08 
 3: A7 00 00 2F 4C 31 52 48 31 33 31 32 44 32 03 4E 
 4: A9 30 08 00 00 00 89 C6  CRC OK CRC(rt) 89c6 : 89c6
```

| address  | datatype | example data | decoded | function |
| --- | --- | --- | --- | --- |
| `[0x00] `| uint8[] | `[0x8635F44093DF1A60]` | | Header |
| `[0x08]` | uint32t | `[0x000007E6]` | 2022 | GPS year |
| `[0x0C]` | uint8 | `[0x06]` | 6 | GPS month |
| `[0x0D]` | uint8 | `[0x19]` | 25 | GPS day |
| `[0x0E]` | uint8 | `[0x0C]` | 12 | GPS hour |
| `[0x0F]` | uint8 | `[0x11]` | 17 | GPS minute |
| `[0x10]` | uint8 | `[0x2A]` | 42 | GPS second |
| `[0x11]` | uint8 | `[0x03]` | 3 | GPS fix status |
| `[0x12]` | uint8 | `[0x08]` | 8 | GPS number of satellites |
| `[0x13]` | int32 | `[0x0A27E42B]` | 170386475 | GPS longtitude * 1e-7 [deg] |
| `[0x17]` | int32 | `[0x1D26F147]` | 489091399 | GPS latitude * 1e-7 [deg] |
| `[0x1B]` | int32 | `[0x0002FBFD]` | 195581 | GPS altitude [mm] above sea level |
| `[0x1F]` | uint32 | `[0x00000003]` | 3 | GPS Down velocity component [cm/s] |
| `[0x23]` | uint32 | `[0x00000001]` | 1 | GPS ground speed [cm/s] |
| `[0x27]` | uint32 | `[0x01226220]` | 19030560 | GPS heading * 1e-5 [deg] |
| `[0x2B]` | uint32 | `[0x2189D0C0]` | 562680000 | GPS time of week [ms] |
| `[0x2F]` | int16 | `[0x08A7]` | 2215 | GPS week |
| `[0x31]` | uint32 | `[0x00002F4C]` | 12108 | PTU main temperature (T/100)-100 [deg C] |
| `[0x35]` | uint8 | `[0x31]` | 49 | Battery voltage * 1e-1 [V] |
| `[0x36]` | char[8] | `[0x5248313331324432]` | RH1312D2 | Sonde ID |
| `[0x3E]` | uint16 | `[0x034E]` | 846 | Frame count |
| `[0x40]` | uint16 | `[0xA930]` | 433.12 | TX frequency [MHz] |
| `[0x42]` | uint8 | `[0x08]` | 8 | TX power [dBm] |
| `[0x43]` | uint8[3] | `[0x000000]` |   | Empty |
| `[0x46]` | uint16 | `[0x89C6]` |  35270 | CRC16 checksum |

## Configuration
* In main.c file
* TX frequency: `#define TX_FREQUENCY_MHZ 433.120f`
* TX power info for packet: `#define TX_POWER_DBM 8`
* TX power set for radio: `#define TX_POWER_SI4032 SI4032_TX_POWER_8_DBM`
* Sonde ID `static const char SondeID[8] = {'R','H','1','3','1','2','D','2'};`
* Calibration data for PTU: `static const PTUCalibrationData calib_ptu = {.cal_T1 = {1.303953f, -0.095812f, 0.005378f}, .cal_H = {44.937469f, 5.023599f}, .cal_T2 = {1.265843f, 0.122289f, 0.005889f} };
`
* ***Warning!*** Download calibration data from original sonde before reprogramming!

* TODO:
* Configuration over USART3

## Decoder setup
* nrz-audio-modem is used
* NRZ coding, 62 byte packet length, 4800 baud rate, Vaisala RS41 sounding print mode (-P 3)
* 'sox -t pulseaudio default -t wav - 2>/dev/null | ./decoder -i - -L 62 -b 4800 -P 3'
* Output from NRZ decoder is without comment and CRC.
* Comment and CRC are added in habitat_upload

## Habitat upload (test)
* Uploading to http://habitat.habhub.org/. Software in ./habitat_upload/habitat_upload directory.
* Reading payload content from stdin (output from nrz-audio-modem).
* './sounding_sonde.sh'

# Launch
* At 12.8.2022 17:09 UTC
* From JN88MV
![Habhub](launched_sonde_ph1eac8f.png?raw=true "Habhub path")
* TODO: Add more pictures

# End
* At 12.8.2022 19:45 UTC
* Low signal strength, but still working. 
* Landed position unknown.
* Send info to: t.dubina (at) volny (dot) cz
