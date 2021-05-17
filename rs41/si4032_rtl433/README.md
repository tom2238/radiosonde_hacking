# RS41 si4032_rtl433

RS41 project with Si4032 ISM TRANSMITTER, 433.92 MHz Weather sensor
* First inicialize all components (clock, gpio, spi ...)
* Invert onboard leds state
* In main loop blink with leds and after cca 15 second send T/H data
* Default transmitting frequency is 433.9059 MHz
* Output power is 5 dBm (3.16 mW)
* Modulation type is Unmodulated carrier (suitable for CW or RTTY)
* Z31915-TX sensor imitation, see https://fccid.io/2AJ9O-Z31915T
* Modulation, frame format etc, see https://hal.inria.fr/hal-01637456/document
* Another source of information, see https://github.com/merbanan/rtl_433/blob/master/src/devices/infactory.c
* Fixed data transmission, ID 155, Temperature 32°F, Humidity 30%
* Good protocol description - http://www.tfd.hu/tfdhu/files/wsprotocol/auriol_protocol_v20.pdf - not used here

## Decoding with rtl_433
* Output from decoder
```
time      : 2021-05-17 18:39:53
model     : inFactory sensor                       ID        : 155
Temperature: 32.00 °F   Humidity  : 30 %

```

## Issues
* My home weather station cannot decode this packets. Maybe CRC calculation techniq is bad or baud rate speed is not perfect.
* rtl_433 maybe ignore CRC, I am not sure.
* Another station have only temperature sensor and use hideki protocol. https://github.com/merbanan/rtl_433/blob/master/src/devices/hideki.c. I cannot find any suitable hideki protocol description now.
