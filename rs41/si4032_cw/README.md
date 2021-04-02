# RS41 si4032_cw

RS41 project with Si4032 ISM TRANSMITTER, CW (morse code) example.
* First inicialize all components (clock, gpio, spi ...)
* Invert onboard leds state
* In main loop blink with leds and after cca 15 second send morse message.
* Default transmitting frequency is 433.125 MHz
* Output power is 1 dBm (1.26 mW)
* Modulation type is Unmodulated carrier (suitable for CW or RTTY)

![Gqrx CW FlDigi](si4032_cw.png?raw=true "Gqrx CW FlDigi")
