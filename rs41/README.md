# RS41 project
* Some C projects for Vaisala RS41 radiosonde with stm32f100c8 MCU
* Using `QtCreator` as IDE and `Makefile` file for compilling, 
* Developen with 'LibOpenCM3' firmware library http://libopencm3.org/
* For erase and write code is `st-flash` used https://github.com/stlink-org/stlink

## Projects
* test* - Incomplete test projects
* blink - Blinking with two onboard leds using stupid delay loop
* fancyblink - Blinking with two onboard leds using timer and millis counter
* si4032_cw - ISM band CW transmitter using Si4032 onboard chip
* si4032_fifo - ISM band transmitter using Si4032 onboard chip, use FIFO mode and GFSK modulation, short packet transmission
* adc_uart - Measure MCU voltage and send it over UART
* si4032_long_packet - ISM band transmitter using Si4032 onboard chip, use FIFO mode and GFSK modulation, long packet transmission
* si4032_infinite_packet - ISM band transmitter using Si4032 onboard chip, use FIFO mode and GFSK modulation, packet length over 255 bytes, tx packet handler is off
* si4032_infinite_manchester - ISM band transmitter using Si4032 onboard chip, use FIFO mode and GFSK modulation, continuous packet transmission with Manchester coding, , tx packet handler is off
* si4032_aprs - APRS tracker, use FIFO mode and GFSK modulation
* si4032_rtl433 - Sending temperature and humidity to weather station
* si4032_marker - Channel marker, use FIFO mode with baudrate changing
* ublox6 - Serial print and packet sending with GPS information
* ptu_reading - Try attempting to read PTU :-) 
* sounding_sonde - Prepare sonde for launch with baloon :-)

## Si4032 datasheet
* Si4030/31/32 ISM TRANSMITTER - https://www.silabs.com/documents/public/data-sheets/Si4030-31-32.pdf
* Si4030/31/32 REGISTER DESCRIPTIONS - https://www.silabs.com/documents/public/application-notes/AN466.pdf
* FIFO MODE, DIRECT MODE, AND PACKET HANDLER OPERATION FOR EZRADIOPRO - https://www.silabs.com/documents/public/application-notes/AN537.pdf
* EZRADIOPRO® PROGRAMMING GUIDE - https://www.silabs.com/documents/public/application-notes/AN415.pdf

## Ublox UBX-G6010-ST datasheet
* Product summary - https://www.u-blox.com/sites/default/files/products/documents/UBX-G6010-ST_ProductSummary_%28GPS.G6-HW-09001%29.pdf
* UBX-G6010 datasheet - http://innovictor.com/pdf/UBXG6010_UBXG600_UBXG0010_DataSheet_GPS%20G6-X-09004_Confidential.pdf
* Protocol Specification V14 - https://www.u-blox.com/sites/default/files/products/documents/u-blox6-GPS-GLONASS-QZSS-V14_ReceiverDescrProtSpec_%28GPS.G6-SW-12013%29_Public.pdf

## PTU chips datasheet
* Hex unbuffered inverter - https://assets.nexperia.com/documents/data-sheet/74HCU04.pdf
* Triple buffer - https://assets.nexperia.com/documents/data-sheet/74LVC3G34.pdf
* Low-voltage,single-supply,4-channelspst analogswitch - https://www.ti.com/lit/ds/symlink/ts3a4751.pdf
* SPDT Analog Switch Single-Channel 2:1 Multiplexer and Demultiplexer - https://www.ti.com/lit/ds/symlink/ts5a9411.pdf
* TinyLogic UHS 3-Input NORGate - https://www.onsemi.com/pdf/datasheet/nc7sz27-d.pdf
