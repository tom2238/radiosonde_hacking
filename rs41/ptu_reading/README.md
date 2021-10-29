# RS41 PTU reading

RS41 project with attempting to read PTU values.
* Using TIM2 as pulse counter in input capture mode (connected on PA1 pin).
* Using TIM3 as stopwatch to calculate the length of the burst (number of pulses and frequency).
* Waiting for 1024 pulses and then calculate the frequency.
* TEMP and HUMI mode are activated by high logic level.
* Individual sensors are selected by high logic level.
* Reference heating: controlled by GPIO1 of Si4032 radio.
* If reference heating is enabled (default after Si4032 reset), measurement doesn't work!!
* For more information see https://github.com/bazjo/RS41_Hardware/ and logicdata/Meas+Div, Startup.logicdata + logicdata/Switches+Meas_Out, Startup.logicdata (required software: Logic 1.2.18).

## Temperature channel
* PB6 - 1st measurement (REF1)
* PC14 - 2nd measurement (HUMI TEMP)
* PC15 - 2nd measurement (SENSOR TEMP)
* PA3 - 3rd measurement (REF2)
* PB12 - TEMP mode activation / start measurement

## Humidity channel
* PB4 - 1st measurement (REF1)
* PB3 - 2nd measurement (HUMI value)
* PB5 - 3rd measurement (REF2)
* PA2 - HUMI mode activation / start measurement

```
T_REF1: 266689 pulses T_Humidity: 370634 pulses T_Sensor: 359655 pulses T_REF2: 386394 pulses
H_REF1: 561279 pulses H_Sensor: 561063 pulses H_REF2: 561079 pulses
Temperature sensor: 20.692 degC Temperature humidity: 21.279 degC
T_REF1: 266689 pulses T_Humidity: 370626 pulses T_Sensor: 359775 pulses T_REF2: 385994 pulses
H_REF1: 561280 pulses H_Sensor: 561065 pulses H_REF2: 561078 pulses
Temperature sensor: 21.039 degC Temperature humidity: 21.544 degC
T_REF1: 266689 pulses T_Humidity: 370642 pulses T_Sensor: 359535 pulses T_REF2: 385957 pulses
H_REF1: 561284 pulses H_Sensor: 561066 pulses H_REF2: 561074 pulses
Temperature sensor: 20.869 degC Temperature humidity: 21.582 degC
T_REF1: 266689 pulses T_Humidity: 370642 pulses T_Sensor: 359743 pulses T_REF2: 386013 pulses
H_REF1: 561285 pulses H_Sensor: 561282 pulses H_REF2: 560537 pulses
Temperature sensor: 21.001 degC Temperature humidity: 21.544 degC
```
