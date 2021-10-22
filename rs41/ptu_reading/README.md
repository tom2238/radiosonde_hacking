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
* PA3 - 1st measurement (REF1)
* PC14 - 2nd measurement (HUMI TEMP)
* PC15 - 2nd measurement (SENSOR TEMP)
* PB6 - 3rd measurement (REF2)
* PB12 - TEMP mode activation / start measurement

## Humidity channel
* PB4 - 1st measurement (REF1)
* PB3 - 2nd measurement (HUMI value)
* PB5 - 3rd measurement (REF2)
* PA2 - HUMI mode activation / start measurement
