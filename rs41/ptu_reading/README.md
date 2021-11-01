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

## Sample data
* Temperature looks good but humidity not (maybe H_REF1 or H_REF2 is bad)
```
T_REF1: 266681 pulses T_Humidity: 373730 pulses T_Sensor: 362271 pulses T_REF2: 386262 pulses
H_REF1: 561136 pulses H_Sensor: 560377 pulses H_REF2: 560393 pulses
Temperature sensor: 22.880 degC Temperature humidity: 23.788 degC Humidity sensor: 41 % Dew point: 9.092 degC
T_REF1: 266681 pulses T_Humidity: 373690 pulses T_Sensor: 363312 pulses T_REF2: 386253 pulses
H_REF1: 561136 pulses H_Sensor: 560377 pulses H_REF2: 560393 pulses
Temperature sensor: 23.723 degC Temperature humidity: 23.763 degC Humidity sensor: 41 % Dew point: 9.793 degC
T_REF1: 266681 pulses T_Humidity: 373642 pulses T_Sensor: 363904 pulses T_REF2: 385909 pulses
H_REF1: 561136 pulses H_Sensor: 560377 pulses H_REF2: 560393 pulses
Temperature sensor: 24.425 degC Temperature humidity: 23.966 degC Humidity sensor: 41 % Dew point: 10.376 degC
T_REF1: 266681 pulses T_Humidity: 373618 pulses T_Sensor: 364229 pulses T_REF2: 385909 pulses
H_REF1: 561133 pulses H_Sensor: 560926 pulses H_REF2: 560385 pulses
Temperature sensor: 24.688 degC Temperature humidity: 23.948 degC Humidity sensor: 0 % Dew point: .000 degC
T_REF1: 266681 pulses T_Humidity: 373538 pulses T_Sensor: 362932 pulses T_REF2: 385906 pulses
H_REF1: 561139 pulses H_Sensor: 560921 pulses H_REF2: 560393 pulses
Temperature sensor: 23.643 degC Temperature humidity: 23.887 degC Humidity sensor: 0 % Dew point: .000 degC
T_REF1: 266681 pulses T_Humidity: 373474 pulses T_Sensor: 362311 pulses T_REF2: 385909 pulses
H_REF1: 561137 pulses H_Sensor: 560929 pulses H_REF2: 560393 pulses
Temperature sensor: 23.139 degC Temperature humidity: 23.835 degC Humidity sensor: 0 % Dew point: .000 degC
T_REF1: 266681 pulses T_Humidity: 373434 pulses T_Sensor: 362999 pulses T_REF2: 386275 pulses
H_REF1: 561136 pulses H_Sensor: 560385 pulses H_REF2: 560393 pulses
Temperature sensor: 23.457 degC Temperature humidity: 23.547 degC Humidity sensor: 32 % Dew point: 6.243 degC
T_REF1: 266689 pulses T_Humidity: 373450 pulses T_Sensor: 363815 pulses T_REF2: 385989 pulses
H_REF1: 561136 pulses H_Sensor: 560377 pulses H_REF2: 560393 pulses
Temperature sensor: 24.300 degC Temperature humidity: 23.759 degC Humidity sensor: 41 % Dew point: 10.272 degC
```
