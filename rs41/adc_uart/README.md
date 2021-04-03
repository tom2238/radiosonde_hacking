# RS41 ADC reading

Reading following values from ADC1:
* MCU supply voltage
* ADC reference voltage (should be 1.2 V)
* Battery voltage from PA5 pin (USB +5V source)
* MCU temperature

USART3 configuration:
* Baudrate 9600 bps
* 8 data bits, 1 stop bit, no parity
* No flow control

## Example output

```
ADC supply: 2.977 V, ADC reference: 1.195 V, Battery voltage: 4.668 V, MCU temperature: 24.729 °C
ADC supply: 2.977 V, ADC reference: 1.195 V, Battery voltage: 4.679 V, MCU temperature: 24.729 °C
ADC supply: 2.977 V, ADC reference: 1.195 V, Battery voltage: 4.651 V, MCU temperature: 24.729 °C
ADC supply: 2.977 V, ADC reference: 1.195 V, Battery voltage: 4.622 V, MCU temperature: 24.729 °C
ADC supply: 2.977 V, ADC reference: 1.195 V, Battery voltage: 4.535 V, MCU temperature: 24.729 °C
```

Battery voltage is measured via 1:2 resistor voltage divider. Multiply measured voltage twice to get correct value.
