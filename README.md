# FreeRTOS

Arduino FreeRTOS - Carbon monoxide Gas Sensor MQ7

The project consists of reading analog input values by using a gas sensor (Carbon Monoxide) MQ-7 and displaying the information on a 16X2 I2C LCD display.

Two tasks were used in this:
Task_Sensor_MQ7 and Task_LCD
Task_Sensor_MQ7: Reads the Analog value from MQ-7 gas sensor every second and sends it to task task_LCD, and also writes on Serial monitor.
Task_LCD displays the values on I2c 16x2 display , the reading values sent by task task_Sensor_MQ7.
Two LEDs were also used:
Green Led and Red Led.
Green Led, indicates that the value read from the gas sensor is below then the threshold, this means that there is no gas detected, so the LED remains on. And the red Led off
Red Led, indicates that the value read from the gas sensor is above the threshold, this means that there is gas detected, so the led remains on and green off.
