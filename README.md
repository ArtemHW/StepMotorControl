# StepMotorControl

## Description 
This is my initial project on the stm32 microcontroller, utilizing the BluePeel prototyping board with stm32f103c8t6 MCU. The objective was to create a stepper motor controller that can be operated through buttons or a phone application. The microcontroller communicates with the phone via the bluetooth module through UART. The motor direction (clockwise or counterclockwise) can be controlled using buttons. The motor's angular velocity and torque can be adjusted using two potentiometers. To prevent energy waste and overheating of the motor, a thermistor was integrated to monitor the motor shell's temperature. If the temperature limit is crossed, the microcontroller switches to Standby mode. The repository includes the project's source code and circuit diagram.

## Circuit Diagram 

## Block scheme 
![App Screenshot](https://github.com/ArtemHW/images/blob/main/BlueMotCon.png)
## Thermistor wiring diagram and calibration
![App Screenshot](https://github.com/ArtemHW/images/blob/main/Thermistor.png)
![App Screenshot](https://github.com/ArtemHW/images/blob/main/Calibration_thermistor.png)
## Mobile application 
