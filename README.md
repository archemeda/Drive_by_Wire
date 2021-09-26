# Drive_by_Wire

This is a drive by wire system design used in Teknofest 2020 Robotaksi competetion by KIZILELMA Team

Authors:
Aykut Erdem Tüm
Hüseyin Bacak
Burhan Engin
Kadir Kazdal

The project is coded using HAL drivers and STM Cube IDE. 
Microcontroller: STM32F407VGT 
Motor Drivers: BTS7960B 
Steering Motor: https://www.keskinlerelektronik.com/urun/63zy24-40-24v-30rpm-dc-motor
Wireless Communication Module: NRF24
Linear potentiometer: Opkon RTL

The task is divided into 3 main components

-> Steer by wire
-> Brake by wire
-> Throttle by wire

Block diagram of the system is shown bellow

![Untitled Diagram drawio](https://user-images.githubusercontent.com/79100312/134819660-7f5222f0-aa45-4107-90ab-48dfaee790b5.png)

Steer by wire is achieved by connecting the steering motor to steering wheel rod using gears and reading wheel's current position via a linear potentiometer to calculate the error between the commanded position and current position. Calculated error is then fed into the PID algorithm to get the PWM value required to control the motor drivers.

Brake by wire is achieved by connecting a linear actuator to the brake pedal and controlling its position with a linear potentiometer and PID algorithm same as steer by wire

Throttle by wire is achieved by connecting STM to the vehicle's throttle signal cable. After calculations and experiments we found out that vehicle starts after 2250 pwm value is given to the cable for about 5 seconds and this pwm value is its minimum value. Using this info we used vehicles own motor driver to control speed

Details can be seen in code
