# Fly_MCU

My Fly Board include:

TOP Layer:
- Main MCU stm32f103
- RF module based on si4463
- 10DOF MPU9250	(optionally)
- BEC 1A (for servo motors)	(optionally)

BOTTOM Layer:
- ESC based on ATmega8A	(optionally)
- GPS Module NEO-6/NEO-7 (optionally)

The device can:
- to manage 1 brushless motor (by RF module);
- to manage 2 servo motors. (auto-correction can be enabled using the 10Dof module)
- data output from the module 10Dof and GPS (and battery voltage) over the air

Features of the program:
- code with multiple threads. You can set priorities
- includes Kalman filter
- includes an SD card library
- Written in SW4STM32

In the future I will add the possibility of write log file on the SD card

This board is double sided for DIY PCB home print !!!

![Image alt](https://github.com/fademike/Fly_MCU/raw/master/stm32f103-fly.png)


