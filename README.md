### Firstly, congratulations for HEU robot team - Nooploop Wings of Dream winning Robomaster east division Champion in 2021-2022 season.

This is the Embedded code of Harbin Engineering University Robot Team - Wings of Dream. It's included gimbal and chassis control two parts, and the signal working flow could see as follows. In chassis control part, the MCU get the DBUS signal from DR16, and decode the control command and deliver the gimbal control command to the MCU on gimbal with CAN1. And the MCU on chassis is in charge of four 3508 motor PID control, super caps management, communication with optical detection system (in other words, NUC), and communication with judgement system. The gimbal part takes charge of TWO order PID gimbal control, trajectory tracking, Extended Kalman Filter estimation, and shooting control.

All framework are developed by CubeMX and FreeRTOS. Code are programmed based on HAL.

More technical details would be added in the future.


--2018-2019 the Captain of Wings of Dream 
					Ruiqi Jiang 2022/6/29
