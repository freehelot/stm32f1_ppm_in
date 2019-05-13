# stm32f1_ppm_in
PPM Read with DEBUG print RC channels on STM32F1 (bluepill)

* PPM Reader with DEBUG print RC channels
* STM32F103C8T6 ([BluePill](https://wiki.stm32duino.com/index.php?title=Blue_Pill)) tested
* This program gets PPM signal input from RC radio PPM out connection.
* Timer 4 is used with chanels 1 and 2, connect PPM signal cable to PB6, GND to GND.
* Debuged with ST-link, PB3 is SWdebug and reset is done via ST-link.
* Up to 12 channels have been tested working with no problems on Taranis X9+, Turnigy 9x and arduino uno as ppm generator.
* PPM signal triggers interrupts and channel 1 and channel 2 are used for detecting edges (rising or falling) and update channel values.
* Counter reset of channels is done hardware wise since it's recommended over software reset (multiple interrupts, interrupt priority etc.)

I couldn't find any similar project on STM32 not using Arduino IDE which reads PPM input done with interrupts and I didn't want my main while loop clogged with reading PPM inputs.

Documented but expect grammatical errors and lot of unneeded comments left through code testing. Code is not perfect and should and will be updated in the future. 

Documented but expect grammatical errors and lot of unneeded comments left through code testing. Everything was done with STM32Cube and Truestudio. Part of project for reading PPM from RX like [Taranis](https://github.com/opentx/opentx/wiki/Taranis-I-O-ports) X9D and to generate PPM back to trainer port. Cheap [simulator dongles](https://www.banggood.com/22-in-1-RC-Flight-Simulator-Cable-for-Realflight-G7-G6-G5-G4-p-950398.html?rmmds=search&cur_warehouse=CN) are based on the same STM32F103C8T6 with the same principle.

