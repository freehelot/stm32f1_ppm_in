# stm32f1_ppm_in
PPM Read with DEBUG print RC channels on STM32F1 (bluepill)

* PPM Reader with DEBUG print RC channels
* STM32F103C8T6 (BluePill) tested
* This program gets PPM signal input from RC radio PPM out connection.
* Timer 4 is used with chanels 1 and 2, connect PPM signal cable to PB6, GND to GND.
* Debuged with ST-link, PB3 is SWdebug and reset is done via ST-link.
* Up to 12 channels have been tested working with no problems on Taranis X9+, Turnigy 9x and arduino uno as ppm generator.
* PPM signal triggers interrupts and channel 1 and channel 2 are used for detecting edges (rising or falling) and update channel values.
* Counter reset of channels is done hardware wise since it's recommended over software reset (multiple interrupts, interrupt priority etc.)

Documented but expect grammatical errors and lot of unneeded comments left through code testing. 
Everything was done with STM32Cube and Truestudio. 
Part of project for reading PPM  from RX like Taranis X9D and to generate PPM back to trainer port. Cheap simulator dongles are based on the same  STM32F103C8T6 with the same principle.
