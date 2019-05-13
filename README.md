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

I couldn't find any similar project on STM32 not using Arduino IDE which reads PPM input done with interrupts and I didn't want my main while loop clogged with reading PPM inputs.

Documented but expect grammatical errors and lot of unneeded comments left through code testing. Code is not perfect and should and will be updated in the future. 

Everything was done with STM32Cube and Truestudio and I've supplied every file from the project folder needed.  
Part of project for reading PPM  from RX like Taranis X9D and to generate PPM back to trainer port. Cheap simulator dongles are based on the same  STM32F103C8T6 with the same principle. All you have to do is just attach PPM and GND to STM32 from the back of Taranis/other TX.
![](https://wiki.stm32duino.com/images/a/ae/Bluepillpinout.gif)
