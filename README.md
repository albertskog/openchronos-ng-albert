#Openchronos-ng

This is my fork of [http://sourceforge.net/u/joachimbreitner/openchronos-ng](http://sourceforge.net/u/joachimbreitner/openchronos-ng)

Which in its turn is a fork of Openchronos-ng, modular opensource firmware for the TI eZ430 Chronos. - [http://openchronos-ng.sourceforge.net](http://openchronos-ng.sourceforge.net)



## Building
To build, first choose what modules to include by running

	make config

Then, to compile, run

	make
	
The generated file `openchronos.txt` may now be uploaded using Control Center. After the upload the screen will say "boot". Press the light button to enter re-programming mode again or any other to start the clock.


## Usage
This firmware is different from the stock TI one. Use * to enter (and exit) the menu to choose what mode to display. Each mode may use the rest of the buttons  (and * long press) in any way desired.

## Modules
Clock functions are called modules and reside in the modules folder. Take a look at some of the simple modules to get a hang of how they work.
