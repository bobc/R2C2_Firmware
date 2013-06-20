Portable Firmware for ARM electronics
=====================================

This is a fork of R2C2 firmware. It contains some new and experimental features.

* designed for flexibility
    * application settings and pin configurations can be loaded at run time from an SD card. A default config can still be used if no SD card available.

* light weight RTOS
	* cooperative multi tasking, message queues, IO subsystem to allow redirectable stream IO

* GCode engine runs as separate task to comms processors, so that the comms need never hang up when the move queue is full

* allows multiple comms streams via USB and UARTs

* designed for portability
	* The CPU and other platform specific code is abstracted into a HAL. There is a minimum amount of code that requires porting.
	* application and high level drivers run on top of HAL.


Supported boards
================

Fully working
-------------

* R2C2 by bitBox http://www.3dprinting-r2c2.com/

Ports in progress
-----------------

* Smoothieboard by Arthur Wolf http://smoothieware.org/smoothieboard

Planned
-------

* Arduino Due http://arduino.cc/en/Main/ArduinoBoardDue with RAMPS-FD http://reprap.org/wiki/RAMPS-FD
