Portable Firmware for ARM Electronics
=====================================

This is a fork of R2C2 firmware. It contains some new and experimental features.

* designed for flexibility
    * application settings and pin configurations can be loaded at run time from an SD card, or a default config_xxx.h can still be used if no SD card available.

* light weight RTOS
	* cooperative multi tasking, message queues, IO subsystem to allow redirectable stream IO

* GCode engine runs as separate task to comms processors, so that the comms need never hang up when the move queue is full

* allows multiple comms streams via USB and UARTs

* designed for portability
	* The CPU and other platform specific code is abstracted into a HAL. There is a minimum amount of code that requires porting.
	* application and high level drivers run on top of HAL.

* supports targets with smaller Flash (minimum 64KB Flash, 8KB RAM) 

USB Mass Storage Bootloader for LPC17xx
=======================================

The R2C2 USB bootloader has been adapted to run on R2C2 and Smoothieboard.

The Smoothieboard USB bootloader requires an extra switch, otherwise the ROM serial bootloader is invoked.  

The USB bootloader can be easily configured to run on other LPC17xx versions by config file.
 
Supported boards
================

Fully working
-------------

* R2C2 by bitBox http://www.3dprinting-r2c2.com/

Ports in progress
-----------------

* Smoothieboard by Arthur Wolf http://smoothieware.org/smoothieboard

Planned/considered
----------------

* Arduino Due http://arduino.cc/en/Main/ArduinoBoardDue with RAMPS-FD http://reprap.org/wiki/RAMPS-FD
* 4pi by Kliment http://reprap.org/wiki/4pi
* Sango-bc by Bob Cousins http://reprap.org/wiki/Sango_bc
* STMF4Discovery 
* Kinetis K20 (Teensy3)

 