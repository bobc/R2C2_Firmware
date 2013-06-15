This is a fork of R2C2 firmware by Bob Cousins.

It contains some new and experimental features.

o designed for flexibility
 	- application settings and pin configurations can be loaded at run time from an SD card.
 	  A default config can still be used if no SD card available.
	   
o portable, light weight RTOS features 
	- cooperative multi tasking, message queues, IO subsystem to allow redirectable stream IO
	
o GCode engine runs as separate task to comms processors, so that the comms need never
  hang up when the move queue is full
  
o allows multiple comms streams via USB and UARTs   	
 	
o designed for portability
	- The CPU and other platform specific code is abstracted into a HAL. There is a minumum amount
	  of code that requires porting. 
	- application and high level drivers run on top of HAL.
	

--
R2C2 electronic board is a new cutting edge technology for RepRap 3D printers (and other 3D printers and general CNC machines).

Visit R2C2 website: http://www.3dprinting-r2c2.com/
