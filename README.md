=This is a fork of R2C2 firmware by Bob Cousins.=

It contains some new and experimental features.

* designed for flexibility
    * application settings and pin configurations can be loaded at run time from an SD card. A default config can still be used if no SD card available.
	   
* portable, light weight RTOS features 
	* cooperative multi tasking, message queues, IO subsystem to allow redirectable stream IO
	
* GCode engine runs as separate task to comms processors, so that the comms need never hang up when the move queue is full
  
* allows multiple comms streams via USB and UARTs   	
 	
* designed for portability
	* The CPU and other platform specific code is abstracted into a HAL. There is a minumum amount of code that requires porting. 
	* application and high level drivers run on top of HAL.
