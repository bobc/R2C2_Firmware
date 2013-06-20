/* Copyright (c) 2013 Bob Cousins bobcousins42@googlemail.com              */
/* All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.
   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef _CONFIG_PINS_H
#define _CONFIG_PINS_H

#include "ios.h"

/*
        Machine Pin Definitions
*/

/* ==========================================================================
  Defaults for Smoothieboard rev 0.1
  
  NB *** User pin configuration in config_pin.txt overrides these defaults ***           
========================================================================== */

// x axis pins
#define X_STEP_PIN    PIN_DEF (2, 0, ACTIVE_HIGH)
#define X_DIR_PIN     PIN_DEF (0, 5, ACTIVE_HIGH)
#define X_ENABLE_PIN  PIN_DEF (0, 4, ACTIVE_LOW) 
#define X_MIN_PIN     PIN_DEF (1, 24,  ACTIVE_LOW) 

// y axis pins
#define Y_STEP_PIN    PIN_DEF (2, 1, ACTIVE_HIGH)
#define Y_DIR_PIN     PIN_DEF (0, 11, ACTIVE_HIGH)
#define Y_ENABLE_PIN  PIN_DEF (0, 10, ACTIVE_LOW) 
#define Y_MIN_PIN     PIN_DEF (1, 26,  ACTIVE_LOW) 

// z axis pins
#define Z_STEP_PIN    PIN_DEF (2, 2, ACTIVE_HIGH)
#define Z_DIR_PIN     PIN_DEF (0, 20, ACTIVE_HIGH) 
#define Z_ENABLE_PIN  PIN_DEF (0, 19, ACTIVE_LOW)  
#define Z_MIN_PIN     PIN_DEF (1, 28,  ACTIVE_LOW) 

// e axis pins
#define E_STEP_PIN    PIN_DEF (2, 3, ACTIVE_HIGH)
#define E_DIR_PIN     PIN_DEF (0, 22, ACTIVE_HIGH)
#define E_ENABLE_PIN  PIN_DEF (0, 21, ACTIVE_LOW) 


//
#define STEPPERS_RESET_PIN              UNDEFINED_PIN_DEF

// CTC #1 / Extruder 0
#define EXTRUDER_0_HEATER_PIN           PIN_DEF (2,4,ACTIVE_HIGH)
#define EXTRUDER_0_FAN_PIN              UNDEFINED_PIN_DEF

#define EXTRUDER_0_SENSOR_ADC_PIN       PIN_DEF_EX (0,23,ACTIVE_HIGH,1)
#define EXTRUDER_0_SENSOR_ADC_CHANNEL   0                        


// CTC #2 / Heated Bed
#define HEATED_BED_0_HEATER_PIN         PIN_DEF (2,6,ACTIVE_HIGH)

#define HEATED_BED_0_ADC_PIN            PIN_DEF_EX (0,24,ACTIVE_HIGH,1)
#define HEATED_BED_0_SENSOR_ADC_CHANNEL 1                        

// might be needed before config file is read?
// No buzzer
#define BUZZER_PORT                     UNDEFINED_PORT
#define BUZZER_PIN_NUMBER               UNDEFINED_PIN_NUMBER

// the push switch to enter the bootloader
#define BOOT_SW_PORT                    2
#define BOOT_SW_PIN_NUMBER              10

// SPI for SD card interface
#define SPI_SSEL0                       PIN_DEF (0,6,ACTIVE_HIGH)
#define SPI_SCK0                        PIN_DEF (0,7,ACTIVE_HIGH)
#define SPI_MISO0                       PIN_DEF (0,8,ACTIVE_HIGH)
#define SPI_MOSI0                       PIN_DEF (0,9,ACTIVE_HIGH)
#define SD_SPI_CHANNEL                  1

#define CFG_HAVE_DIGIPOT                1
#define CFG_DIGIPOT_I2C_CHAN            1
#define CFG_PIN_DIGIPOT_I2C_SCL         PIN_DEF_EX (0,1,ACTIVE_HIGH,3) 
#define CFG_PIN_DIGIPOT_I2C_SDA         PIN_DEF_EX (0,0,ACTIVE_HIGH,3)

#endif  /* _CONFIG_PINS_H */
