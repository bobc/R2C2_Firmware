/* Copyright (C) 2009-2010 Michael Moon aka Triffid_Hunter   */
/* Copyright (c) 2011 Jorge Pinto - casainho@gmail.com       */
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

#define PORT_A 0
#define PORT_B 1
#define PORT_C 2
#define PORT_D 3

#define PIN_D0        PIN_DEF (PORT_A, 8, ACTIVE_HIGH)
#define PIN_D1        PIN_DEF (PORT_A, 9, ACTIVE_HIGH)
#define PIN_D2        PIN_DEF (PORT_B, 25, ACTIVE_HIGH)
#define PIN_D3        PIN_DEF (PORT_C, 28, ACTIVE_HIGH)

#define PIN_D4        PIN_DEF (PORT_C, 26, ACTIVE_HIGH)
#define PIN_D5        PIN_DEF (PORT_C, 25, ACTIVE_HIGH)
#define PIN_D6        PIN_DEF (PORT_C, 24, ACTIVE_HIGH)
#define PIN_D7        PIN_DEF (PORT_C, 23, ACTIVE_HIGH)

#define PIN_D8        PIN_DEF_EX (PORT_C, 22, ACTIVE_LOW, 0)
#define PIN_D9        PIN_DEF_EX (PORT_C, 21, ACTIVE_LOW, 0)
#define PIN_D10       PIN_DEF (PORT_C, 29, ACTIVE_LOW)
#define PIN_D11       PIN_DEF (PORT_D, 7, ACTIVE_LOW)

#define PIN_D12       PIN_DEF (PORT_D, 8, ACTIVE_HIGH)
#define PIN_D13       PIN_DEF (PORT_B, 27, ACTIVE_HIGH)

#define PIN_D14       PIN_DEF (PORT_D, 4, ACTIVE_HIGH)
#define PIN_D15       PIN_DEF (PORT_D, 5, ACTIVE_HIGH)
#define PIN_D16       PIN_DEF (PORT_A, 13, ACTIVE_HIGH)
#define PIN_D17       PIN_DEF (PORT_A, 12, ACTIVE_HIGH)

#define PIN_D18       PIN_DEF (PORT_A, 11, ACTIVE_HIGH)
#define PIN_D19       PIN_DEF (PORT_A, 10, ACTIVE_HIGH)
#define PIN_D20       PIN_DEF (PORT_B, 12, ACTIVE_HIGH)
#define PIN_D21       PIN_DEF (PORT_B, 13, ACTIVE_HIGH)

#define PIN_D22       PIN_DEF (PORT_B, 26, ACTIVE_HIGH)
#define PIN_D23       PIN_DEF (PORT_A, 14, ACTIVE_HIGH)
#define PIN_D24       PIN_DEF (PORT_A, 15, ACTIVE_HIGH)
#define PIN_D25       PIN_DEF (PORT_D, 0, ACTIVE_HIGH)

/*
        Machine Pin Definitions
*/

/* ==========================================================================
  Defaults for RAMPS-FD Ver 1 Rev A
  
  NB *** User pin configuration in config_pin.txt overrides these defaults ***           
========================================================================== */

//x axis pins

//X
#define S0_STEP_PIN    PIN_DEF (PORT_B, 18, ACTIVE_HIGH)   
#define S0_DIR_PIN     PIN_DEF (PORT_B, 17, ACTIVE_HIGH)   
#define S0_ENABLE_PIN  PIN_DEF (PORT_C, 15, ACTIVE_LOW)    

#define X_MIN_PIN      PIN_DEF (PORT_B, 26,  ACTIVE_HIGH)    // D22
#define X_MAX_PIN      PIN_DEF (PORT_D, 9,  ACTIVE_HIGH)    

//y axis pins
#define S1_STEP_PIN    PIN_DEF (PORT_B, 20, ACTIVE_HIGH)   
#define S1_DIR_PIN     PIN_DEF (PORT_B, 19, ACTIVE_HIGH)   
#define S1_ENABLE_PIN  PIN_DEF (PORT_C, 17, ACTIVE_LOW)    

#define Y_MIN_PIN      PIN_DEF (PORT_A, 15,  ACTIVE_HIGH)    
#define Y_MAX_PIN      PIN_DEF (PORT_C, 6,   ACTIVE_HIGH)    

//z axis pins
#define S2_STEP_PIN    PIN_DEF (PORT_B, 16, ACTIVE_HIGH)   
#define S2_DIR_PIN     PIN_DEF (PORT_B, 15, ACTIVE_HIGH)   
#define S2_ENABLE_PIN  PIN_DEF (PORT_C, 19, ACTIVE_LOW)    

#define Z_MIN_PIN      PIN_DEF (PORT_D, 1,  ACTIVE_HIGH)   
#define Z_MAX_PIN      PIN_DEF (PORT_C, 2,  ACTIVE_HIGH)   

//E0 axis pins
#define S3_STEP_PIN    PIN_DEF (PORT_C, 4, ACTIVE_HIGH)  
#define S3_DIR_PIN     PIN_DEF (PORT_D, 3, ACTIVE_HIGH)  
#define S3_ENABLE_PIN  PIN_DEF (PORT_A, 19, ACTIVE_LOW)   

//E1
#define S4_STEP_PIN    PIN_DEF (PORT_A, 20, ACTIVE_HIGH)  
#define S4_DIR_PIN     PIN_DEF (PORT_C, 9, ACTIVE_HIGH)  
#define S4_ENABLE_PIN  PIN_DEF (PORT_C, 7, ACTIVE_LOW)   

//E2
#define S5_STEP_PIN    PIN_DEF (PORT_D, 10, ACTIVE_HIGH)  
#define S5_DIR_PIN     PIN_DEF (PORT_C, 16, ACTIVE_HIGH)  
#define S5_ENABLE_PIN  PIN_DEF (PORT_C, 18, ACTIVE_LOW)   


// sensors
#define SENSOR_0_ADC_PIN          PIN_DEF_EX (PORT_A, 16, ACTIVE_HIGH, 0)  // AD0
#define SENSOR_0_ADC_CHANNEL      7
#define SENSOR_0_TABLE_INDEX      0

#define SENSOR_1_ADC_PIN          PIN_DEF_EX (PORT_A, 24, ACTIVE_HIGH, 0)  // AD1
#define SENSOR_1_ADC_CHANNEL      6
#define SENSOR_1_TABLE_INDEX      0

#define SENSOR_2_ADC_PIN          PIN_DEF_EX (PORT_A, 23, ACTIVE_HIGH, 0)  // AD2
#define SENSOR_2_ADC_CHANNEL      5
#define SENSOR_2_TABLE_INDEX      0

#define SENSOR_3_ADC_PIN          PIN_DEF_EX (PORT_A, 22, ACTIVE_HIGH, 0)  // AD3
#define SENSOR_3_ADC_CHANNEL      4
#define SENSOR_3_TABLE_INDEX      0

//
#define STEPPERS_RESET_PIN              UNDEFINED_PIN_DEF

//
#define AUX_0_OUTPUT_PIN              PIN_D12
#define AUX_0_OUTPUT_PWM              hal_pwm_bang_bang
#define AUX_0_OUTPUT_CHANNEL          0

#define AUX_1_OUTPUT_PIN              PIN_D2
#define AUX_1_OUTPUT_PWM              hal_pwm_bang_bang
#define AUX_1_OUTPUT_CHANNEL          0
//

// Heated Bed
#define HEATED_BED_0_HEATER_PIN         PIN_D8
#define HEATED_BED_0_HEATER_PWM         hal_pwm_hw_pwm
#define HEATED_BED_0_HEATER_CHANNEL     5
#define HEATED_BED_0_SENSOR_INDEX       0

// Extruder 0
#define EXTRUDER_0_HEATER_PIN           PIN_D9
#define EXTRUDER_0_HEATER_PWM           hal_pwm_hw_pwm
#define EXTRUDER_0_HEATER_CHANNEL       4
#define EXTRUDER_0_SENSOR_INDEX         1

//
#define EXTRUDER_1_HEATER_PIN           PIN_D10
#define EXTRUDER_1_HEATER_PWM           hal_pwm_bang_bang
#define EXTRUDER_1_HEATER_CHANNEL       0
#define EXTRUDER_1_SENSOR_INDEX         2

//
#define EXTRUDER_2_HEATER_PIN           PIN_D11
#define EXTRUDER_2_HEATER_PWM           hal_pwm_bang_bang
#define EXTRUDER_2_HEATER_CHANNEL       0
#define EXTRUDER_2_SENSOR_INDEX         3



// might be needed before config file is read?
#define BUZZER_PORT                     UNDEFINED_PORT
#define BUZZER_PIN_NUMBER               UNDEFINED_PIN_NUMBER                  

// the push switch to enter the bootloader
#define BOOT_SW_PORT                    UNDEFINED_PORT
#define BOOT_SW_PIN_NUMBER              UNDEFINED_PIN_NUMBER 

// SPI
#define SD_SPI_SSEL                       PIN_DEF (PORT_C,26,ACTIVE_HIGH)
#define SD_SPI_SCK                        PIN_DEF (PORT_A,27,ACTIVE_HIGH)
#define SD_SPI_MISO                       PIN_DEF (PORT_A,25,ACTIVE_HIGH)
#define SD_SPI_MOSI                       PIN_DEF (PORT_A,26,ACTIVE_HIGH)
#define SD_SPI_CHANNEL                  0

#define CFG_HAVE_DIGIPOT                0
#define CFG_DIGIPOT_I2C_CHAN            0
#define CFG_PIN_DIGIPOT_I2C_SCL         UNDEFINED_PIN_DEF 
#define CFG_PIN_DIGIPOT_I2C_SDA         UNDEFINED_PIN_DEF

//
#define DEBUG_PIN_0   PIN_D7
#define DEBUG_PIN_1   PIN_D6

#endif  /* _CONFIG_PINS_H */
