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

/*
        Machine Pin Definitions
*/

/* ==========================================================================
  Defaults for R2C2 rev 1.2 hardware
  
  NB *** User pin configuration in config_pin.txt overrides these defaults ***           
========================================================================== */

//x axis pins

#define S0_STEP_PIN    PIN_DEF (1, 20, ACTIVE_HIGH)   /* P1.20 */
#define S0_DIR_PIN     PIN_DEF (1, 23, ACTIVE_HIGH)   /* P1.23 */
#define S0_ENABLE_PIN  PIN_DEF (1, 24, ACTIVE_LOW)    /* P1.24 */

#define X_MIN_PIN     PIN_DEF (2, 6,  ACTIVE_LOW)    /* P2.6 */

//y axis pins
#define S1_STEP_PIN    PIN_DEF (1, 25, ACTIVE_HIGH)   /* P1.25 */
#define S1_DIR_PIN     PIN_DEF (1, 26, ACTIVE_HIGH)   /* P1.26 */
#define S1_ENABLE_PIN  PIN_DEF (1, 28, ACTIVE_LOW)    /* P1.28 */

#define Y_MIN_PIN     PIN_DEF (2, 7,  ACTIVE_LOW)    /* P2.7 */

//z axis pins
#define S2_STEP_PIN    PIN_DEF (1, 29, ACTIVE_HIGH)   /* P1.29 */
#define S2_DIR_PIN     PIN_DEF (0, 0, ACTIVE_HIGH)   /* P0.0 */
#define S2_ENABLE_PIN  PIN_DEF (0, 1, ACTIVE_LOW)    /* P0.1 */

#define Z_MIN_PIN     PIN_DEF (2, 8,  ACTIVE_LOW)    /* P2.8 */

//e axis pins
#define S3_STEP_PIN    PIN_DEF (0, 10, ACTIVE_HIGH)   /* P0.10 */
#define S3_DIR_PIN     PIN_DEF (0, 11, ACTIVE_HIGH)   /* P0.11 */
#define S3_ENABLE_PIN  PIN_DEF (2, 10, ACTIVE_LOW)    /* P2.10 */



// sensors
// extruder 0
#define SENSOR_0_ADC_PIN          PIN_DEF_EX (0, 2, ACTIVE_HIGH, 2) 
#define SENSOR_0_ADC_CHANNEL      7
#define SENSOR_0_TABLE_INDEX      0

// heatbed
#define SENSOR_1_ADC_PIN          PIN_DEF_EX (0, 3, ACTIVE_HIGH, 2)
#define SENSOR_1_ADC_CHANNEL      6
#define SENSOR_1_TABLE_INDEX      1

//
#define STEPPERS_RESET_PIN              PIN_DEF (0,22,ACTIVE_LOW)         /* P0.22 */

//
#define AUX_0_OUTPUT_PIN              PIN_DEF (2,3,ACTIVE_HIGH)       /* P2.3 */
#define AUX_0_OUTPUT_PWM              hal_pwm_bang_bang
#define AUX_0_OUTPUT_CHANNEL          0

// CTC #1 / Extruder 0
#define EXTRUDER_0_HEATER_PIN           PIN_DEF (2,4,ACTIVE_HIGH)       /* P2.4 */
#define EXTRUDER_0_HEATER_PWM           hal_pwm_bang_bang
#define EXTRUDER_0_HEATER_CHANNEL       0
//#define EXTRUDER_0_FAN_PIN              PIN_DEF (2,3,ACTIVE_HIGH)       /* P2.3 */
#define EXTRUDER_0_SENSOR_INDEX         0

// CTC #2 / Heated Bed
#define HEATED_BED_0_HEATER_PIN         PIN_DEF (2,5,ACTIVE_HIGH)       /* P2.5 */
#define HEATED_BED_0_HEATER_PWM         hal_pwm_bang_bang
#define HEATED_BED_0_HEATER_CHANNEL     0
#define HEATED_BED_0_SENSOR_INDEX       1

// might be needed before config file is read?
#define BUZZER_PORT                     2                               /* P2.2 PWM1[3] */
#define BUZZER_PIN_NUMBER               2                               /* P2.2 PWM1[3] */

// the push switch to enter the bootloader (may be used by uart3: DBG_RXD)
#define BOOT_SW_PORT                    4
#define BOOT_SW_PIN_NUMBER              29

// SPI
#define SD_SPI_SSEL                       PIN_DEF (0,16,ACTIVE_HIGH)
#define SD_SPI_SCK                        PIN_DEF (0,15,ACTIVE_HIGH)
#define SD_SPI_MISO                       PIN_DEF (0,17,ACTIVE_HIGH)
#define SD_SPI_MOSI                       PIN_DEF (0,18,ACTIVE_HIGH)
#define SD_SPI_CHANNEL                  0

#define CFG_HAVE_DIGIPOT                0
#define CFG_DIGIPOT_I2C_CHAN            0
#define CFG_PIN_DIGIPOT_I2C_SCL         UNDEFINED_PIN_DEF 
#define CFG_PIN_DIGIPOT_I2C_SDA         UNDEFINED_PIN_DEF

#endif  /* _CONFIG_PINS_H */
