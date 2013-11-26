/* Copyright (c) 2012 Bob Cousins bobcousins42@googlemail.com       */
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

#include <string.h>

// hal
//? #include "spi.h"
#include "ios.h"    // pin defs

//lib_fatfs
#ifdef HAVE_FILESYSTEM
#include "ff.h"
#endif

// lib_r2c2
#include "debug.h"    // may not be initialised yet?

// app
#include "gcode_parse.h"  // exec gcode

#include "stepper.h"        // step_led options

#include "config.h"
#include "app_config.h"

#include "config_app.h"
#include "config_pins.h"

/* structure reflecting the configuration of the system
 */
tApplicationConfiguration config;

#define NUM_TOKENS(table) (sizeof(table)/sizeof(tConfigItem))

// lookup table for general/application level configuration
// the lookup table associates keywords with config items in RAM
static const tConfigItem config_lookup [] = 
{
  //  
  // general config
  //
  
  { "machine_model",        &config.machine_model,      TYPE_INT,       {.val_i= CFG_MACHINE_MODEL    }},
  { "acceleration",         &config.acceleration,       TYPE_DOUBLE,    {.val_d= CFG_ACCELERATION}},         /* in mm / second^2 */
  { "junction_deviation",   &config.junction_deviation, TYPE_DOUBLE,    {.val_d= CFG_JUNCTION_DEVIATION }},  
  
  { "auto_power_off_time",  &config.auto_power_off_time, TYPE_INT,      {.val_i= CFG_AUTO_POWER_OFF_TIME  }},
  
  //  
  // interfaces
  //
  
  { "debug_flags",          &config.debug_flags,            TYPE_INT,   {.val_i= CFG_DEBUG_FLAGS   }},
  { "step_led_flash_method",&config.step_led_flash_method,  TYPE_INT,   {.val_i= CFG_STEP_LED_FLASH_METHOD }},
  { "beep_on_events",       &config.beep_on_events,         TYPE_INT,   {.val_i= CFG_BEEP_ON_EVENTS  }}, // 0x0000000F

  { "control_panel",        &config.interface_control_panel_enabled, TYPE_INT, {.val_i = 0}},
  
  { "cp_lcd_type",          &config.interface_cp_lcd_type,    TYPE_INT, {.val_i = 1}},
  { "cp_lcd_rows",          &config.interface_cp_lcd_rows,    TYPE_INT, {.val_i = 4}},
  { "cp_lcd_cols",          &config.interface_cp_lcd_cols,    TYPE_INT, {.val_i = 20}},

  { "tcp_ip_enabled",       &config.interface_tcp_ip_enabled,   TYPE_INT, {.val_i = 0}},
  { "network_interface",    &config.interface_tcp_ip_phy_type,  TYPE_INT, {.val_i = 0}},

  //  
  // axis config
  //
    
  { "steps_per_mm_x", &config.axis[X_AXIS].steps_per_mm, TYPE_DOUBLE, {.val_d= CFG_STEPS_PER_MM_X}},  // 80
  { "steps_per_mm_y", &config.axis[Y_AXIS].steps_per_mm, TYPE_DOUBLE, {.val_d= CFG_STEPS_PER_MM_Y}},  // 80
  { "steps_per_mm_z", &config.axis[Z_AXIS].steps_per_mm, TYPE_DOUBLE, {.val_d= CFG_STEPS_PER_MM_Z}},  // 6400
  { "steps_per_mm_e", &config.axis[E_AXIS].steps_per_mm, TYPE_DOUBLE, {.val_d= CFG_STEPS_PER_MM_E }}, // 36 

  /* used for G0 rapid moves and as a cap for all other feedrates */
  { "maximum_feedrate_x", &config.axis[X_AXIS].maximum_feedrate, TYPE_INT, {.val_i= CFG_MAX_FEEDRATE_X}},   /* mm/min */
  { "maximum_feedrate_y", &config.axis[Y_AXIS].maximum_feedrate, TYPE_INT, {.val_i= CFG_MAX_FEEDRATE_Y}},   /* mm/min */
  { "maximum_feedrate_z", &config.axis[Z_AXIS].maximum_feedrate, TYPE_INT, {.val_i= CFG_MAX_FEEDRATE_Z}},   /* mm/min */
  { "maximum_feedrate_e", &config.axis[E_AXIS].maximum_feedrate, TYPE_INT, {.val_i= CFG_MAX_FEEDRATE_E}},   /* mm/min */

  // if axis acceleration is 0, general default acceleration will be used
  { "x.acceleration", &config.axis[X_AXIS].acceleration, TYPE_INT, {.val_i= 0 }},
  { "y.acceleration", &config.axis[Y_AXIS].acceleration, TYPE_INT, {.val_i= 0 }},
  { "z.acceleration", &config.axis[Z_AXIS].acceleration, TYPE_INT, {.val_i= 0 }},
  { "e.acceleration", &config.axis[E_AXIS].acceleration, TYPE_INT, {.val_i= 0 }},

  { "x.dir.invert", &config.axis[X_AXIS].dir_invert, TYPE_INT, {.val_i= 0 }},
  { "y.dir.invert", &config.axis[Y_AXIS].dir_invert, TYPE_INT, {.val_i= 0 }},
  { "z.dir.invert", &config.axis[Z_AXIS].dir_invert, TYPE_INT, {.val_i= 0 }},

  /* used when searching endstops and similar */
  { "search_feedrate_x", &config.axis[X_AXIS].search_feedrate, TYPE_INT, {.val_i= CFG_SEARCH_FEEDRATE_X}},  //120
  { "search_feedrate_y", &config.axis[Y_AXIS].search_feedrate, TYPE_INT, {.val_i= CFG_SEARCH_FEEDRATE_Y}},  //120
  { "search_feedrate_z", &config.axis[Z_AXIS].search_feedrate, TYPE_INT, {.val_i= CFG_SEARCH_FEEDRATE_Z}},   //60
  { "search_feedrate_e", &config.axis[E_AXIS].search_feedrate, TYPE_INT, {.val_i= 0}},
  
  { "homing_feedrate_x", &config.axis[X_AXIS].homing_feedrate, TYPE_INT, {.val_i= CFG_HOMING_FEEDRATE_X }}, //3000
  { "homing_feedrate_y", &config.axis[Y_AXIS].homing_feedrate, TYPE_INT, {.val_i= CFG_HOMING_FEEDRATE_Y }}, //3000
  { "homing_feedrate_z", &config.axis[Z_AXIS].homing_feedrate, TYPE_INT, {.val_i= CFG_HOMING_FEEDRATE_Z }},   //60
  
  // home pos is left front
  { "home_direction_x", &config.axis[X_AXIS].home_direction, TYPE_INT, {.val_i= CFG_HOME_DIRECTION_X }}, 
  { "home_direction_y", &config.axis[Y_AXIS].home_direction, TYPE_INT, {.val_i= CFG_HOME_DIRECTION_Y }},
  { "home_direction_z", &config.axis[Z_AXIS].home_direction, TYPE_INT, {.val_i= CFG_HOME_DIRECTION_Z }},
  
  { "home_pos_x",       &config.axis[X_AXIS].home_pos, TYPE_INT, {.val_i= CFG_HOME_POS_X }},
  { "home_pos_y",       &config.axis[Y_AXIS].home_pos, TYPE_INT, {.val_i= CFG_HOME_POS_Y }},
  { "home_pos_z",       &config.axis[Z_AXIS].home_pos, TYPE_INT, {.val_i= CFG_HOME_POS_Z }},

  { "printing_vol_x",   &config.axis[X_AXIS].printing_vol , TYPE_INT, {.val_i= CFG_PRINTING_VOL_X }},
  { "printing_vol_y",   &config.axis[Y_AXIS].printing_vol , TYPE_INT, {.val_i= CFG_PRINTING_VOL_Y }},
  { "printing_vol_z",   &config.axis[Z_AXIS].printing_vol , TYPE_INT, {.val_i= CFG_PRINTING_VOL_Z }},
  
  //  
  // config for printers
  //

  // dump pos
  { "have_dump_pos",    &config.have_dump_pos , TYPE_INT,   {.val_i= 0}},
  { "dump_pos_x",       &config.dump_pos_x , TYPE_INT,      {.val_i= 0}},
  { "dump_pos_y",       &config.dump_pos_x , TYPE_INT,      {.val_i= 0}},
  
  // rest pos
  { "have_rest_pos",    &config.have_rest_pos , TYPE_INT,   {.val_i=0}},
  { "rest_pos_x",       &config.rest_pos_x , TYPE_INT,      {.val_i=0}},
  { "rest_pos_y",       &config.rest_pos_y , TYPE_INT,      {.val_i=0}},

  // wipe pos
  { "have_wipe_pos",    &config.have_wipe_pos , TYPE_INT,     {.val_i= 0 }},
  { "wipe_entry_pos_x", &config.wipe_entry_pos_x , TYPE_INT,  {.val_i= 0 }},
  { "wipe_entry_pos_y", &config.wipe_entry_pos_y , TYPE_INT,  {.val_i= 0 }},
  { "wipe_pos_x",       &config.wipe_entry_pos_x , TYPE_INT,  {.val_i= 0 }},     // DEPRECATED
  { "wipe_pos_y",       &config.wipe_entry_pos_y , TYPE_INT,  {.val_i= 0 }},     // DEPRECATED
  { "wipe_exit_pos_x",  &config.wipe_exit_pos_x , TYPE_INT,   {.val_i= 0 }},
  { "wipe_exit_pos_y",  &config.wipe_exit_pos_y , TYPE_INT,   {.val_i= 0 }},

  { "steps_per_revolution_e", &config.steps_per_revolution_e, TYPE_INT, {.val_i=3200}},  // 200 * 16
  
  { "wait_on_temp",     &config.wait_on_temp, TYPE_INT,       {.val_i= 0 }},
    
  { "num_extruders",    &config.num_extruders, TYPE_INT,      {.val_i= 1 }},

  { "enable_extruder_0", &config.enable_extruder_0, TYPE_INT, {.val_i= 1 }},
};


// lookup table for pin mapping
// this allows some portability to different setups without affecting application config
// This has default values for the R2C2 v1.2 electronics board
static const tConfigItem config_lookup_pindef [] = 
{

  // Motor 0 (X)
  { "x.pin_step",   &config.motor_driver[0].pin_step,   TYPE_PIN_DEF, {.val_pin_def = S0_STEP_PIN  }},
  { "x.pin_dir",    &config.motor_driver[0].pin_dir,    TYPE_PIN_DEF, {.val_pin_def = S0_DIR_PIN   }},
  { "x.pin_enable", &config.motor_driver[0].pin_enable, TYPE_PIN_DEF, {.val_pin_def = S0_ENABLE_PIN}},
  { "x.pin_reset",  &config.motor_driver[0].pin_reset,  TYPE_PIN_DEF, {.val_pin_def = UNDEFINED_PIN_DEF}},
  
  // Motor 1 (Y)
  { "y.pin_step",   &config.motor_driver[1].pin_step,   TYPE_PIN_DEF, {.val_pin_def = S1_STEP_PIN   }},
  { "y.pin_dir",    &config.motor_driver[1].pin_dir,    TYPE_PIN_DEF, {.val_pin_def = S1_DIR_PIN    }},
  { "y.pin_enable", &config.motor_driver[1].pin_enable, TYPE_PIN_DEF, {.val_pin_def = S1_ENABLE_PIN }},
  { "y.pin_reset",  &config.motor_driver[1].pin_reset,  TYPE_PIN_DEF, {.val_pin_def = UNDEFINED_PIN_DEF}},

  // Motor 2 (Z)
  { "z.pin_step",   &config.motor_driver[2].pin_step,   TYPE_PIN_DEF, {.val_pin_def = S2_STEP_PIN   }},
  { "z.pin_dir",    &config.motor_driver[2].pin_dir,    TYPE_PIN_DEF, {.val_pin_def = S2_DIR_PIN    }},
  { "z.pin_enable", &config.motor_driver[2].pin_enable, TYPE_PIN_DEF, {.val_pin_def = S2_ENABLE_PIN }},
  { "z.pin_reset",  &config.motor_driver[2].pin_reset,  TYPE_PIN_DEF, {.val_pin_def = UNDEFINED_PIN_DEF}},

  // Motor 3 (E)
  { "e.pin_step",   &config.motor_driver[3].pin_step,   TYPE_PIN_DEF, {.val_pin_def = S3_STEP_PIN   }},
  { "e.pin_dir",    &config.motor_driver[3].pin_dir,    TYPE_PIN_DEF, {.val_pin_def = S3_DIR_PIN    }},
  { "e.pin_enable", &config.motor_driver[3].pin_enable, TYPE_PIN_DEF, {.val_pin_def = S3_ENABLE_PIN }},
  { "e.pin_reset",  &config.motor_driver[3].pin_reset,  TYPE_PIN_DEF, {.val_pin_def = UNDEFINED_PIN_DEF}},

#if ( CFG_MAX_EXTRUDERS > 1)  
  { "e1.pin_step",   &config.motor_driver[4].pin_step,   TYPE_PIN_DEF, {.val_pin_def = S4_STEP_PIN   }},
  { "e1.pin_dir",    &config.motor_driver[4].pin_dir,    TYPE_PIN_DEF, {.val_pin_def = S4_DIR_PIN    }},
  { "e1.pin_enable", &config.motor_driver[4].pin_enable, TYPE_PIN_DEF, {.val_pin_def = S4_ENABLE_PIN }},
  { "e1.pin_reset",  &config.motor_driver[4].pin_reset,  TYPE_PIN_DEF, {.val_pin_def = UNDEFINED_PIN_DEF}},
#endif

#if ( CFG_MAX_EXTRUDERS > 2)  
  { "e2.pin_step",   &config.motor_driver[5].pin_step,   TYPE_PIN_DEF, {.val_pin_def = S5_STEP_PIN   }},
  { "e2.pin_dir",    &config.motor_driver[5].pin_dir,    TYPE_PIN_DEF, {.val_pin_def = S5_DIR_PIN    }},
  { "e2.pin_enable", &config.motor_driver[5].pin_enable, TYPE_PIN_DEF, {.val_pin_def = S5_ENABLE_PIN }},
  { "e2.pin_reset",  &config.motor_driver[5].pin_reset,  TYPE_PIN_DEF, {.val_pin_def = UNDEFINED_PIN_DEF}},
#endif

//
  { "x.motor",         &config.axis[X_AXIS].motor_index,  TYPE_INT, {.val_i = X_MOTOR }},

  { "x.pin_min_limit", &config.axis[X_AXIS].pin_min_limit,  TYPE_PIN_DEF, {.val_pin_def = X_MIN_PIN}},
  { "x.pin_max_limit", &config.axis[X_AXIS].pin_max_limit,  TYPE_PIN_DEF, {.val_pin_def = UNDEFINED_PIN_DEF}},
  
  // Y Axis
  { "y.motor",         &config.axis[Y_AXIS].motor_index,  TYPE_INT, {.val_i = Y_MOTOR }},
  { "y.pin_min_limit", &config.axis[Y_AXIS].pin_min_limit,  TYPE_PIN_DEF, {.val_pin_def = Y_MIN_PIN    }},
  { "y.pin_max_limit", &config.axis[Y_AXIS].pin_max_limit,  TYPE_PIN_DEF, {.val_pin_def = UNDEFINED_PIN_DEF}},

  // Z Axis
  { "z.motor",         &config.axis[Z_AXIS].motor_index,  TYPE_INT, {.val_i = Z_MOTOR }},
  { "z.pin_min_limit", &config.axis[Z_AXIS].pin_min_limit,  TYPE_PIN_DEF, {.val_pin_def = Z_MIN_PIN        }},
  { "z.pin_max_limit", &config.axis[Z_AXIS].pin_max_limit,  TYPE_PIN_DEF, {.val_pin_def = UNDEFINED_PIN_DEF}},

  // E Axis
  { "e.motor",         &config.axis[E_AXIS].motor_index,  TYPE_INT, {.val_i = E0_MOTOR }},
  { "e.pin_min_limit", &config.axis[E_AXIS].pin_min_limit,  TYPE_PIN_DEF, {.val_pin_def=UNDEFINED_PIN_DEF}},
  { "e.pin_max_limit", &config.axis[E_AXIS].pin_max_limit,  TYPE_PIN_DEF, {.val_pin_def=UNDEFINED_PIN_DEF}},

#if ( CFG_MAX_EXTRUDERS > 1)  
  // E1 Axis
  { "e1.motor",         &config.axis[E1_AXIS].motor_index,  TYPE_INT, {.val_i = E1_MOTOR }},
  { "e1.pin_min_limit", &config.axis[E1_AXIS].pin_min_limit,  TYPE_PIN_DEF, {.val_pin_def=UNDEFINED_PIN_DEF}},
  { "e1.pin_max_limit", &config.axis[E1_AXIS].pin_max_limit,  TYPE_PIN_DEF, {.val_pin_def=UNDEFINED_PIN_DEF}},
#endif

#if ( CFG_MAX_EXTRUDERS > 2)  
  // E2 Axis
  { "e2.motor",         &config.axis[E2_AXIS].motor_index,  TYPE_INT, {.val_i = E2_MOTOR }},
  { "e2.pin_min_limit", &config.axis[E2_AXIS].pin_min_limit,  TYPE_PIN_DEF, {.val_pin_def=UNDEFINED_PIN_DEF}},
  { "e2.pin_max_limit", &config.axis[E2_AXIS].pin_max_limit,  TYPE_PIN_DEF, {.val_pin_def=UNDEFINED_PIN_DEF}},
#endif


  { "all_steppers.reset",  &config.pin_all_steppers_reset,  TYPE_PIN_DEF, {.val_pin_def = STEPPERS_RESET_PIN}},
  
  { "have_digipot",         &config.have_digipot,  TYPE_INT, {.val_i = CFG_HAVE_DIGIPOT}},
  { "digipot_i2c_channel",  &config.digipot_i2c_channel,  TYPE_INT, {.val_i = CFG_DIGIPOT_I2C_CHAN}},
  
  { "digipot_i2c_scl",  &config.digipot_i2c_scl,  TYPE_PIN_DEF, {.val_pin_def = CFG_PIN_DIGIPOT_I2C_SCL}},
  { "digipot_i2c_sda",  &config.digipot_i2c_sda,  TYPE_PIN_DEF, {.val_pin_def = CFG_PIN_DIGIPOT_I2C_SDA}},
//

#ifdef AUX_0_OUTPUT_PIN
  { "aux_0.pin_output",             &config.aux_output[0].pin_output,           TYPE_PIN_DEF, {.val_pin_def = AUX_0_OUTPUT_PIN}},
#endif
#ifdef AUX_1_OUTPUT_PIN
  { "aux_1.pin_output",             &config.aux_output[1].pin_output,           TYPE_PIN_DEF, {.val_pin_def = AUX_1_OUTPUT_PIN}},
#endif

//
  { "num_sensors",    &config.num_sensors, TYPE_INT,    {.val_i = CFG_MAX_SENSORS}},

  { "sensor_0.temp_sensor",         &config.sensor[0].pin,           TYPE_PIN_DEF, {.val_pin_def = SENSOR_0_ADC_PIN}},      
  { "sensor_0.adc_channel",         &config.sensor[0].adc_channel,   TYPE_INT,     {.val_i       = SENSOR_0_ADC_CHANNEL}},
  { "sensor_0.temp_table",          &config.sensor[0].table_index,   TYPE_INT,     {.val_i       = SENSOR_0_TABLE_INDEX}},

  { "sensor_1.temp_sensor",         &config.sensor[1].pin,           TYPE_PIN_DEF, {.val_pin_def = SENSOR_1_ADC_PIN}},      
  { "sensor_1.adc_channel",         &config.sensor[1].adc_channel,   TYPE_INT,     {.val_i       = SENSOR_1_ADC_CHANNEL}},
  { "sensor_1.temp_table",          &config.sensor[1].table_index,   TYPE_INT,     {.val_i       = SENSOR_1_TABLE_INDEX}},

#if CFG_MAX_SENSORS > 2
  { "sensor_2.temp_sensor",         &config.sensor[2].pin,           TYPE_PIN_DEF, {.val_pin_def = SENSOR_2_ADC_PIN}},      
  { "sensor_2.adc_channel",         &config.sensor[2].adc_channel,   TYPE_INT,     {.val_i       = SENSOR_2_ADC_CHANNEL}},
  { "sensor_2.temp_table",          &config.sensor[2].table_index,   TYPE_INT,     {.val_i       = SENSOR_2_TABLE_INDEX}},
#endif

#if CFG_MAX_SENSORS > 3
  { "sensor_3.temp_sensor",         &config.sensor[3].pin,           TYPE_PIN_DEF, {.val_pin_def = SENSOR_3_ADC_PIN}},      
  { "sensor_3.adc_channel",         &config.sensor[3].adc_channel,   TYPE_INT,     {.val_i       = SENSOR_3_ADC_CHANNEL}},
  { "sensor_3.temp_table",          &config.sensor[3].table_index,   TYPE_INT,     {.val_i       = SENSOR_3_TABLE_INDEX}},
#endif

  // CTC #1 / Extruder 0
  { "extruder_0.heater",              &config.extruder_ctc[0].pin_heater,           TYPE_PIN_DEF, {.val_pin_def = EXTRUDER_0_HEATER_PIN}},
//!  { "extruder_0.cooler",              &config.extruder_ctc[0].pin_cooler,           TYPE_PIN_DEF, {.val_pin_def = EXTRUDER_0_FAN_PIN}},
  { "extruder_0.temp_sensor",         &config.extruder_ctc[0].sensor_index,         TYPE_INT,     {.val_i = EXTRUDER_0_SENSOR_INDEX}},

//  { "extruder_0.temp_sensor",         &config.extruder_ctc[0].pin_temp_sensor,      TYPE_PIN_DEF, {.val_pin_def = EXTRUDER_0_SENSOR_ADC_PIN}},
//  { "extruder_0.sensor_adc_channel",  &config.extruder_ctc[0].sensor_adc_channel,   TYPE_INT,     {.val_i = EXTRUDER_0_SENSOR_ADC_CHANNEL}},

  // CTC #2 / Heated Bed
  { "heated_bed.heater",              &config.heated_bed_ctc.pin_heater,            TYPE_PIN_DEF, {.val_pin_def = HEATED_BED_0_HEATER_PIN}},
//!  { "heated_bed.cooler",              &config.heated_bed_ctc.pin_cooler,            TYPE_PIN_DEF, {.val_pin_def = UNDEFINED_PIN_DEF}},
  { "heated_bed.temp_sensor",         &config.heated_bed_ctc.sensor_index,          TYPE_INT,     {.val_i = HEATED_BED_0_SENSOR_INDEX}},

//  { "heated_bed.temp_sensor",         &config.heated_bed_ctc.pin_temp_sensor,       TYPE_PIN_DEF, {.val_pin_def = HEATED_BED_0_ADC_PIN}},
//  { "heated_bed.sensor_adc_channel",  &config.heated_bed_ctc.sensor_adc_channel,    TYPE_INT,     {.val_i = HEATED_BED_0_SENSOR_ADC_CHANNEL}},

#if ( CFG_MAX_EXTRUDERS > 1)  
  // Extruder 1
  { "extruder_1.heater",              &config.extruder_ctc[1].pin_heater,           TYPE_PIN_DEF, {.val_pin_def = EXTRUDER_1_HEATER_PIN}},
  { "extruder_1.temp_sensor",         &config.extruder_ctc[1].sensor_index,      TYPE_PIN_DEF, {.val_i = EXTRUDER_1_SENSOR_INDEX}},
#endif

#if ( CFG_MAX_EXTRUDERS > 2)  
  // Extruder 2
  { "extruder_2.heater",              &config.extruder_ctc[2].pin_heater,           TYPE_PIN_DEF, {.val_pin_def = EXTRUDER_2_HEATER_PIN}},
  { "extruder_2.temp_sensor",         &config.extruder_ctc[2].sensor_index,      TYPE_PIN_DEF, {.val_i = EXTRUDER_2_SENSOR_INDEX}},
#endif

  { "sd.spi.ssel0",         &config.sd_spi_ssel,   TYPE_PIN_DEF, {.val_pin_def =  SD_SPI_SSEL  }},
  { "sd.spi.sck",           &config.sd_spi_sck,   TYPE_PIN_DEF, {.val_pin_def =  SD_SPI_SCK  }},
  { "sd.spi.mosi",          &config.sd_spi_mosi,    TYPE_PIN_DEF, {.val_pin_def =  SD_SPI_MOSI  }},
  { "sd.spi.miso",          &config.sd_spi_miso,    TYPE_PIN_DEF, {.val_pin_def =  SD_SPI_MISO  }},
  { "sd.spi.channel",       &config.sd_spi_channel,   TYPE_INT,     {.val_i = SD_SPI_CHANNEL}},

  { "buzzer",          &config.buzzer_pin,   TYPE_PIN_DEF, {.val_pin_def = PIN_DEF (BUZZER_PORT, BUZZER_PIN_NUMBER, 1)   }},

  { "cp_lcd_data_0",   &config.interface_cp_lcd_pin_data[0],   TYPE_PIN_DEF, {.val_pin_def = UNDEFINED_PIN_DEF   }},
  { "cp_lcd_data_1",   &config.interface_cp_lcd_pin_data[1],   TYPE_PIN_DEF, {.val_pin_def = UNDEFINED_PIN_DEF   }},
  { "cp_lcd_data_2",   &config.interface_cp_lcd_pin_data[2],   TYPE_PIN_DEF, {.val_pin_def = UNDEFINED_PIN_DEF   }},
  { "cp_lcd_data_3",   &config.interface_cp_lcd_pin_data[3],   TYPE_PIN_DEF, {.val_pin_def = UNDEFINED_PIN_DEF   }},
  { "cp_lcd_data_4",   &config.interface_cp_lcd_pin_data[4],   TYPE_PIN_DEF, {.val_pin_def = UNDEFINED_PIN_DEF   }},
  { "cp_lcd_data_5",   &config.interface_cp_lcd_pin_data[5],   TYPE_PIN_DEF, {.val_pin_def = UNDEFINED_PIN_DEF   }},
  { "cp_lcd_data_6",   &config.interface_cp_lcd_pin_data[6],   TYPE_PIN_DEF, {.val_pin_def = UNDEFINED_PIN_DEF   }},
  { "cp_lcd_data_7",   &config.interface_cp_lcd_pin_data[7],   TYPE_PIN_DEF, {.val_pin_def = UNDEFINED_PIN_DEF   }},
  { "cp_lcd_rw",   &config.interface_cp_lcd_pin_rw,   TYPE_PIN_DEF, {.val_pin_def = UNDEFINED_PIN_DEF   }},
  { "cp_lcd_rs",   &config.interface_cp_lcd_pin_rs,   TYPE_PIN_DEF, {.val_pin_def = UNDEFINED_PIN_DEF   }},
  { "cp_lcd_en",   &config.interface_cp_lcd_pin_en,   TYPE_PIN_DEF, {.val_pin_def = UNDEFINED_PIN_DEF   }},

  { "cp_led_0",   &config.interface_cp_led_pin[0],   TYPE_PIN_DEF, {.val_pin_def = UNDEFINED_PIN_DEF   }},
  { "cp_led_1",   &config.interface_cp_led_pin[1],   TYPE_PIN_DEF, {.val_pin_def = UNDEFINED_PIN_DEF   }},
  { "cp_led_2",   &config.interface_cp_led_pin[2],   TYPE_PIN_DEF, {.val_pin_def = UNDEFINED_PIN_DEF   }},

  { "cp_btn_0",   &config.interface_cp_btn_pin[0],   TYPE_PIN_DEF, {.val_pin_def = PIN_DEF(1,22,1)   }},
  { "cp_btn_1",   &config.interface_cp_btn_pin[1],   TYPE_PIN_DEF, {.val_pin_def = PIN_DEF(1,4,1)   }},
  { "cp_btn_2",   &config.interface_cp_btn_pin[2],   TYPE_PIN_DEF, {.val_pin_def = PIN_DEF(1,8,1)   }},
  { "cp_btn_3",   &config.interface_cp_btn_pin[3],   TYPE_PIN_DEF, {.val_pin_def = PIN_DEF(0,26,1)   }},
  { "cp_btn_4",   &config.interface_cp_btn_pin[4],   TYPE_PIN_DEF, {.val_pin_def = PIN_DEF(2,1,1)   }},
  { "cp_btn_5",   &config.interface_cp_btn_pin[5],   TYPE_PIN_DEF, {.val_pin_def = PIN_DEF(2,0,1)   }},
  { "cp_btn_6",   &config.interface_cp_btn_pin[6],   TYPE_PIN_DEF, {.val_pin_def = PIN_DEF(1,1,1)   }},
  { "cp_btn_7",   &config.interface_cp_btn_pin[7],   TYPE_PIN_DEF, {.val_pin_def = PIN_DEF(1,10,1)    }},
  { "cp_btn_8",   &config.interface_cp_btn_pin[8],   TYPE_PIN_DEF, {.val_pin_def = PIN_DEF(0,9,1)    }},
  { "cp_btn_9",   &config.interface_cp_btn_pin[9],   TYPE_PIN_DEF, {.val_pin_def = UNDEFINED_PIN_DEF   }},


};


static tKeyHash config_keys [NUM_TOKENS(config_lookup)];
static tKeyHash config_pindef_keys [NUM_TOKENS(config_lookup_pindef)];


static tLineBuffer line_buf;

//
//
//
void app_config_set_defaults(void)
{
  set_defaults (config_lookup, NUM_TOKENS(config_lookup));
  
  set_defaults (config_lookup_pindef, NUM_TOKENS(config_lookup_pindef));

  // set default axis map
  config.num_axes = NUM_AXES;

  config.axis[0].letter_code = 'X';
  config.axis[0].is_configured = true;

  config.axis[1].letter_code = 'Y';
  config.axis[1].is_configured = true;

  config.axis[2].letter_code = 'Z';
  config.axis[2].is_configured = true;

  config.axis[3].letter_code = 'E';
  config.axis[3].is_configured = true;

#if CFG_MAX_AXES > 4
  config.axis[4].letter_code = 'B';
  config.axis[4].is_configured = true;
#endif

#if CFG_MAX_AXES > 5
  config.axis[5].letter_code = 'C';
  config.axis[5].is_configured = true;
#endif
}


// read the config files from SD Card
void app_config_read (void)
{
#ifdef HAVE_FILESYSTEM
  FRESULT res;    /* FatFs function common result code */
      
  create_key_hash_table (NUM_TOKENS(config_lookup_pindef), config_lookup_pindef, config_pindef_keys);
  create_key_hash_table (NUM_TOKENS(config_lookup), config_lookup, config_keys);

  res = read_config_file ("conf_pin.txt", config_lookup_pindef, NUM_TOKENS(config_lookup_pindef), config_pindef_keys);

  res = read_config_file ("config.txt", config_lookup, NUM_TOKENS(config_lookup), config_keys);
#endif  
}

// print the config tables
void app_config_print()
{
  print_config_table (config_lookup, NUM_TOKENS(config_lookup) );
}

//TODO: move
// read a file and execute GCode commands
void exec_gcode_file (char *filename)
{
#ifdef HAVE_FILESYSTEM
  char *pLine;
  FIL file;
  FRESULT res;
  tGcodeInputMsg GcodeInputMsg;

  res = f_open(&file, filename, FA_OPEN_EXISTING | FA_READ);
  if (res == FR_OK)
  {
    GcodeInputMsg.pLineBuf = &line_buf;
    GcodeInputMsg.out_file = NULL;
    
    pLine = f_gets(line_buf.data, sizeof(line_buf.data), &file); /* read one line */
    while (pLine)
    {
      line_buf.len = strlen(pLine);
      gcode_parse_line (&GcodeInputMsg);
      pLine = f_gets(line_buf.data, sizeof(line_buf.data), &file); /* read next line */
    }

    /* Close file */
    res = f_close(&file);
    if (res)
      debug("Error closing %s\n", filename);
  }  
#endif
}
