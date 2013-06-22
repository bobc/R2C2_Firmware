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

// hal
#include "adc.h"
#include "timer.h"

// app
#include "app_config.h"
#include "temp.h"
#include "debug.h"


/* Table for NTC EPCOS B57560G104F and R1 = 330R for Extruder0
 * Table for NTC EPCOS B57560G104F and R1 = 12K for HeatedBed0 */
 // 274
 // 10k

// config
#include "thermistor_tables.h"

// todo:
tTempLookupEntry temptable [NUMBER_OF_SENSORS] [NUM_TEMPS] = {
    #include "thermistor_extruder.h"
,
    #include "thermistor_heatbed.h"
};

uint16_t current_temp [NUMBER_OF_SENSORS] = {0};


static uint8_t adc_channel [NUMBER_OF_SENSORS] = {0};
static uint32_t adc_filtered [NUMBER_OF_SENSORS] = {0};

#ifndef	ABSDELTA
#define	ABSDELTA(a, b)	(((a) >= (b))?((a) - (b)):((b) - (a)))
#endif

static uint16_t read_temp(uint8_t sensor_number);



static tTimer  temperatureTimer;


void temperatureTimerCallback (tTimer *pTimer)
{
  /* Read and average temperatures */
  for (int j=0; j < NUMBER_OF_SENSORS; j++)
    current_temp[j] = read_temp(j);
}


uint16_t temp_get(uint8_t sensor_number)
{
  return current_temp[sensor_number];
}


void temp_init (void)
{
  adc_channel [EXTRUDER_0]   = config.extruder_ctc[0].sensor_adc_channel;
  adc_channel [HEATED_BED_0] = config.heated_bed_ctc.sensor_adc_channel;

  AddSlowTimer (&temperatureTimer);
  StartSlowTimer (&temperatureTimer, 10, temperatureTimerCallback);
  temperatureTimer.AutoReload = 1;
}

/* Read and average the ADC input signal */
static uint16_t read_temp (uint8_t sensor_number)
{
  int32_t raw = 0;
  int16_t celsius = 0;
  uint8_t i;

  raw = analog_read(adc_channel[sensor_number]);
  
  // filter the ADC values with simple IIR
  adc_filtered[sensor_number] = ((adc_filtered[sensor_number] * 7) + raw) / 8;
  
  raw = adc_filtered[sensor_number];
  
  /* Go and use the temperature table to math the temperature value... */
  if (raw < temptable[sensor_number][0].adc_value) /* Limit the smaller value... */
  {
    celsius = temptable[sensor_number][0].temperature;
  }
  else if (raw >= temptable[sensor_number][NUM_TEMPS-1].adc_value) /* Limit the higher value... */
  {
    celsius = temptable[sensor_number][NUM_TEMPS-1].temperature;
  }
  else
  {
    for (i=1; i<NUM_TEMPS; i++)
    {
      if (raw < temptable[sensor_number][i].adc_value)
      {
        celsius = temptable[sensor_number][i-1].temperature +
            (raw - temptable[sensor_number][i-1].adc_value) *
            (temptable[sensor_number][i].temperature - temptable[sensor_number][i-1].temperature) /
            (temptable[sensor_number][i].adc_value - temptable[sensor_number][i-1].adc_value);

        break;
      }
    }
  }

  return celsius;
}

bool temp_set_table_entry (uint8_t sensor_number, uint16_t temp, uint16_t adc_val)
{
  if (sensor_number < NUMBER_OF_SENSORS)
  {
    for (int entry=0; entry < NUM_TEMPS; entry++)
    {
      if (temptable[sensor_number][entry].temperature == temp)
      {
        temptable[sensor_number][entry].adc_value = adc_val;
        return true;
      }
    }
    return false;
  }
  else
    return false;
}

uint16_t temp_get_table_entry (uint8_t sensor_number, uint16_t temp)
{
  uint16_t result = 0xffff;
  
  if (sensor_number < NUMBER_OF_SENSORS)
  {
    for (int entry=0; entry < NUM_TEMPS; entry++)
    {
      if (temptable[sensor_number][entry].temperature == temp)
      {
        result = temptable[sensor_number][entry].adc_value;
        break;
      }
    }
  }
  return result;
}
