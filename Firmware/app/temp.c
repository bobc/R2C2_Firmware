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
#include "hal_adc.h"
#include "timer.h"

// app
#include "app_config.h"
#include "temp.h"
#include "debug.h"

#include "thermistor_tables.h"

typedef struct {
    uint8_t adc_channel;
    uint8_t table_index;

    uint16_t current_temp;    
    uint32_t adc_filtered;
} tSensor;

static tSensor sensor_data [CFG_MAX_SENSORS];

static tTimer  temperatureTimer;

#ifndef	ABSDELTA
#define	ABSDELTA(a, b)	(((a) >= (b))?((a) - (b)):((b) - (a)))
#endif

static uint16_t read_temp (uint8_t sensor_number);



void temperatureTimerCallback (tTimer *pTimer)
{
  /* Read and average temperatures */
  for (int j=0; j < CFG_MAX_SENSORS; j++)
    sensor_data[j].current_temp = read_temp(j);
}


uint16_t temp_get(uint8_t sensor_number)
{
  return sensor_data[sensor_number].current_temp;
}

uint16_t temp_get_raw (uint8_t sensor_number)
{
  return sensor_data[sensor_number].adc_filtered;
}


void temp_init (void)
{
  uint16_t j;

//TODO:
  for (j=0; j < CFG_MAX_SENSORS; j++)
  {
    sensor_data [j].adc_channel = config.sensor[j].adc_channel;
    sensor_data [j].table_index = config.sensor[j].table_index;

    // setup ADC
    hal_adc_configure_pin (config.sensor[j].pin);
  }


//
  AddSlowTimer (&temperatureTimer);
  StartSlowTimer (&temperatureTimer, 10, temperatureTimerCallback); // every 10 ms
  temperatureTimer.AutoReload = 1;
}

/* Read and average the ADC input signal */
static uint16_t read_temp (uint8_t sensor_number)
{
  int32_t raw = 0;
  int16_t celsius = 0;
  uint8_t i;
  tTempLookupEntry *pTempTable = thermistor_table[sensor_data[sensor_number].table_index];

  raw = hal_analog_read(sensor_data[sensor_number].adc_channel);
  
  // filter the ADC values with simple IIR
  sensor_data[sensor_number].adc_filtered = ((sensor_data[sensor_number].adc_filtered * 7) + raw) / 8;
  
  raw = sensor_data[sensor_number].adc_filtered;
  
  pTempTable = thermistor_table[sensor_data[sensor_number].table_index];

  /* Go and use the temperature table to math the temperature value... */
  if (raw < pTempTable[0].adc_value) /* Limit the smaller value... */
  {
    celsius = pTempTable[0].temperature;
  }
  else if (raw >= pTempTable[NUM_TEMPS-1].adc_value) /* Limit the higher value... */
  {
    celsius = pTempTable[NUM_TEMPS-1].temperature;
  }
  else
  {
    for (i=1; i<NUM_TEMPS; i++)
    {
      if (raw < pTempTable[i].adc_value)
      {
        celsius = pTempTable[i-1].temperature +
            (raw - pTempTable[i-1].adc_value) *
            (pTempTable[i].temperature - pTempTable[i-1].temperature) /
            (pTempTable[i].adc_value - pTempTable[i-1].adc_value);

        break;
      }
    }
  }

  return celsius;
}

bool temp_set_table_entry (uint8_t table_number, uint16_t temp, uint16_t adc_val)
{
  if (table_number < NUMBER_OF_TABLES)
  {
    tTempLookupEntry *pTempTable = thermistor_table[table_number];

    for (int entry=0; entry < NUM_TEMPS; entry++)
    {
      if (pTempTable[entry].temperature == temp)
      {
        pTempTable[entry].adc_value = adc_val;
        return true;
      }
    }
    return false;
  }
  else
    return false;
}

uint16_t temp_get_table_entry (uint8_t table_number, uint16_t temp)
{
  uint16_t result = 0xffff;
  
  if (table_number < NUMBER_OF_TABLES)
  {
    tTempLookupEntry *pTempTable = thermistor_table[table_number];

    for (int entry=0; entry < NUM_TEMPS; entry++)
    {
      if (pTempTable[entry].temperature == temp)
      {
        result = pTempTable[entry].adc_value;
        break;
      }
    }
  }
  return result;
}
