/* ========================================================================== */
/*                                                                            */
/*   Filename.c                                                               */
/*   (c) 2001 Author                                                          */
/*                                                                            */
/*   Description                                                              */
/*                                                                            */
/* ========================================================================== */

#include "rtos_api.h"

#include "lpc17xx_pinsel.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_wdt.h"
#include "lpc17xx_adc.h"

#include "app_config.h"


void ctc_init (tCtcSettings *ctc_config)
{
  PINSEL_CFG_Type PinCfg;

  // heater output pin
  set_pin_mode (ctc_config->pin_heater, OUTPUT);
  write_pin (ctc_config->pin_heater, DISABLE);

  // fan output pin
  set_pin_mode(ctc_config->pin_cooler, OUTPUT);
  write_pin (ctc_config->pin_cooler, DISABLE);


  PinCfg.Funcnum = PINSEL_FUNC_2; /* ADC function */
  PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
  PinCfg.Pinmode = PINSEL_PINMODE_TRISTATE;
  PinCfg.Portnum = ctc_config->pin_temp_sensor.port;
  PinCfg.Pinnum = ctc_config->pin_temp_sensor.pin_number;
  PINSEL_ConfigPin(&PinCfg);

}