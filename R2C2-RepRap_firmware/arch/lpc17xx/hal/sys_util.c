/* ========================================================================== */
/*                                                                            */
/*   Filename.c                                                               */
/*   (c) 2001 Author                                                          */
/*                                                                            */
/*   Description                                                              */
/*                                                                            */
/* ========================================================================== */


#include "sys_util.h"

#include "LPC17xx.h"    // NVIC_SystemReset
#include "lpc17xx_nvic.h"

#define USER_FLASH_START 0x10000 /* For USB bootloader */
//#define USER_FLASH_START 0x0 /* No USB bootloader */

void sys_initialise (void)
{
  // DeInit NVIC and SCBNVIC
  NVIC_DeInit();
  NVIC_SCBDeInit();

  /* Configure the NVIC Preemption Priority Bits:
   * two (2) bits of preemption priority, six (6) bits of sub-priority.
   * Since the Number of Bits used for Priority Levels is five (5), so the
   * actual bit number of sub-priority is three (3)
   */
  // FreeRTOS requires 0 it seems
  NVIC_SetPriorityGrouping(0x0);

  /* Change the Vector Table to the USER_FLASH_START
     in case the user application uses interrupts */
  SCB->VTOR = (USER_FLASH_START & 0x1FFFFF80);
}

void sys_reboot (void)
{
  NVIC_SystemReset();
}

