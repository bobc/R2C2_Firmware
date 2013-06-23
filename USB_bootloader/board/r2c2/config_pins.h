/* Copyright (c) 2012 Bob Cousins bobcousins42@googlemail.com              */
/* **************************************************************************
   All rights reserved.

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
****************************************************************************/
// **************************************************************************
//
// Description: For R2C2 board
//
// **************************************************************************

#ifndef _CONFIG_PINS_H
#define _CONFIG_PINS_H


// --------------------------------------------------------------------------
// Defines
// --------------------------------------------------------------------------

#define EXTRUDER_0_HEATER_PORT          2        /* P2.4 */
#define EXTRUDER_0_HEATER_PIN           (1 << 4) /* P2.4 */
#define HEATED_BED_0_HEATER_PORT        2        /* P2.5 */
#define HEATED_BED_0_HEATER_PIN         (1 << 5) /* P2.5 */
#define EXTRUDER_0_FAN_PORT             2         /* P2.3 */
#define EXTRUDER_0_FAN_PIN              (1<<3)

#define BUZZER_PORT                     2         /* P2.2 PWM1[3] */
#define BUZZER_PIN                      (1 << 22) /* P2.2 PWM1[3] */

#define STEPPERS_RESET_PORT             0         /* P0.22 */
#define STEPPERS_RESET_PIN              (1 << 22) /* P0.22 */

#define X_ENABLE_PORT                   1         /* P1.24 */
#define X_ENABLE_PIN                    (1 << 24) /* P1.24 */
#define X_STEP_PORT                     1         /* P1.20 */
#define X_STEP_PIN                      (1 << 20) /* P1.20 */

#define Y_ENABLE_PORT                   1         /* P1.28 */
#define Y_ENABLE_PIN                    (1 << 28) /* P1.28 */
#define Y_STEP_PORT                     1         /* P1.25 */
#define Y_STEP_PIN                      (1 << 25) /* P1.25 */

#define Z_ENABLE_PORT                   0         /* P0. 1 */
#define Z_ENABLE_PIN                    (1 <<  1) /* P0. 1 */
#define Z_STEP_PORT                     1         /* P1.29 */
#define Z_STEP_PIN                      (1 << 29) /* P1.29 */

#define E_ENABLE_PORT                   2         /* P2.10 */
#define E_ENABLE_PIN                    (1 << 10) /* P2.10 */
#define E_STEP_PORT                     0         /* P0.10 */
#define E_STEP_PIN                      (1 << 10) /* P0.10 */

/* Bootloader IO button P4.29 */
#define BOOT_BUTTON_PORT    4
#define BOOT_BUTTON_PIN     29

#define SD_SSP                LPC_SSP0
#define SD_SCLK_PORT          0
#define SD_SCLK_PIN           15

#define SD_MISO_PORT          0
#define SD_MISO_PIN           17

#define SD_MOSI_PORT          0
#define SD_MOSI_PIN           18

#define SD_SSEL_PORT          0
#define SD_SSEL_PIN           16

// SSP channel, func num

// uart num, func num
#define DBG_UART                LPC_UART0
#define DBG_UART_NUM            0
#define DBG_UART_PORT           0
#define DBG_UART_TX_PIN         2
#define DBG_UART_RX_PIN         3


// --------------------------------------------------------------------------
// Types
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Public Variables
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Public functions
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
//
// --------------------------------------------------------------------------

#endif // _CONFIG_PINS_H

