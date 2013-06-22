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
// Description:
//
// **************************************************************************

// No guard because this file may be included more than once

/* table for Extruder */ 

// rt: 100000
// r2: 4700
// max adc: 4096

/* {ADC value, temperature} */

{
   {0, 0} , // 0.00 C
   {128, 276} , // 276.17 C
   {256, 231} , // 231.15 C
   {384, 207} , // 207.13 C
   {512, 190} , // 190.75 C
   {640, 178} , // 178.35 C
   {768, 168} , // 168.28 C
   {896, 159} , // 159.72 C
   {1024, 152} , // 152.34 C
   {1152, 145} , // 145.68 C
   {1280, 139} , // 139.65 C
   {1408, 134} , // 134.10 C
   {1536, 128} , // 128.89 C
   {1664, 123} , // 123.97 C
   {1792, 119} , // 119.26 C
   {1920, 114} , // 114.73 C
   {2048, 110} , // 110.33 C
   {2176, 106} , // 106.02 C
   {2304, 101} , // 101.75 C
   {2432, 97} , // 97.50 C
   {2560, 93} , // 93.24 C
   {2688, 88} , // 88.92 C
   {2816, 84} , // 84.51 C
   {2944, 79} , // 79.97 C
   {3072, 75} , // 75.20 C
   {3200, 70} , // 70.15 C
   {3328, 64} , // 64.68 C
   {3456, 58} , // 58.62 C
   {3584, 51} , // 51.73 C
   {3712, 43} , // 43.53 C
   {3840, 32} , // 32.90 C
   {3968, 16} , // 16.68 C
   {4096, 0} , // 0.00 C
}