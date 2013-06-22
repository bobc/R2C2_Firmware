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
/* Table for NTC EPCOS B57560G104F (100k) and R1 = 274R for Extruder0 */
  
/* {ADC value, temperature} */

{
   {0, 0} , // 0.00 C
   {128, 0} , // 0.00 C
   {256, 0} , // 0.00 C
   {384, 0} , // 0.00 C
   {512, 0} , // 0.00 C
   {640, 0} , // 0.00 C
   {768, 0} , // 0.00 C
   {896, 0} , // 0.00 C
   {1024, 0} , // 0.00 C
   {1152, 0} , // 0.00 C
   {1280, 289} , // 289.70 C
   {1408, 279} , // 279.85 C
   {1536, 270} , // 270.77 C
   {1664, 262} , // 262.23 C
   {1792, 254} , // 254.12 C
   {1920, 246} , // 246.38 C
   {2048, 238} , // 238.92 C
   {2176, 231} , // 231.63 C
   {2304, 224} , // 224.49 C
   {2432, 217} , // 217.42 C
   {2560, 210} , // 210.37 C
   {2688, 203} , // 203.27 C
   {2816, 196} , // 196.06 C
   {2944, 188} , // 188.65 C
   {3072, 180} , // 180.97 C
   {3200, 172} , // 172.86 C
   {3328, 164} , // 164.18 C
   {3456, 154} , // 154.69 C
   {3584, 143} , // 143.92 C
   {3712, 131} , // 131.22 C
   {3840, 115} , // 115.04 C
   {3968, 90} , // 90.63 C
   {4096, 0} , // 0.00 C
}
