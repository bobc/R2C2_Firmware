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

/* table for Heatbed */ 
/* Table for NTC EPCOS B57560G104F (100k) and R1 = 10K for HeatedBed */

/* {ADC value, temperature} */

{
   {0, 0} , // 0.00 C
   {128, 229} , // 229.48 C
   {256, 191} , // 191.11 C
   {384, 170} , // 170.39 C
   {512, 156} , // 156.23 C
   {640, 145} , // 145.39 C
   {768, 136} , // 136.65 C
   {896, 129} , // 129.17 C
   {1024, 122} , // 122.68 C
   {1152, 116} , // 116.85 C
   {1280, 111} , // 111.53 C
   {1408, 106} , // 106.62 C
   {1536, 102} , // 102.02 C
   {1664, 97} , // 97.67 C
   {1792, 93} , // 93.50 C
   {1920, 89} , // 89.47 C
   {2048, 85} , // 85.56 C
   {2176, 81} , // 81.73 C
   {2304, 77} , // 77.93 C
   {2432, 74} , // 74.15 C
   {2560, 70} , // 70.35 C
   {2688, 66} , // 66.49 C
   {2816, 62} , // 62.53 C
   {2944, 58} , // 58.45 C
   {3072, 54} , // 54.19 C
   {3200, 49} , // 49.67 C
   {3328, 44} , // 44.77 C
   {3456, 39} , // 39.32 C
   {3584, 33} , // 33.09 C
   {3712, 25} , // 25.71 C
   {3840, 16} , // 16.09 C
   {3968, 1} , // 1.33 C
   {4096, 0} , // 0.00 C
}
