/* Copyright (c) 2012 Rui Ribeiro - racribeiro@gmail.com       */
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


#ifndef	_SOUNDPLAY_H
#define	_SOUNDPLAY_H

#include <stdint.h>

/*
 * For more notes and frequencies
 *
 * http://www.phy.mtu.edu/~suits/notefreqs.html
 *
 */

#define FREQ_C4     262 //Middle C
#define FREQ_C4S    277
#define FREQ_D4F    277
#define FREQ_D4     294
#define FREQ_D4S    311
#define FREQ_E4     329
#define FREQ_F4     349
#define FREQ_G4     392
#define FREQ_A4     440
#define FREQ_B4     494

#define WHOLE_NOTE_TIME 1000

void set_whole_note_time (uint32_t new_note_time);
void play_music_string (char* music);

#endif	/* _SOUNDPLAY_H */