/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"
#include "version.h"

#include "config/config.h"
#include "config/runtime_config.h"

#include "flight/failsafe.h"

#include "io/rc_controls.h"

#include "rx/rx.h"

/*

 This is short status line generation

*/

#include "status.h"

/*
 Copy with inverted
*/

void strcpyi(char* dst, char* src)
{
  // warning - no boundary checks!
  uint8_t i = 0;
  while (src[i]) {
    dst[i] = src[i] | 0x80;
    i++;
  }
}

/*

Composes short status message
buffer - place to write to
size - size of the buffer

*/

void composeStatus(char * buffer, uint8_t size)
{
  // let's compose status message
  // inverted characters are used to report
  // errors or warnings

  // let's start with failsafe

  uint8_t length = size;
  uint8_t statusTooLong = 0;
  buffer[length-1] = 0x00;
  buffer[0] = '?' | 0x80;
  switch (failsafePhase()) {
      case FAILSAFE_IDLE:
          buffer[0] = '-';
          break;
      case FAILSAFE_RX_LOSS_DETECTED:
          buffer[0] = 'R' | 0x80;
          break;
      case FAILSAFE_LANDING:
          buffer[0] = 'l' | 0x80;
          break;
      case FAILSAFE_LANDED:
          buffer[0] = 'L' | 0x80;
          break;
      case FAILSAFE_RX_LOSS_MONITORING:
          buffer[0] = 'M';
          break;
      case FAILSAFE_RX_LOSS_RECOVERED:
          buffer[0] = 'r';
          break;
  }

  // receive status
  buffer[1] = rxIsReceivingSignal() ? 'R' : ('!' | 0x80);

  // now is time check for ARMING status
  length = 2;
  uint8_t i;
  // iterate trough the flags
  for (i=0;i<4;i++) {
    // check if flag is set
    if ((1<<i) & (armingFlags)) {
      // check if we have enough space in line
      if ((length+2+strlen(armingFlagDescription[i].shortDescription))<(size)) {
        buffer[length] = ' ';
        if (armingFlagDescription[i].inverted)
          strcpyi(buffer+length+1,(char *)armingFlagDescription[i].shortDescription);
          else strcpy(buffer+length+1,armingFlagDescription[i].shortDescription);
        length += strlen(armingFlagDescription[i].shortDescription)+1;
      } else {
        // set too long flag
        statusTooLong = 1;
      }
    }
  }

  // airmode hack
  // why is not airmode in the flight mode flags?

  if (isAirmodeActive()) {
    if ((length+5)<(size)) {
      buffer[length] = ' ';
      buffer[length+1] = 'A' | 0x80;
      buffer[length+2] = 'I' | 0x80;
      buffer[length+3] = 'R' | 0x80;
      length += 4;
    } else {
      // set too long flag
      statusTooLong = 1;
    }
  }
  // now is time to report flight modes
  // iterate trough the flags
  for (i=0;i<12;i++) {
    // check if flag is set
    if ((1<<i) & (flightModeFlags)) {
      // check if we have enough space in line
      if ((length+2+strlen(flightFlagDescription[i].shortDescription))<(size)) {
        buffer[length] = ' ';
        if (flightFlagDescription[i].inverted)
          strcpyi(buffer+length+1,(char *)flightFlagDescription[i].shortDescription);
          else strcpy(buffer+length+1,flightFlagDescription[i].shortDescription);
        length += strlen(flightFlagDescription[i].shortDescription)+1;
      } else {
        // set too long flag
        statusTooLong = 1;
      }
    }
  }

  // ..and state flags
  // iterate trough the flags
  for (i=0;i<5;i++) {
    // check if flag is set
    if ((1<<i) & (stateFlags)) {
      // check if we have enough space in line
      if ((length+2+strlen(stateFlagDescription[i].shortDescription))<(size)) {
        buffer[length] = ' ';
        if (stateFlagDescription[i].inverted)
          strcpyi(buffer+length+1,(char *)stateFlagDescription[i].shortDescription);
          else strcpy(buffer+length+1,stateFlagDescription[i].shortDescription);
        length += strlen(stateFlagDescription[i].shortDescription)+1;
      } else {
        // set too long flag
        statusTooLong = 1;
      }
    }
  }

  // pad the rest of the buffer with spaces (poor mans erase of display)
  while (length < size - 1) {
      buffer[length++] = ' ';
  }
  //mark end of string
  buffer[length] = 0;

  // check for too long status line and report it
  if (statusTooLong) {
    buffer[length-1] = '~' | 0x80;
  }

}
