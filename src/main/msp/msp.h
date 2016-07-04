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

#pragma once

typedef struct mspPacket_s {
    sbuf_t buf;
    int16_t cmd;
    int16_t result;
} mspPacket_t;

void mspInit(void);

//
// server
//
int mspProcessCommand(mspPacket_t *command, mspPacket_t *reply);

// return positive for ACK, negative on error, zero for no reply
int mspServerCommandHandler(mspPacket_t *cmd, mspPacket_t *reply);

//
// client
//
void mspProcessReply(mspPacket_t *reply);

// return positive for ACK, negative on error
int mspClientReplyHandler(mspPacket_t *reply);
