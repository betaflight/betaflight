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

#include <sys/types.h>

#define RX_BUFFER_SIZE    1400
#define TX_BUFFER_SIZE    1400

enum fdmFlags {
    FDM_READ = 1 << 0,
    FDM_WRITE = 1 << 1,
    FDM_EXCEPT = 1 << 2,
    FDM_INTR = 1 << 3,
    FDM_SHUTDOWN = 1 << 4,
};

typedef struct fdmFd {
    struct fdmFd *next;

    int rfd, wfd;
    pid_t pid;
    enum fdmFlags flags;
    void (*readCallback)(struct fdmFd* f);
    void (*writeCallback)(struct fdmFd* f);
    void (*exceptCallback)(struct fdmFd* f);
    void (*intrCallback)(struct fdmFd* f);
} fdmFd;

typedef struct {
    serialPort_t port;
    uint8_t rxBuffer[RX_BUFFER_SIZE];
    uint8_t txBuffer[TX_BUFFER_SIZE];

    bool initialized;
    bool buffering;
    uint8_t id;

    fdmFd fdmFd;
} tcpPort_t;

serialPort_t *serTcpOpen(int id, serialReceiveCallbackPtr rxCallback, uint32_t baudRate, portMode_t mode, portOptions_t options);
void fdmInit(void);
void fdmStop(void);
