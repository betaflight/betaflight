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

/*
 * Authors:
 * Dominic Clifton - Serial port abstraction, Separation of common STM32 code for cleanflight, various cleanups.
 * Hamasaki/Timecop - Initial baseflight code
 */

// pipe2
#define _GNU_SOURCE

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <pthread.h>
#include <sys/select.h>
#include <sys/wait.h>
#include <signal.h>
#include <fcntl.h>
#include <unistd.h>

#include "platform.h"

#include "build/build_config.h"

#include "common/utils.h"
#include "common/maths.h"

#include "io/serial.h"
#include "serial_tcp.h"

static const struct serialPortVTable tcpVTable; // forward declaration, defined at end of file
static tcpPort_t tcpSerialPorts[SERIAL_PORT_COUNT];

// file descriptor manager

static pthread_mutex_t fdmMutex;
static pthread_t fdmWorker;
static volatile bool fdmWorkerRunning;
struct fdmFd *fdmFdHead = NULL;

void signal_empty_handler(int sig)
{
    UNUSED(sig);
}

static void* fdmWorkerFn(void* data)
{
    UNUSED(data);
    fd_set rfds, wfds, efds;

    sigset_t sset;
    sigemptyset(&sset);
    sigaddset(&sset, SIGUSR1);
    pthread_sigmask(SIG_BLOCK, &sset, NULL);

    sigemptyset(&sset);    // don't block signals in pselect

    // long timeout - signal is used to exit pselect
    const struct timespec timeout = {5,0};

    pthread_mutex_lock(&fdmMutex);
    while (fdmWorkerRunning) {
        // set select flags
        FD_ZERO(&rfds);
        FD_ZERO(&wfds);
        FD_ZERO(&efds);
        int fdMax=-1;
        for (struct fdmFd *f = fdmFdHead; f; f = f->next) {
            if (f->flags & FDM_READ && f->rfd >= 0) {
                fdMax = MAX(fdMax, f->rfd);
                FD_SET(f->rfd, &rfds);
            }
            if (f->flags & FDM_WRITE && f->wfd >= 0) {
                fdMax = MAX(fdMax, f->wfd);
                FD_SET(f->wfd, &wfds);
            }
            if (f->flags & FDM_EXCEPT) {
                // catch exceprions on both descriptors
                if (f->rfd >= 0) {
                    fdMax = MAX(fdMax, f->rfd);
                    FD_SET(f->rfd, &efds);
                }
                if (f->wfd >= 0) {
                    fdMax = MAX(fdMax, f->wfd);
                    FD_SET(f->wfd, &efds);
                }
            }
        }

        pthread_mutex_unlock(&fdmMutex);
        int count = pselect(fdMax + 1, &rfds, &wfds, &efds, &timeout, &sset);
        pthread_mutex_lock(&fdmMutex);
        if(count < 0 && errno != EINTR) {
            fprintf(stderr, "pselect failed: %s\n", strerror(errno));
        }
        for (fdmFd *f = fdmFdHead; f; f = f->next) {
            if(count > 0) {  // flags are not cleared on error(EINTR)
                if(f->flags & FDM_READ && f->rfd >=0 && FD_ISSET(f->rfd, &rfds)) {
                    f->readCallback(f);
                }
                if(f->flags & FDM_WRITE && f->wfd >=0 && FD_ISSET(f->wfd, &wfds)) {
                    f->writeCallback(f);
                }
                if(f->flags & FDM_EXCEPT
                   && ((f->rfd >=0 && FD_ISSET(f->rfd, &efds))
                       || (f->wfd >=0 && FD_ISSET(f->wfd, &efds)) )) {
                    f->exceptCallback(f);
                }
            }
            if(f->flags & FDM_INTR) {
                f->flags &= ~FDM_INTR;
                f->intrCallback(f);
            }
        }
        pid_t pid;
        int status;
        while ((pid = waitpid(-1, &status, WNOHANG)) > 0) {
            if (WIFEXITED(status) || WIFSIGNALED(status)) {
                fdmFd *chld = NULL;
                fprintf(stderr, "Child %d terminated\n", pid);
                for (fdmFd *f = fdmFdHead; f; f = f->next) {
                    if (f->pid == pid) {
                        chld = f; break;
                    }
                }
                if (chld) {
                    chld->pid = -1;
                }
            }
        }
    }
    pthread_mutex_unlock(&fdmMutex);
    printf("FDM worker thread finished\n");
    return NULL;
}

// signal worker thread (generate INTR or update masks)
void fdmWorkerSignal(void)
{
    // signal is blocked outside pselect, it's safe to send it without acquiring mutex
    if(fdmWorker)
        pthread_kill(fdmWorker, SIGUSR1);
}

// register fdmFd
void fdmFdAdd(fdmFd *f)
{
    pthread_mutex_lock(&fdmMutex);
    for (fdmFd *fi = fdmFdHead; fi; fi = f->next) {
        if(f == fi) {
            fprintf(stderr, "fdmFd already registered\n");
            return;
        }
    }
    f->next = fdmFdHead;
    fdmFdHead = f;
    pthread_mutex_unlock(&fdmMutex);
    fdmWorkerSignal();
}

// remove fdmFd
void fdmFdDel(fdmFd *f)
{
    pthread_mutex_lock(&fdmMutex);
    for(fdmFd **fi = &fdmFdHead; *fi; fi = &(*fi)->next)
        if(*fi == f) {
            *fi = f->next;
            return;
        }
    fprintf(stderr, "fdmFd not found\n");
    pthread_mutex_unlock(&fdmMutex);
    fdmWorkerSignal();
}

// close Fdm
// No attempt is made to handle queued data  (should be OK in typical FC use case)
void fdmFdShutdown(fdmFd *f)
{
    pthread_mutex_lock(&fdmMutex);
    if(f->wfd >= 0) {
        if(close(f->wfd))
            fprintf(stderr, "fdmFd wfd close(): %s\n", strerror(errno));
        f->wfd = -1;
    }
     if(f->rfd >= 0) {
        if(close(f->rfd))
            fprintf(stderr, "fdmFd rfd close(): %s\n", strerror(errno));
        f->rfd = -1;
    }
    f->flags |= FDM_SHUTDOWN;
    pthread_mutex_unlock(&fdmMutex);
}

// generate interrupt
void fdmFdIntr(fdmFd *f)
{
    pthread_mutex_lock(&fdmMutex);
    f->flags |= FDM_INTR;
    pthread_mutex_unlock(&fdmMutex);
    fdmWorkerSignal();
}

int fdmFdRead(fdmFd *f, void* buf, size_t count)
{
    int ret;
    pthread_mutex_lock(&fdmMutex);
    if (f->rfd >= 0) {
        ret = read(f->rfd, buf, count);
    } else {
        ret = 0;
    }
    if (ret < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            // nag user - read should be called only when something is available
            fprintf(stderr, "fdmFd read() would block: %s\n", strerror(errno));
            ret = 0;
        } else {
            fprintf(stderr, "fdmFd read(): %s\n", strerror(errno));
            fdmFdShutdown(f);
        }
    } else if(ret == 0) {
        // EOF - other side closed the pipe
        fprintf(stderr, "fdmFd read EOF\n");
        fdmFdShutdown(f);
    }
    pthread_mutex_unlock(&fdmMutex);
    return ret;
}

int fdmFdWrite(fdmFd *f, const void* buf, size_t count)
{
    bool signal = false;
    int ret = 0;
    pthread_mutex_lock(&fdmMutex);
    do {
        if(count == 0) {
            // writer finished, stop waiting for fd write
            f->flags &= ~FDM_WRITE;   // no need to signal it
            break;
        }
        if (f->wfd < 0)
            break;
        ret = write(f->wfd, buf, count);
        if (ret < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                // handle it as if nothing was written
                fprintf(stderr, "fdmFd write() would block: %s\n", strerror(errno));
                ret = 0;
            } else if (errno == EPIPE) {
                // child closed pipe
                fprintf(stderr, "fdmFd write(): pipe closed\n");
                fdmFdShutdown(f);
                break;
            } else {
                fprintf(stderr, "fdmFd read(): %s\n", strerror(errno));
                fdmFdShutdown(f);
                break;
            }
        }
        if(ret < (int)count) {
            // partial write. Enable waiting on FD
            f->flags |= FDM_WRITE;
            signal = true;
        }
    } while(0);
    pthread_mutex_unlock(&fdmMutex);
    if(signal)
        fdmWorkerSignal();
    return ret;
}

void fdmInit(void)
{
    printf("fdmInit()\n");
    pthread_mutexattr_t attr;
    pthread_mutexattr_init(&attr);
    pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
    if (pthread_mutex_init(&fdmMutex, &attr) != 0) {
        printf("fdmInit: fdmMutex init: %s\n", strerror(errno));
        exit(1);
    }
    // register signal handler
    signal(SIGUSR1, &signal_empty_handler);

    fdmWorkerRunning = true;
}

void fdmStart(void)
{
    if(pthread_create(&fdmWorker, NULL, &fdmWorkerFn, NULL)) {
        fprintf(stderr, "fdmInit: fdmWorker create: %s\n", strerror(errno));
        exit(1);
    }
}

void fdmStop(void)
{
    printf("fdmStop()\n");
    fdmWorkerRunning = false;
    fdmWorkerSignal();
    pthread_join(fdmWorker, NULL);
    printf("fdmStop() done\n");
}

// bridge file desccriptors to serial code
void tcpWriteCb(struct fdmFd* f);
void tcpReadCb(struct fdmFd* f);
void tcpIntrCb(struct fdmFd* f);
void tcpDataOut(tcpPort_t *s);

static bool tcpPortInit(tcpPort_t *s, int id, uint32_t baudRate, portMode_t mode, portOptions_t options)
{
    if(s->initialized) {
        fprintf(stderr, "UART%u: port is already initialized!\n", s->id + 1);
        return true;
    }

    s->initialized = true;
    s->id = id;

    // create child process

    int stdin_pipe[2], stdout_pipe[2];
    pipe2(stdin_pipe, O_CLOEXEC);
    pipe2(stdout_pipe, O_CLOEXEC);

    pid_t pid = fork();
    if(pid < 0) {
        fprintf(stderr, "UART%u: fork() failed: %s\n", s->id + 1, strerror(errno));
        return false;
    } else if (pid == 0) {
        // in child process
        dup2(stdin_pipe[0], STDIN_FILENO);
        dup2(stdout_pipe[1], STDOUT_FILENO);
        // O_CLOEXEC - no need to close parent's end

        // build arguments for uart handler
        char pport[10];
        snprintf(pport, sizeof(pport), "%d", id + 1);
        char pbaud[10];
        snprintf(pbaud, sizeof(pbaud), "%d", baudRate);
        char pmode[5];
        snprintf(pmode, sizeof(pmode), "%s%s",
                 (mode & MODE_RX) ? "RX" : "",
                 (mode & MODE_RX) ? "TX" : "");
        char poptions[100];
        static struct {
            portOptions_t opt;
            const char* name;
        } optmap[] = {
            {SERIAL_INVERTED, "inverted"},
            {SERIAL_STOPBITS_2, "stopbits_2"},
            {SERIAL_PARITY_EVEN, "parity_even"},
            {SERIAL_BIDIR, "bidir"},
            {SERIAL_BIDIR_PP, "bidir_pp"}
        };
        char *p = poptions;
        for(unsigned i = 0; i < ARRAYLEN(optmap); i++)
            if(options & optmap[i].opt)
                p += snprintf(p, ARRAYEND(poptions) - p, "%s%s", (p > poptions) ? ",":"", optmap[i].name);
        *p = '\0';

        execl("./uart", "./uart",
              "--port", pport,
              "--baud", pbaud,
              "--mode", pmode,
              "--options", poptions,
              (char*) NULL);
        fprintf(stderr, "UART%u: can't execute UART handler '%s' : %s\n", s->id + 1, "./uart", strerror(errno));
        exit(1);
    }
    // parent process
    // close child's ends of pipes
    close(stdin_pipe[0]);
    close(stdout_pipe[1]);

    s->fdmFd.rfd = stdout_pipe[0];
    s->fdmFd.wfd = stdin_pipe[1];
    s->fdmFd.flags = FDM_READ | FDM_EXCEPT;
    s->fdmFd.pid = pid;
    s->fdmFd.readCallback = &tcpReadCb;
    s->fdmFd.writeCallback = &tcpWriteCb;
    s->fdmFd.intrCallback = &tcpIntrCb;

    fdmFdAdd(&s->fdmFd);

    return true;
}

serialPort_t *serTcpOpen(int id, serialReceiveCallbackPtr rxCallback, uint32_t baudRate, portMode_t mode, portOptions_t options)
{
#if !(defined(USE_UART1) || defined(USE_UART2) || defined(USE_UART3) || defined(USE_UART4) || defined(USE_UART5) || defined(USE_UART6) || defined(USE_UART7) || defined(USE_UART8))
    return NULL;
#endif
    if (id < 0 || id >= SERIAL_PORT_COUNT)
        return NULL;
    tcpPort_t *s = &tcpSerialPorts[id];
    if(!tcpPortInit(s, id, baudRate, mode, options))
        return NULL;

    serialImplOpen(&s->port, mode, &tcpVTable,
                   s->rxBuffer, sizeof(s->rxBuffer), s->txBuffer, sizeof(s->txBuffer),
                   rxCallback);

    s->port.baudRate = baudRate;
    s->port.options = options;

    return &s->port;
}

void tcpKickTx(serialPort_t *instance)
{
    tcpPort_t *s = container_of(instance, tcpPort_t, port);
    if(!s->buffering)
        fdmFdIntr(&s->fdmFd);
}

static void tcpBeginWrite(serialPort_t *instance)
{
    tcpPort_t *s = container_of(instance, tcpPort_t, port);
    if(s->buffering)
        fprintf(stderr, "UART%d: tcpBeginWrite called twice\n", s->id + 1);
    s->buffering = true;
}

static void tcpEndWrite(serialPort_t *instance)
{
    tcpPort_t *s = container_of(instance, tcpPort_t, port);
    if(!s->buffering)
        fprintf(stderr, "UART%d: tcpEndWrite called twice\n", s->id + 1);
    s->buffering = false;
    tcpDataOut(s);  // force flush of buffer
}

// write buffered data to file descriptor
void tcpDataOut(tcpPort_t *s)
{
    void *txData;
    int txLen, written;
    do {
        txLen = serialGetTxData(&s->port, &txData);
        // txLen == 0 signals to fdm that we are finished
        written = fdmFdWrite(&s->fdmFd, txData, txLen);
        if(written > 0)
            serialAckTxData(&s->port, written);
    } while(txLen && written == txLen);            // repeat until no data or write buffer is full
}

// is enabled after write pipe was full
void tcpWriteCb(struct fdmFd* f)
{
    tcpDataOut(container_of(f, tcpPort_t, fdmFd));
}

void tcpIntrCb(struct fdmFd* f)
{
    tcpDataOut(container_of(f, tcpPort_t, fdmFd));
}

void tcpDataIn(tcpPort_t *s)
{
    void *rxBuff;
    int available = serialGetRxDataBuffer(&s->port, &rxBuff);

    if (available <= 0) {
        // no space to store data, discard them
        fprintf(stderr, "UART%u: no buffer space\n", s->id + 1);
        return;
    }
    int count = fdmFdRead(&s->fdmFd, rxBuff, available);
    // errors are handled in fdmFdRead
    if(count > 0) {
        serialAckRxData(&s->port, count);
    }
}

void tcpReadCb(struct fdmFd* f)
{
    tcpDataIn(container_of(f, tcpPort_t, fdmFd));
}

static const struct serialPortVTable tcpVTable = {
    SERIALIMPL_VTABLE,
    .serialSetBaudRate = NULL,
    .setMode = NULL,
    .beginWrite = tcpBeginWrite,
    .endWrite = tcpEndWrite,
    .kickTx = tcpKickTx,
};
