#include "drivers/bus_spi.h"

void spiPinConfigure(const struct spiPinConfig_s *pConfig)
{
    UNUSED(pConfig);
}

void debugInit(void)
{
    // NOOP
}

int _open(const char *fn, int oflag, ...)
{
    UNUSED(fn);
    UNUSED(oflag);
    return -1;
}

int _close(int fd)
{
    UNUSED(fd);
    return -1;
}

int _lseek(int fd, int pos, int whence)
{
    UNUSED(fd);
    UNUSED(pos);
    UNUSED(whence);
    return -1;
}

int _fstat(int fd, char *buf)
{
    UNUSED(fd);
    UNUSED(buf);
    return -1;
}

int _isatty(int fd)
{
    UNUSED(fd);
    return -1;
}

int _getpid(void)
{
    return 0;
}

int _kill(int pid, int sig)
{
    UNUSED(pid);
    UNUSED(sig);
    return -1;
}

int _read(int handle, char *buffer, int length)
{
    UNUSED(handle);
    UNUSED(buffer);
    UNUSED(length);
    return -1;
}

int _write(int handle, char *buffer, int length)
{
    UNUSED(handle);
    UNUSED(buffer);
    UNUSED(length);
    return -1;
}
