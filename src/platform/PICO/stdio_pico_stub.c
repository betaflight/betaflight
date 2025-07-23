#if 0
TODO remove this in favour of rp2_common/pico_clib_interface/newlib_interface.c
/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include "common/utils.h"

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

#endif
