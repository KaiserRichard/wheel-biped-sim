/*
 * Minimal Bare-Metal Newlib Syscall Stubs for arm-none-eabi-gcc
 */

#include <stddef.h>
#include <stdint.h>

void __attribute__((weak)) _init(void)
{
}

void __attribute__((weak)) _fini(void)
{
}

void *_sbrk(ptrdiff_t incr)
{
    (void)incr;
    return (void *)-1;
}

int _close(int file)
{
    (void)file;
    return -1;
}

int _fstat(int file, void *st)
{
    (void)file;
    (void)st;
    return 0;
}

int _isatty(int file)
{
    (void)file;
    return 1;
}

int _lseek(int file, int ptr, int dir)
{
    (void)file;
    (void)ptr;
    (void)dir;
    return 0;
}

int _read(int file, char *ptr, int len)
{
    (void)file;
    (void)ptr;
    (void)len;
    return 0;
}

int _write(int file, const char *ptr, int len)
{
    (void)file;
    (void)ptr;
    return len;
}

void _exit(int status)
{
    (void)status;
    while (1) {
    }
}

int _kill(int pid, int sig)
{
    (void)pid;
    (void)sig;
    return -1;
}

int _getpid(void)
{
    return 1;
}
