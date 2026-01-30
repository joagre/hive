// Newlib syscall stubs for bare-metal STM32
//
// Minimal implementations for libc functions that need OS support.

#include <stdint.h>
#include <sys/stat.h>
#include <errno.h>
#include "debug_swo.h"

// System core clock - 168 MHz after PLL configuration
uint32_t SystemCoreClock = 168000000;

// Heap end (from linker script)
extern char _end;
static char *heap_ptr = &_end;

// sbrk - heap allocation for malloc
void *_sbrk(int incr) {
    char *prev_heap_ptr = heap_ptr;
    heap_ptr += incr;
    return prev_heap_ptr;
}

// Minimal I/O stubs (return error)
int _close(int file) {
    (void)file;
    return -1;
}
int _fstat(int file, struct stat *st) {
    (void)file;
    st->st_mode = S_IFCHR;
    return 0;
}
int _isatty(int file) {
    (void)file;
    return 1;
}
int _lseek(int file, int ptr, int dir) {
    (void)file;
    (void)ptr;
    (void)dir;
    return 0;
}
int _read(int file, char *ptr, int len) {
    (void)file;
    (void)ptr;
    (void)len;
    return 0;
}
int _write(int file, char *ptr, int len) {
    (void)file;
    // Route stdout/stderr to SWO debug output
    for (int i = 0; i < len; i++) {
        debug_swo_putc(ptr[i]);
    }
    return len;
}

// Process stubs
int _getpid(void) {
    return 1;
}
int _kill(int pid, int sig) {
    (void)pid;
    (void)sig;
    errno = EINVAL;
    return -1;
}
void _exit(int status) {
    (void)status;
    while (1)
        ;
}

// Empty SystemInit (clock configured in platform_init)
void SystemInit(void) {
    // Clock configuration done in platform_crazyflie.c
}
