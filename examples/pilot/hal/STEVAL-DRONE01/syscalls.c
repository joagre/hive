// Newlib syscall stubs for bare-metal STM32
//
// These stubs satisfy the linker when using newlib (nano).
// _write sends output to USART1 for debug, others return error.

#include <sys/stat.h>
#include <errno.h>
#include "usart1.h"

int _close(int fd) {
    (void)fd;
    return -1;
}

int _lseek(int fd, int offset, int whence) {
    (void)fd;
    (void)offset;
    (void)whence;
    return -1;
}

int _read(int fd, char *buf, int len) {
    (void)fd;
    (void)buf;
    (void)len;
    return -1;
}

int _write(int fd, const char *buf, int len) {
    // Send stdout/stderr to USART1
    if (fd == 1 || fd == 2) {
        usart1_write(buf, len);
        return len;
    }
    return -1;
}

int _fstat(int fd, struct stat *st) {
    (void)fd;
    st->st_mode = S_IFCHR;
    return 0;
}

int _isatty(int fd) {
    (void)fd;
    return 1;
}

void *_sbrk(int incr) {
    extern char _end;
    static char *heap_end = 0;
    char *prev_heap_end;

    if (heap_end == 0) {
        heap_end = &_end;
    }
    prev_heap_end = heap_end;
    heap_end += incr;
    return prev_heap_end;
}
