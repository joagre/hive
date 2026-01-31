// Newlib syscall stubs for bare-metal STM32
//
// Minimal implementations for libc functions that need OS support.
// SystemCoreClock is defined in system_stm32f4xx.c

#include <sys/stat.h>
#include <errno.h>

// Heap boundaries (from linker script)
// Note: Stack is in CCM (separate memory), heap is in RAM - they can't collide.
// We check against _heap_end which is the linker-defined heap limit.
extern char _end;      // End of .bss, start of heap
extern char _heap_end; // End of heap region (from linker script)
static char *heap_ptr = &_end;

// sbrk - heap allocation for malloc
// Returns (void*)-1 on failure (heap limit exceeded)
void *_sbrk(int incr) {
    char *prev_heap_ptr = heap_ptr;
    char *new_heap_ptr = heap_ptr + incr;

    // Check for heap overflow
    if (new_heap_ptr > &_heap_end) {
        errno = ENOMEM;
        return (void *)-1;
    }

    heap_ptr = new_heap_ptr;
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
    (void)ptr;
    (void)len;
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
void __attribute__((noreturn)) _exit(int status) {
    (void)status;
    while (1) {
        __asm__ volatile("wfi");
    }
}

// ST Standard Peripheral Library assert handler
// Called when USE_FULL_ASSERT is defined and assert_param() fails
#include <stdint.h>
void assert_failed(uint8_t *file, uint32_t line) {
    (void)file;
    (void)line;
    // Halt on assertion failure
    while (1)
        ;
}
