/**
 * ARM Semihosting interface for QEMU testing
 */

#ifndef SEMIHOSTING_H
#define SEMIHOSTING_H

/* Write a null-terminated string to debug console */
void semihosting_puts(const char *str);

/* Write a single character to debug console */
void semihosting_putc(char c);

/* Exit the application with status code */
void semihosting_exit(int status) __attribute__((noreturn));

/* Minimal printf implementation (returns number of chars written for printf compat) */
int semihosting_printf(const char *fmt, ...);

#endif /* SEMIHOSTING_H */
