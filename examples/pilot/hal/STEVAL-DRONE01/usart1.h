// USART1 debug serial driver for STM32F401 (STEVAL-DRONE01)
//
// Simple polling-based UART for debug output.
// TX: PA9, RX: PA10

#ifndef USART1_H
#define USART1_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// ----------------------------------------------------------------------------
// Configuration
// ----------------------------------------------------------------------------

// Common baud rates
typedef enum {
    USART1_BAUD_9600   = 9600,
    USART1_BAUD_19200  = 19200,
    USART1_BAUD_38400  = 38400,
    USART1_BAUD_57600  = 57600,
    USART1_BAUD_115200 = 115200,
    USART1_BAUD_230400 = 230400,
    USART1_BAUD_460800 = 460800,
    USART1_BAUD_921600 = 921600
} usart1_baud_t;

// Default baud rate
#define USART1_DEFAULT_BAUD     USART1_BAUD_115200

// Configuration structure
typedef struct {
    uint32_t baud_rate;     // Baud rate (e.g., 115200)
    bool tx_enable;         // Enable transmitter
    bool rx_enable;         // Enable receiver
} usart1_config_t;

// Default configuration: 115200 baud, TX only
#define USART1_CONFIG_DEFAULT { \
    .baud_rate = 115200, \
    .tx_enable = true, \
    .rx_enable = false \
}

// ----------------------------------------------------------------------------
// API
// ----------------------------------------------------------------------------

// Initialize USART1
// config: Configuration options (NULL for defaults)
void usart1_init(const usart1_config_t *config);

// Deinitialize USART1
void usart1_deinit(void);

// Set baud rate
void usart1_set_baud(uint32_t baud_rate);

// ----------------------------------------------------------------------------
// Transmit functions
// ----------------------------------------------------------------------------

// Send single character (blocking)
void usart1_putc(char c);

// Send string (blocking)
void usart1_puts(const char *str);

// Send data buffer (blocking)
void usart1_write(const void *data, size_t len);

// Send formatted string (printf-style, blocking)
// Returns number of characters written
// Note: Uses a fixed 256-byte buffer internally
int usart1_printf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

// Check if transmit buffer is empty (ready for next byte)
bool usart1_tx_ready(void);

// Wait for all data to be transmitted
void usart1_flush(void);

// ----------------------------------------------------------------------------
// Receive functions
// ----------------------------------------------------------------------------

// Receive single character (blocking)
char usart1_getc(void);

// Receive single character with timeout
// Returns true if character received, false on timeout
bool usart1_getc_timeout(char *c, uint32_t timeout_ms);

// Check if receive data is available
bool usart1_rx_ready(void);

// Read available data into buffer (non-blocking)
// Returns number of bytes read
size_t usart1_read(void *buf, size_t max_len);

// ----------------------------------------------------------------------------
// Debug helpers
// ----------------------------------------------------------------------------

// Print hex dump of memory region
void usart1_hexdump(const void *data, size_t len);

// Print integer in various formats
void usart1_print_int(int32_t value);
void usart1_print_uint(uint32_t value);
void usart1_print_hex(uint32_t value);
void usart1_print_float(float value, int decimals);

// Print newline
void usart1_newline(void);

#endif // USART1_H
