#ifndef HIVE_IO_SOURCE_H
#define HIVE_IO_SOURCE_H

#include "hive_types.h"
#include <stdint.h>
#include <stdbool.h>

// I/O source types for epoll event loop
typedef enum {
    IO_SOURCE_TIMER,
    IO_SOURCE_NETWORK,
    IO_SOURCE_WAKEUP
} io_source_type_t;

// Forward declarations
typedef struct timer_entry_t timer_entry_t;

// Network I/O request data
typedef struct {
    int fd;
    void *buf;
    size_t len;
    actor_id_t actor;
    int operation; // NET_OP_RECV, NET_OP_SEND, etc.
} net_io_data_t;

// I/O source - tagged union for epoll events
typedef struct io_source_t {
    io_source_type_t type;
    union {
        timer_entry_t *timer; // Timer event
        net_io_data_t net;    // Network I/O event
        int wakeup;           // Wakeup signal
    } data;
} io_source_t;

// Pool for io_source_t allocation
#define HIVE_IO_SOURCE_POOL_SIZE 128

#endif // HIVE_IO_SOURCE_H
