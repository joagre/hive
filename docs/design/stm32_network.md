# STM32 Network I/O Design

## Overview

Add networking to STM32 using lwIP (Lightweight IP stack). This document covers both UDP and TCP options, with a recommendation for which to implement first.

## Protocol Choice: UDP vs TCP

### Recommendation: Start with UDP

For embedded telemetry applications, **UDP is the pragmatic choice**:

| Aspect | UDP | TCP |
|--------|-----|-----|
| API complexity | 7 HAL functions | 10 HAL functions |
| Connection state | None | Complex state machine |
| Event types | 1 (receive) | 5 (accept, connect, read, write, error) |
| Memory per endpoint | ~100 bytes | ~2KB per connection |
| Total RAM | ~5KB | ~50KB (8 connections) |
| lwIP API | Raw API (simple) | Socket API (required) |
| Fit for telemetry | Excellent | Overkill |

### Use Case Analysis

**Telemetry (UDP ideal):**
- Send attitude, position, battery data
- Latest value wins, lost packets acceptable
- Fire-and-forget semantics

**Commands (UDP acceptable):**
- Setpoint updates, arm/disarm
- Idempotent, can be resent
- Order not critical for most commands

**When TCP is needed:**
- File transfer, firmware updates
- Log download requiring reliability
- Protocol requiring ordered delivery

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│  Application (actors)                                           │
│  hive_udp_send(), hive_udp_recv() / hive_tcp_*() for TCP       │
├─────────────────────────────────────────────────────────────────┤
│  hive_udp.c / hive_tcp.c (platform-independent)                │
│  Async operation management, timeouts, actor wakeup             │
├─────────────────────────────────────────────────────────────────┤
│  HAL: hive_hal_udp_stm32.c / hive_hal_tcp_stm32.c              │
│  lwIP wrapper (Raw API for UDP, Socket API for TCP)            │
├─────────────────────────────────────────────────────────────────┤
│  lwIP (IP stack)                                                │
│  Raw API (udp.h) or Socket API (sockets.h)                     │
├─────────────────────────────────────────────────────────────────┤
│  ethernetif.c (board-specific)                                  │
│  Links lwIP to STM32 ETH peripheral or W5500                   │
├─────────────────────────────────────────────────────────────────┤
│  Hardware: STM32 ETH MAC or W5500 (SPI)                        │
└─────────────────────────────────────────────────────────────────┘
```

---

# Part 1: UDP Implementation

## UDP HAL API

### File: `include/hal/hive_hal_udp.h`

```c
#ifndef HIVE_HAL_UDP_H
#define HIVE_HAL_UDP_H

#include "hive_types.h"

// Network address for UDP
typedef struct {
    uint32_t ip;    // IPv4 in network byte order
    uint16_t port;  // Port in host byte order
} hive_udp_addr_t;

// Initialize UDP subsystem
hive_status_t hive_hal_udp_init(void);

// Cleanup UDP subsystem
void hive_hal_udp_cleanup(void);

// Create a UDP socket
// Returns socket handle in *handle_out
hive_status_t hive_hal_udp_socket(int *handle_out);

// Bind socket to local port
hive_status_t hive_hal_udp_bind(int handle, uint16_t port);

// Close socket
hive_status_t hive_hal_udp_close(int handle);

// Send datagram (non-blocking)
// Returns HIVE_SUCCESS or HIVE_ERR_WOULDBLOCK
hive_status_t hive_hal_udp_sendto(int handle, const void *data, size_t len,
                                   const hive_udp_addr_t *dest);

// Receive datagram (non-blocking)
// Returns HIVE_SUCCESS with data and sender address
// Returns HIVE_ERR_WOULDBLOCK if no data available
hive_status_t hive_hal_udp_recvfrom(int handle, void *buf, size_t buflen,
                                     size_t *received, hive_udp_addr_t *from);

#endif // HIVE_HAL_UDP_H
```

## UDP HAL Implementation (lwIP Raw API)

### File: `src/hal/stm32/hive_hal_udp_stm32.c`

```c
#include "hal/hive_hal_udp.h"
#include "lwip/udp.h"
#include "lwip/pbuf.h"
#include <string.h>

// Simple socket table (lwIP raw API uses PCB pointers)
#define MAX_UDP_SOCKETS 8

typedef struct {
    struct udp_pcb *pcb;
    struct pbuf *rx_queue;      // Received packets waiting
    hive_udp_addr_t rx_from;    // Sender of head packet
    volatile bool has_data;
} udp_socket_t;

static udp_socket_t s_sockets[MAX_UDP_SOCKETS];
static volatile bool s_udp_event_pending;

// Callback when packet received
static void udp_recv_callback(void *arg, struct udp_pcb *pcb,
                               struct pbuf *p, const ip_addr_t *addr, u16_t port) {
    udp_socket_t *sock = (udp_socket_t *)arg;

    if (sock->rx_queue != NULL) {
        // Already have a packet queued - drop new one (or chain if desired)
        pbuf_free(p);
        return;
    }

    sock->rx_queue = p;
    sock->rx_from.ip = addr->addr;
    sock->rx_from.port = port;
    sock->has_data = true;
    s_udp_event_pending = true;
}

hive_status_t hive_hal_udp_init(void) {
    memset(s_sockets, 0, sizeof(s_sockets));
    s_udp_event_pending = false;
    return HIVE_SUCCESS;
}

void hive_hal_udp_cleanup(void) {
    for (int i = 0; i < MAX_UDP_SOCKETS; i++) {
        if (s_sockets[i].pcb) {
            udp_remove(s_sockets[i].pcb);
            if (s_sockets[i].rx_queue) {
                pbuf_free(s_sockets[i].rx_queue);
            }
        }
    }
}

hive_status_t hive_hal_udp_socket(int *handle_out) {
    // Find free slot
    for (int i = 0; i < MAX_UDP_SOCKETS; i++) {
        if (s_sockets[i].pcb == NULL) {
            struct udp_pcb *pcb = udp_new();
            if (pcb == NULL) {
                return HIVE_ERROR(HIVE_ERR_NOMEM, "Failed to create UDP PCB");
            }

            s_sockets[i].pcb = pcb;
            s_sockets[i].rx_queue = NULL;
            s_sockets[i].has_data = false;

            udp_recv(pcb, udp_recv_callback, &s_sockets[i]);

            *handle_out = i;
            return HIVE_SUCCESS;
        }
    }
    return HIVE_ERROR(HIVE_ERR_NOMEM, "No free UDP sockets");
}

hive_status_t hive_hal_udp_bind(int handle, uint16_t port) {
    if (handle < 0 || handle >= MAX_UDP_SOCKETS || !s_sockets[handle].pcb) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "Invalid socket handle");
    }

    err_t err = udp_bind(s_sockets[handle].pcb, IP_ADDR_ANY, port);
    if (err != ERR_OK) {
        return HIVE_ERROR(HIVE_ERR_IO, "Bind failed");
    }
    return HIVE_SUCCESS;
}

hive_status_t hive_hal_udp_close(int handle) {
    if (handle < 0 || handle >= MAX_UDP_SOCKETS || !s_sockets[handle].pcb) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "Invalid socket handle");
    }

    udp_remove(s_sockets[handle].pcb);
    if (s_sockets[handle].rx_queue) {
        pbuf_free(s_sockets[handle].rx_queue);
    }
    memset(&s_sockets[handle], 0, sizeof(udp_socket_t));
    return HIVE_SUCCESS;
}

hive_status_t hive_hal_udp_sendto(int handle, const void *data, size_t len,
                                   const hive_udp_addr_t *dest) {
    if (handle < 0 || handle >= MAX_UDP_SOCKETS || !s_sockets[handle].pcb) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "Invalid socket handle");
    }

    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, len, PBUF_RAM);
    if (p == NULL) {
        return HIVE_ERROR(HIVE_ERR_NOMEM, "Failed to allocate pbuf");
    }

    memcpy(p->payload, data, len);

    ip_addr_t addr;
    addr.addr = dest->ip;

    err_t err = udp_sendto(s_sockets[handle].pcb, p, &addr, dest->port);
    pbuf_free(p);

    if (err != ERR_OK) {
        return HIVE_ERROR(HIVE_ERR_IO, "Send failed");
    }
    return HIVE_SUCCESS;
}

hive_status_t hive_hal_udp_recvfrom(int handle, void *buf, size_t buflen,
                                     size_t *received, hive_udp_addr_t *from) {
    if (handle < 0 || handle >= MAX_UDP_SOCKETS || !s_sockets[handle].pcb) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "Invalid socket handle");
    }

    udp_socket_t *sock = &s_sockets[handle];

    if (!sock->has_data || sock->rx_queue == NULL) {
        return HIVE_ERROR(HIVE_ERR_WOULDBLOCK, "No data available");
    }

    // Copy data from pbuf
    size_t copy_len = sock->rx_queue->tot_len;
    if (copy_len > buflen) {
        copy_len = buflen;
    }
    pbuf_copy_partial(sock->rx_queue, buf, copy_len, 0);

    *received = copy_len;
    if (from) {
        *from = sock->rx_from;
    }

    // Free the pbuf
    pbuf_free(sock->rx_queue);
    sock->rx_queue = NULL;
    sock->has_data = false;

    return HIVE_SUCCESS;
}

// Called by scheduler to check for pending UDP events
bool hive_hal_udp_poll(void) {
    if (s_udp_event_pending) {
        s_udp_event_pending = false;
        return true;
    }
    return false;
}
```

## UDP Event Integration

UDP event integration is trivial compared to TCP:

```c
// In hive_hal_event.c - event wait function
void hive_hal_event_wait(int timeout_ms) {
    uint64_t deadline = hive_hal_get_time_us() + timeout_ms * 1000;

    while (hive_hal_get_time_us() < deadline) {
        // Process lwIP (handles incoming packets, calls callbacks)
        sys_check_timeouts();

        // Check if any UDP socket has data
        if (hive_hal_udp_poll()) {
            return;  // Wake scheduler
        }

        // Check timers
        if (timer_pending()) {
            return;
        }

        __WFI();  // Sleep until interrupt
    }
}
```

Only one event type to handle: "data available". No connection states, no write readiness tracking.

## UDP lwIP Configuration

### File: `lwipopts.h` (UDP-only, minimal)

```c
#ifndef LWIPOPTS_H
#define LWIPOPTS_H

// Minimal memory for UDP only
#define MEM_SIZE                    (4 * 1024)   // 4KB heap
#define MEMP_NUM_PBUF               8
#define MEMP_NUM_UDP_PCB            8            // Max UDP sockets
#define PBUF_POOL_SIZE              8
#define PBUF_POOL_BUFSIZE           512          // Smaller for telemetry

// UDP only - disable TCP
#define LWIP_UDP                    1
#define LWIP_TCP                    0

// No socket layer needed for UDP raw API
#define LWIP_SOCKET                 0
#define LWIP_NETCONN                0

// Threading (bare-metal)
#define NO_SYS                      1
#define SYS_LIGHTWEIGHT_PROT        0

// Disable unused features
#define LWIP_DHCP                   0
#define LWIP_DNS                    0
#define LWIP_IGMP                   0
#define LWIP_ICMP                   1            // Keep for ping

// Checksum by hardware
#define CHECKSUM_BY_HARDWARE        1
#define CHECKSUM_GEN_IP             0
#define CHECKSUM_GEN_UDP            0
#define CHECKSUM_CHECK_IP           0
#define CHECKSUM_CHECK_UDP          0

#define LWIP_DEBUG                  0

#endif // LWIPOPTS_H
```

## UDP Memory Requirements

| Component | RAM Usage |
|-----------|-----------|
| lwIP heap | 4 KB |
| Packet buffers (8 × 512) | 4 KB |
| UDP PCBs (8 sockets) | ~800 bytes |
| Socket table | ~200 bytes |
| **Total** | **~10 KB** |

Compare to TCP: ~50KB. UDP uses **5× less RAM**.

---

# Part 2: TCP Implementation

TCP is more complex but may be needed for reliable file transfer or firmware updates.

## TCP HAL API

The existing `hive_hal_tcp.h` defines the TCP API (10 functions):
- `init`, `cleanup`
- `socket`, `bind`, `listen`, `accept`, `connect`, `connect_check`, `close`
- `recv`, `send`

## lwIP API Choice for TCP

**Socket API (recommended for TCP):**
- Familiar BSD-like interface
- Maps directly to HAL functions
- Requires `LWIP_SOCKET=1`
- Adds ~5KB overhead

**Raw API (not recommended for TCP):**
- Complex connection state machine
- Difficult to integrate with Hive's blocking model
- Only worth it if RAM is extremely tight

## TCP HAL Implementation

### File: `src/hal/stm32/hive_hal_tcp_stm32.c`

```c
#include "hal/hive_hal_tcp.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include <string.h>

// Socket-to-fd mapping (lwIP sockets start at LWIP_SOCKET_OFFSET)
// We use lwIP socket numbers directly as our "fd"

hive_status_t hive_hal_tcp_init(void) {
    // lwIP init is done separately at system startup
    // (tcpip_init() called from main before hive_init)
    return HIVE_SUCCESS;
}

void hive_hal_tcp_cleanup(void) {
    // Nothing to do - lwIP lifecycle managed by system
}

hive_status_t hive_hal_tcp_socket(int *out) {
    int sock = lwip_socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        return HIVE_ERROR(HIVE_ERR_IO, "Failed to create socket");
    }

    // Set non-blocking
    int flags = lwip_fcntl(sock, F_GETFL, 0);
    lwip_fcntl(sock, F_SETFL, flags | O_NONBLOCK);

    // Set SO_REUSEADDR
    int opt = 1;
    lwip_setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    *out = sock;
    return HIVE_SUCCESS;
}

hive_status_t hive_hal_tcp_bind(int fd, uint16_t port) {
    struct sockaddr_in addr = {0};
    addr.sin_family = AF_INET;
    addr.sin_port = lwip_htons(port);
    addr.sin_addr.s_addr = INADDR_ANY;

    if (lwip_bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        return HIVE_ERROR(HIVE_ERR_IO, "Bind failed");
    }
    return HIVE_SUCCESS;
}

hive_status_t hive_hal_tcp_listen(int fd, int backlog) {
    if (lwip_listen(fd, backlog) < 0) {
        return HIVE_ERROR(HIVE_ERR_IO, "Listen failed");
    }
    return HIVE_SUCCESS;
}

hive_status_t hive_hal_tcp_accept(int listen_fd, int *out) {
    struct sockaddr_in addr;
    socklen_t len = sizeof(addr);

    int conn = lwip_accept(listen_fd, (struct sockaddr *)&addr, &len);
    if (conn < 0) {
        if (errno == EWOULDBLOCK || errno == EAGAIN) {
            return HIVE_ERROR(HIVE_ERR_WOULDBLOCK, "No pending connections");
        }
        return HIVE_ERROR(HIVE_ERR_IO, "Accept failed");
    }

    // Set new connection non-blocking
    int flags = lwip_fcntl(conn, F_GETFL, 0);
    lwip_fcntl(conn, F_SETFL, flags | O_NONBLOCK);

    *out = conn;
    return HIVE_SUCCESS;
}

hive_status_t hive_hal_tcp_connect(int fd, const char *ip, uint16_t port) {
    struct sockaddr_in addr = {0};
    addr.sin_family = AF_INET;
    addr.sin_port = lwip_htons(port);

    if (!inet_aton(ip, &addr.sin_addr)) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "Invalid IP address");
    }

    int ret = lwip_connect(fd, (struct sockaddr *)&addr, sizeof(addr));
    if (ret == 0) {
        return HIVE_SUCCESS;  // Connected immediately
    }

    if (errno == EINPROGRESS) {
        return HIVE_ERROR(HIVE_ERR_INPROGRESS, "Connection in progress");
    }

    return HIVE_ERROR(HIVE_ERR_IO, "Connect failed");
}

hive_status_t hive_hal_tcp_connect_check(int fd) {
    int error = 0;
    socklen_t len = sizeof(error);

    if (lwip_getsockopt(fd, SOL_SOCKET, SO_ERROR, &error, &len) < 0) {
        return HIVE_ERROR(HIVE_ERR_IO, "getsockopt failed");
    }

    if (error != 0) {
        return HIVE_ERROR(HIVE_ERR_IO, "Connection failed");
    }

    return HIVE_SUCCESS;
}

hive_status_t hive_hal_tcp_close(int fd) {
    lwip_close(fd);
    return HIVE_SUCCESS;
}

hive_status_t hive_hal_tcp_recv(int fd, void *buf, size_t len, size_t *bytes_read) {
    ssize_t ret = lwip_recv(fd, buf, len, 0);

    if (ret < 0) {
        if (errno == EWOULDBLOCK || errno == EAGAIN) {
            return HIVE_ERROR(HIVE_ERR_WOULDBLOCK, "No data available");
        }
        return HIVE_ERROR(HIVE_ERR_IO, "Recv failed");
    }

    *bytes_read = (size_t)ret;  // 0 = EOF (connection closed)
    return HIVE_SUCCESS;
}

hive_status_t hive_hal_tcp_send(int fd, const void *buf, size_t len, size_t *bytes_written) {
    ssize_t ret = lwip_send(fd, buf, len, 0);

    if (ret < 0) {
        if (errno == EWOULDBLOCK || errno == EAGAIN) {
            return HIVE_ERROR(HIVE_ERR_WOULDBLOCK, "Send buffer full");
        }
        return HIVE_ERROR(HIVE_ERR_IO, "Send failed");
    }

    *bytes_written = (size_t)ret;
    return HIVE_SUCCESS;
}
```

## TCP Event Integration (Complex)

TCP requires tracking multiple event types per socket:

### Option A: Polling (Simple but inefficient)

```c
void hive_hal_event_wait(int timeout_ms) {
    uint64_t deadline = hive_hal_get_time_us() + timeout_ms * 1000;

    while (hive_hal_get_time_us() < deadline) {
        sys_check_timeouts();

        if (check_socket_events()) {
            return;
        }

        __WFI();
    }
}
```

### Option B: lwIP Callbacks (Efficient but complex)

```c
typedef struct {
    int fd;
    volatile bool readable;
    volatile bool writable;
    volatile bool error;
} socket_state_t;

static socket_state_t s_sockets[MEMP_NUM_NETCONN];

static void recv_callback(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err) {
    socket_state_t *state = (socket_state_t *)arg;
    state->readable = true;
    g_net_event_pending = true;
}

void hive_hal_event_wait(int timeout_ms) {
    while (!g_net_event_pending && !timer_expired) {
        __WFI();
    }

    if (g_net_event_pending) {
        g_net_event_pending = false;
        dispatch_socket_events();
    }
}
```

### Option C: lwip_select() (Recommended for TCP)

```c
static fd_set s_read_fds, s_write_fds;
static int s_max_fd = -1;

hive_status_t hive_hal_event_register(int fd, uint32_t events, void *source) {
    if (events & HIVE_EVENT_READ) {
        FD_SET(fd, &s_read_fds);
    }
    if (events & HIVE_EVENT_WRITE) {
        FD_SET(fd, &s_write_fds);
    }
    if (fd > s_max_fd) s_max_fd = fd;
    s_sources[fd] = source;
    return HIVE_SUCCESS;
}

void hive_hal_event_wait(int timeout_ms) {
    struct timeval tv = {
        .tv_sec = timeout_ms / 1000,
        .tv_usec = (timeout_ms % 1000) * 1000
    };

    fd_set read_copy = s_read_fds;
    fd_set write_copy = s_write_fds;

    int ret = lwip_select(s_max_fd + 1, &read_copy, &write_copy, NULL, &tv);

    if (ret > 0) {
        for (int fd = 0; fd <= s_max_fd; fd++) {
            if (FD_ISSET(fd, &read_copy) || FD_ISSET(fd, &write_copy)) {
                io_source_t *source = s_sources[fd];
                if (source) {
                    hive_tcp_handle_event(source);
                }
            }
        }
    }
}
```

## TCP lwIP Configuration

### File: `lwipopts.h` (TCP enabled)

```c
#ifndef LWIPOPTS_H
#define LWIPOPTS_H

// Memory configuration
#define MEM_SIZE                    (16 * 1024)  // 16KB heap
#define MEMP_NUM_PBUF               16
#define MEMP_NUM_TCP_PCB            8            // Max TCP connections
#define MEMP_NUM_TCP_SEG            32
#define PBUF_POOL_SIZE              16
#define PBUF_POOL_BUFSIZE           1536         // MTU + headers

// TCP settings
#define LWIP_TCP                    1
#define TCP_MSS                     1460
#define TCP_WND                     (4 * TCP_MSS)
#define TCP_SND_BUF                 (4 * TCP_MSS)

// Socket API required for TCP
#define LWIP_SOCKET                 1
#define LWIP_COMPAT_SOCKETS         0
#define LWIP_POSIX_SOCKETS_IO_NAMES 0
#define LWIP_SO_RCVTIMEO            1
#define LWIP_SO_SNDTIMEO            1

// Threading (bare-metal)
#define NO_SYS                      1
#define SYS_LIGHTWEIGHT_PROT        0

// Optional: enable UDP too
#define LWIP_UDP                    1

// Disable unused features
#define LWIP_DHCP                   0
#define LWIP_DNS                    0
#define LWIP_IGMP                   0
#define LWIP_ICMP                   1

// Checksum by hardware
#define CHECKSUM_BY_HARDWARE        1
#define CHECKSUM_GEN_IP             0
#define CHECKSUM_GEN_TCP            0
#define CHECKSUM_CHECK_IP           0
#define CHECKSUM_CHECK_TCP          0

#define LWIP_DEBUG                  0

#endif // LWIPOPTS_H
```

## TCP Memory Requirements

| Component | RAM Usage |
|-----------|-----------|
| lwIP heap | 16-32 KB |
| Packet buffers | 8-16 KB |
| TCP PCBs (8 conn) | ~2 KB |
| Socket layer | ~5 KB |
| ETH DMA buffers | 4-8 KB |
| **Total** | **~40-60 KB** |

---

# Part 3: Common Infrastructure

## Ethernet Driver Integration

Both UDP and TCP need the same low-level driver.

### Required Files

```
lib/lwip/                        # lwIP source (git submodule or copy)
  src/core/
  src/netif/
  src/include/

src/hal/stm32/
  hive_hal_udp_stm32.c          # UDP HAL (raw API)
  hive_hal_tcp_stm32.c          # TCP HAL (socket API)
  ethernetif.c                   # Board-specific ETH<->lwIP glue
  ethernetif.h
```

### ETH Driver Outline (`ethernetif.c`)

```c
#include "lwip/netif.h"
#include "lwip/etharp.h"
#include "stm32f4xx_hal.h"

ETH_HandleTypeDef heth;
static struct netif g_netif;

void ethernetif_init(void) {
    // Configure ETH peripheral
    heth.Instance = ETH;
    heth.Init.MACAddr = (uint8_t[]){0x00, 0x80, 0xE1, 0x00, 0x00, 0x01};
    heth.Init.AutoNegotiation = ETH_AUTONEGOTIATION_ENABLE;
    heth.Init.Speed = ETH_SPEED_100M;
    heth.Init.DuplexMode = ETH_MODE_FULLDUPLEX;
    HAL_ETH_Init(&heth);

    // Add network interface to lwIP
    ip4_addr_t ip, mask, gw;
    IP4_ADDR(&ip, 192, 168, 1, 100);
    IP4_ADDR(&mask, 255, 255, 255, 0);
    IP4_ADDR(&gw, 192, 168, 1, 1);

    netif_add(&g_netif, &ip, &mask, &gw, NULL, ethernetif_init_cb, ethernet_input);
    netif_set_default(&g_netif);
    netif_set_up(&g_netif);
}

void ethernetif_input(void) {
    struct pbuf *p;
    while ((p = low_level_input(&g_netif)) != NULL) {
        if (g_netif.input(p, &g_netif) != ERR_OK) {
            pbuf_free(p);
        }
    }
}

void ethernetif_poll(void) {
    ethernetif_input();
    sys_check_timeouts();
}
```

### Main Loop Integration

```c
int main(void) {
    HAL_Init();
    SystemClock_Config();

    // Initialize Ethernet and lwIP BEFORE hive_init
    ethernetif_init();

    hive_init();

    // Spawn actors...

    while (1) {
        ethernetif_poll();
        hive_run_until_blocked();
    }
}
```

## Hardware Requirements

**Supported MCUs** (with built-in Ethernet MAC):
- STM32F407/F417/F427/F437/F429/F439
- STM32F746/F756/F767/F769/F777/F779
- STM32H743/H753/H745/H747/H750

**NOT supported** (no Ethernet MAC):
- STM32F103 (Blue Pill)
- STM32F401/F411 (Black Pill)
- STM32F405

For MCUs without Ethernet MAC, options include:
- External Ethernet module (W5500, ENC28J60) via SPI
- Different communication method (radio, CAN, etc.)

---

# Summary: Implementation Recommendation

## Start with UDP

1. **Add lwIP to build** - Minimal configuration, UDP only
2. **Implement ETH driver** - Same for both protocols
3. **Implement UDP HAL** - 7 functions, raw API
4. **Simple event integration** - Just poll for received packets
5. **Test with telemetry** - Send/receive attitude data

## Add TCP Later (if needed)

1. **Enable `LWIP_SOCKET=1`** - Adds socket layer
2. **Implement TCP HAL** - 10 functions
3. **Complex event integration** - Multiple socket states
4. **Use for file transfer** - Log download, firmware updates

## Complexity Comparison

| Aspect | UDP | TCP |
|--------|-----|-----|
| HAL functions | 7 | 10 |
| lwIP config | Minimal | Full socket layer |
| Event states | 1 | 5 |
| RAM usage | ~10 KB | ~50 KB |
| Implementation effort | Low | Medium-High |
