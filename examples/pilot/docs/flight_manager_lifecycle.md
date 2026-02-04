# Flight Manager Lifecycle

Specification for multi-run test support without power cycling.

## Problem

Currently, the flight manager exits after completing a flight. To run another test:
1. Power cycle the drone
2. Wait for boot sequence
3. Send GO command

This is slow and inflexible. We want to send GO multiple times in a single power-on session.

## Solution

Flight manager becomes a looping state machine. After landing, it returns to IDLE and waits for the next GO. Before each flight, it requests RESET from all siblings so all actors start with clean state.

## State Machine

```
                          ┌─────────────────────────────────────┐
                          │                                     │
                          v                                     │
    ┌──────────────────────────────────────────────────────────────┐
    │                         IDLE                                 │
    │  - Motors disarmed                                           │
    │  - Waiting for GO command                                    │
    └──────────────────────────────────────────────────────────────┘
                          │
                          │ GO command received
                          v
    ┌──────────────────────────────────────────────────────────────┐
    │                       PREFLIGHT                              │
    │  1. Request RESET from all siblings, wait for replies        │
    │     - sensor_actor: call hal_calibrate(), reply ok/fail      │
    │     - estimator_actor: reset filters, reply ok/fail          │
    │     - logger_actor: truncate logs, reply ok/fail             │
    │     - others: reset state (PIDs, waypoints, etc.), reply ok  │
    │  2. If any RESET failed: log error, return to IDLE           │
    │  3. Request ARM from motor_actor, wait for reply             │
    │     - motor_actor: call hal_arm(), reply ok/fail             │
    │  4. If ARM failed: log error, return to IDLE                 │
    └──────────────────────────────────────────────────────────────┘
                          │
                          │ Preflight complete
                          v
    ┌──────────────────────────────────────────────────────────────┐
    │                        ARMED                                 │
    │  - 60 second countdown (configurable)                        │
    │  - Operator can send ABORT to return to IDLE                 │
    │  - Ground station can monitor countdown                      │
    └──────────────────────────────────────────────────────────────┘
                          │
                          │ Countdown complete
                          v
    ┌──────────────────────────────────────────────────────────────┐
    │                        FLYING                                │
    │  - Execute waypoint sequence                                 │
    │  - Normal flight control loop                                │
    │  - Wait for flight duration timer                            │
    └──────────────────────────────────────────────────────────────┘
                          │
                          │ Flight duration complete
                          v
    ┌──────────────────────────────────────────────────────────────┐
    │                       LANDING                                │
    │  1. Send LANDING notification to altitude_actor              │
    │  2. Wait for LANDED notification from altitude_actor         │
    └──────────────────────────────────────────────────────────────┘
                          │
                          │ Landed confirmed
                          v
    ┌──────────────────────────────────────────────────────────────┐
    │                       LANDED                                 │
    │  1. Request DISARM from motor_actor                          │
    │     - motor_actor: call hal_disarm(), reply ok               │
    │  2. Transition to IDLE                                       │
    └──────────────────────────────────────────────────────────────┘
                          │
                          └──────────────────────────────────────────┘
```

### ARMED State Details

The ARMED state provides a safety window between arming and flight:

- **Countdown** - 60 seconds by default (tunable via `armed_countdown_s` parameter)
- **ABORT command** - Operator can abort during countdown, returns to IDLE
- **Telemetry** - Ground station polls STATUS to display countdown remaining
- **No motor spin** - Motors armed but not spinning (ready for instant response)

```
ARMED ──timeout──> FLYING
  │
  └────ABORT────> IDLE (disarm motors)
```

If ABORT is received:
1. Request DISARM from motor_actor (motor_actor calls hal_disarm())
2. Log abort event
3. Return to IDLE (no RESET needed - state is already clean)

### Tunable Parameters

| Parameter | Default | Range | Description |
|-----------|---------|-------|-------------|
| `armed_countdown_s` | 60.0 | 5.0 - 300.0 | Seconds to wait in ARMED before flight |
| `auto_go_delay_s` | 0.0 | 0.0 - 60.0 | Auto-GO after delay (0 = wait for command) |

### Simulation Mode (Webots)

Webots has no radio, so GO command cannot be sent. Instead, flight_manager auto-starts:

- If `SIMULATED_TIME` is defined and `auto_go_delay_s > 0`: auto-transition IDLE -> PREFLIGHT after delay
- Default: `auto_go_delay_s = 2.0` in Webots, `0.0` on hardware

This enables automated testing without external tooling. The rest of the lifecycle (RESET, ARMED countdown, flight, landing) runs normally.

```c
// In flight_manager IDLE state:
#ifdef SIMULATED_TIME
if (params->auto_go_delay_s > 0) {
    hive_sleep_ms((uint32_t)(params->auto_go_delay_s * 1000));
    // Auto-transition to PREFLIGHT
}
#endif
```

## Actor Responsibilities

Clear separation of concerns:

| Actor | Owns | Responsibility |
|-------|------|----------------|
| **sensor_actor** | Hardware sensors | Read sensors, calibrate on RESET (hal_calibrate) |
| **estimator_actor** | State estimation | Fuse sensors into state estimate |
| **altitude_actor** | Vertical control | Altitude PID, landing sequence, sends LANDED |
| **position_actor** | Horizontal control | Position PID |
| **attitude_actor** | Angle control | Attitude PIDs |
| **rate_actor** | Angular rate control | Rate PIDs |
| **motor_actor** | Motor output | Gate outputs, arm/disarm on request (hal_arm/hal_disarm) |
| **waypoint_actor** | Mission | Track waypoint sequence |
| **flight_manager** | Flight phases | Orchestrate state machine via requests |
| **logger_actor** | Logs | Hive log + telemetry CSV lifecycle |
| **comms_actor** | Radio | Relay commands, read state_bus for telemetry, handle params |

### RESET Request/Reply

Each actor handles RESET and replies with success or failure:

| Actor | Action on RESET | Reply |
|-------|-----------------|-------|
| sensor_actor | Call `hal_calibrate()` | ok or error code |
| estimator_actor | Reset filters, establish initial state | ok or error code |
| PID actors | Reset integrators | ok (always succeeds) |
| logger_actor | Truncate logs | ok or error code |
| motor_actor | Clear started flag, zero outputs | ok (always succeeds) |
| waypoint_actor | Reset index to 0 | ok (always succeeds) |
| comms_actor | No-op (stateless) | ok (always succeeds) |

If any actor replies with an error, flight_manager aborts preflight and returns to IDLE.

## Message Flow

```
Ground Station
      │
      v
 comms_actor ─────────────────────────────────────┐
      │                                           │
      ├──GO/ABORT──> flight_manager               │ reads state_bus
      │                   │                       │ for telemetry
      │                   ├──RESET──> [all siblings] ──reply──┐
      │                   │<────────────────────────────────────┘
      │                   │
      │                   ├──ARM/DISARM──> motor_actor ──reply──┐
      │                   │<────────────────────────────────────┘
      │                   │
      │                   ├──LANDING──> altitude_actor
      │                   │<──LANDED───
      │                   │
      │              [state_bus] <─────────────────┘
      │
      └──PARAM_SET/GET──> (comms_actor handles directly via tunable_params struct)
```

**Actor ID resolution**: Each actor receives a sibling list at spawn containing all sibling actor IDs. Use `hive_find_sibling(siblings, count, "name")` to find specific actors.

**Telemetry**: comms_actor subscribes to state_bus and reads state estimates directly. It does not receive telemetry from flight_manager.

### Message Types

| Message | From | To | Pattern |
|---------|------|-----|---------|
| GO | comms_actor | flight_manager | notify |
| ABORT | comms_actor | flight_manager | notify |
| RESET | flight_manager | all siblings | request/reply |
| ARM | flight_manager | motor_actor | request/reply |
| DISARM | flight_manager | motor_actor | request/reply |
| LANDING | flight_manager | altitude_actor | notify |
| LANDED | altitude_actor | flight_manager | notify |
| STATUS | comms_actor | flight_manager | request/reply |

**Notify messages** (fire-and-forget): GO, ABORT, LANDING, LANDED
**Request/reply messages** (explicit error handling): RESET, ARM, DISARM, STATUS

## RESET Request

Flight manager sends `RESET` request to all siblings during PREFLIGHT. Each actor prepares itself for a new flight and replies with success or failure.

### Message Format

```c
#define MSG_TAG_RESET 0x5245  // "RE" in ASCII

// Reply payload (1 byte)
#define RESET_OK    0x00
#define RESET_FAIL  0x01

// Flight manager requests RESET from all siblings:
uint8_t reply;
for (size_t i = 0; i < sibling_count; i++) {
    hive_status_t status = hive_ipc_request(
        siblings[i].id, NULL, 0, &reply, sizeof(reply), RESET_TIMEOUT_MS);
    if (HIVE_FAILED(status) || reply != RESET_OK) {
        HIVE_LOG_ERROR("[FLM] RESET failed for %s", siblings[i].name);
        return false;  // Abort preflight
    }
}
```

### Actor Responses

Each actor handles RESET according to its own needs and replies with status.

| Actor | Action on RESET | Reply |
|-------|-----------------|-------|
| sensor_actor | Call `hal_calibrate()` (gyro bias, baro zero) | RESET_OK or RESET_FAIL |
| estimator_actor | Reset Kalman filter and complementary filter | RESET_OK or RESET_FAIL |
| altitude_actor | Reset PID integrator, clear landing state | RESET_OK |
| position_actor | Reset PID integrator (if any) | RESET_OK |
| attitude_actor | Reset PID integrators | RESET_OK |
| rate_actor | Reset PID integrators | RESET_OK |
| motor_actor | Clear "started" flag, zero motor outputs | RESET_OK |
| waypoint_actor | Reset waypoint index to 0, clear arrival state | RESET_OK |
| logger_actor | Truncate hive log and telemetry CSV | RESET_OK or RESET_FAIL |
| comms_actor | No-op (stateless relay) | RESET_OK |

### Implementation Pattern

```c
// In each actor's main loop, handle RESET request:
hive_message_t msg;
if (HIVE_SUCCEEDED(hive_ipc_recv_match(HIVE_SENDER_ANY, HIVE_MSG_REQUEST,
                                        MSG_TAG_RESET, &msg, 0))) {
    // Reset actor-specific state
    pid_reset(&my_pid);
    // ... other resets

    // Reply with status
    uint8_t reply = RESET_OK;
    hive_ipc_reply(&msg, &reply, sizeof(reply));
}
```

Actors that use `hive_select()` for multiple event sources add RESET as a filter option.

## ARM/DISARM Requests

Motor arming is owned by motor_actor for clean separation of concerns.

### Message Format

```c
#define MSG_TAG_ARM    0x4152  // "AR" in ASCII
#define MSG_TAG_DISARM 0x4441  // "DA" in ASCII

// Reply payload (1 byte)
#define ARM_OK    0x00
#define ARM_FAIL  0x01
```

### motor_actor Implementation

```c
// Handle ARM request
if (HIVE_SUCCEEDED(hive_ipc_recv_match(HIVE_SENDER_ANY, HIVE_MSG_REQUEST,
                                        MSG_TAG_ARM, &msg, 0))) {
    hal_arm();  // Enable motor output
    uint8_t reply = ARM_OK;
    hive_ipc_reply(&msg, &reply, sizeof(reply));
}

// Handle DISARM request
if (HIVE_SUCCEEDED(hive_ipc_recv_match(HIVE_SENDER_ANY, HIVE_MSG_REQUEST,
                                        MSG_TAG_DISARM, &msg, 0))) {
    hal_disarm();  // Disable motor output
    uint8_t reply = ARM_OK;
    hive_ipc_reply(&msg, &reply, sizeof(reply));
}
```

## LANDING/LANDED Notifications

Simple fire-and-forget notifications for landing sequence coordination.

### Message Format

```c
#define MSG_TAG_LANDING 0x4C44  // "LD" in ASCII
#define MSG_TAG_LANDED  0x444E  // "DN" in ASCII

// flight_manager initiates landing:
hive_ipc_notify(altitude_actor_id, MSG_TAG_LANDING, NULL, 0);

// altitude_actor confirms landing complete (or emergency cutoff):
hive_ipc_notify(flight_manager_id, MSG_TAG_LANDED, NULL, 0);
```

### altitude_actor Implementation

```c
// Check for LANDING notification
if (HIVE_SUCCEEDED(hive_ipc_recv_match(flight_manager_id, HIVE_MSG_NOTIFY,
                                        MSG_TAG_LANDING, &msg, 0))) {
    landing_requested = true;
}

// In control loop: execute landing or emergency cutoff
if (landing_requested || emergency_cutoff_triggered) {
    // ... landing/cutoff logic ...
    if (landed_confirmed) {
        hive_ipc_notify(flight_manager_id, MSG_TAG_LANDED, NULL, 0);
    }
}
```

## Log File Handling

Both log files are owned by logger_actor (renamed from telemetry_logger_actor). This single-owner design keeps file lifecycle management in one place. Flight manager orchestrates flights, logger_actor manages logs.

### logger_actor Lifecycle

**Initialization (once at boot):**
```c
void *logger_actor_init(void *args) {
    // Open hive runtime log
    hive_log_file_open(HIVE_LOG_FILE_PATH);

    // Flush early boot messages captured before log was open
    hal_flush_early_log();

    // Open telemetry CSV
    tlog_fd = hive_file_open("/sd/tlog.csv", HIVE_O_WRONLY | HIVE_O_CREAT | HIVE_O_TRUNC);
    write_csv_header(tlog_fd);

    // ...
}
```

**On RESET (before each flight):**
```c
// Truncate hive runtime log
hive_log_file_close();
hive_log_file_open(HIVE_LOG_FILE_PATH);  // O_TRUNC erases flash sector on STM32
HIVE_LOG_INFO("=== New flight run ===");

// Truncate telemetry CSV
hive_file_close(tlog_fd);
tlog_fd = hive_file_open("/sd/tlog.csv", HIVE_O_WRONLY | HIVE_O_CREAT | HIVE_O_TRUNC);
write_csv_header(tlog_fd);
```

**Note:** `hal_flush_early_log()` is only called once at boot. The early log buffer captures HAL init messages before the hive log file is open. After the first flush, the buffer is disabled.

## Startup Sequence

On power-on, the system boots directly to IDLE state:

1. HAL init runs (basic hardware setup, messages go to early log buffer)
2. Supervisor spawns all actors
3. logger_actor init: opens hive log, flushes early buffer, opens telemetry CSV
4. Flight manager enters IDLE
5. Wait for GO command (hardware) or auto-GO after delay (simulation)

Calibration and self-test now happen per-flight in PREFLIGHT, not at boot. This means:
- Faster boot to IDLE
- Fresh calibration for each flight
- Gyro bias measured closer to actual flight time

## Error Handling

### Preflight Failure

Actors report errors explicitly via RESET reply:

- **sensor_actor** - `hal_calibrate()` failed (sensor hardware issue)
- **estimator_actor** - Filter initialization failed (invalid initial state)
- **logger_actor** - Log file operations failed (storage issue)
- **motor_actor** - `hal_arm()` failed (motor hardware issue)

If any RESET or ARM request fails:
1. flight_manager logs which actor failed and why
2. flight_manager returns to IDLE without starting flight
3. comms_actor can report failure to ground station via telemetry
4. Operator investigates and retries with another GO command

### Flight Abort

If emergency cutoff triggers during FLYING (tilt > 45°, altitude > max):
1. altitude_actor commands zero thrust (motors stop via control loop)
2. altitude_actor sends LANDED notification to flight_manager
3. flight_manager transitions to LANDED state
4. Normal LANDED cleanup (DISARM request to motor_actor), return to IDLE
5. System ready for another GO (after operator investigates)

The same LANDED notification is used for both normal landing completion and emergency cutoff. flight_manager doesn't need to distinguish - in both cases it proceeds to LANDED state cleanup.

### Actor Crash

If any supervised actor crashes:
- ONE_FOR_ALL restarts all children
- Flight manager restarts in IDLE state
- System ready for GO command

## Ground Station Interface

| Command | ID | Valid States | Action |
|---------|-----|--------------|--------|
| GO | 0x20 | IDLE | Transition IDLE -> PREFLIGHT |
| ABORT | 0x21 | ARMED | Disarm and return to IDLE |

### New Command: ABORT

Allows operator to cancel flight during the armed countdown window.

```c
#define CMD_ABORT 0x21

// Ground station sends:
// [CMD_ABORT] (no payload)
```

If received in ARMED state: disarm motors, return to IDLE.
If received in other states: ignored (log warning).

### STATUS Request

Query current state for ground station display. Ground station polls during ARMED to show countdown:

```c
#define CMD_GET_STATUS 0x22
#define RESP_STATUS    0x23

// Response payload:
typedef struct {
    uint8_t state;          // 0=IDLE, 1=PREFLIGHT, 2=ARMED, 3=FLYING, 4=LANDING, 5=LANDED
    uint8_t countdown_s;    // Seconds remaining (only valid in ARMED state)
} __attribute__((packed)) status_resp_t;
```

## Migration

### Changes Required

1. **flight_manager_actor.c** - Rewrite as state machine loop, use request/reply for RESET and ARM/DISARM, add auto-GO for simulation
2. **sensor_actor.c** - Handle RESET request (call hal_calibrate, reply ok/fail)
3. **motor_actor.c** - Handle RESET request (clear started flag, reply ok), handle ARM/DISARM requests (call hal_arm/hal_disarm, reply ok/fail)
4. **estimator_actor.c** - Handle RESET request (reset filters, reply ok/fail)
5. **altitude_actor.c** - Handle RESET request (reset PID, landing state, reply ok), handle LANDING notification (start descent), send LANDED notification (landing complete or emergency cutoff)
6. **attitude_actor.c** - Handle RESET request (reset PIDs, reply ok)
7. **rate_actor.c** - Handle RESET request (reset PIDs, reply ok)
8. **waypoint_actor.c** - Handle RESET request (reset index, reply ok)
9. **telemetry_logger_actor.c** - Rename to logger_actor, handle RESET request (truncate logs, reply ok/fail), move hive log open and early flush from pilot.c to init
10. **pilot.c** - Remove hive log open/flush and hal_calibrate/hal_arm (now in actors)
11. **pid.c** - Add `pid_reset()` function if not present
12. **tunable_params.h/c** - Add `armed_countdown_s` and `auto_go_delay_s` parameters

### Actors With Minimal Changes

- **position_actor.c** - Handle RESET request (reset PID if any, reply ok)
- **comms_actor.c** - Handle RESET request (no-op, reply ok)

## Testing

1. **Single flight** - Verify normal IDLE -> PREFLIGHT -> ARMED -> FLYING -> LANDING -> LANDED -> IDLE
2. **Multiple flights** - Send GO, complete flight, send GO again, verify clean state
3. **RESET failure** - Make hal_calibrate() fail, verify RESET reply is FAIL, return to IDLE
4. **ARM failure** - Make hal_arm() fail, verify ARM reply is FAIL, return to IDLE
5. **Armed countdown** - Verify 60s countdown in ARMED state, then transition to FLYING
6. **Abort during armed** - Send ABORT during countdown, verify DISARM request sent, return to IDLE
7. **Abort in wrong state** - Send ABORT in IDLE or FLYING, verify ignored with warning
8. **Emergency cutoff** - Trigger emergency cutoff mid-flight, verify return to IDLE
9. **Log truncation** - Verify logs are fresh after each GO (no data from previous run)
10. **Parameter tuning** - Change params in IDLE, verify new values used in next flight
11. **Request timeout** - Simulate slow actor, verify flight_manager handles timeout gracefully
