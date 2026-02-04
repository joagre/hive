# Flight Manager Lifecycle

Specification for multi-run test support without power cycling.

## Problem

Currently, the flight manager exits after completing a flight. To run another test:
1. Power cycle the drone
2. Wait for boot sequence
3. Send GO command

This is slow and inflexible. We want to send GO multiple times in a single power-on session.

## Solution

Flight manager becomes a looping state machine. After landing, it returns to IDLE and waits for the next GO. Before each flight, it broadcasts RESET to siblings so all actors start with clean state.

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
    │  1. Call hal_calibrate() (gyro bias, baro zero, etc.)        │
    │  2. Broadcast RESET to all siblings                          │
    │     - estimator_actor: reset filters, set initial state      │
    │     - logger_actor: truncate logs                            │
    │     - others: reset state (PIDs, waypoints, etc.)            │
    │  3. Run self-test (validate state data is sane)              │
    │  4. Call hal_arm() to enable motor output                    │
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
    └──────────────────────────────────────────────────────────────┘
                          │
                          │ Final waypoint reached (land command)
                          v
    ┌──────────────────────────────────────────────────────────────┐
    │                       LANDING                                │
    │  - Execute landing sequence                                  │
    │  - Wait for landed confirmation                              │
    └──────────────────────────────────────────────────────────────┘
                          │
                          │ Landed confirmed
                          v
    ┌──────────────────────────────────────────────────────────────┐
    │                       LANDED                                 │
    │  1. Call hal_disarm() to disable motor output                │
    │  2. Transition to IDLE                                       │
    └──────────────────────────────────────────────────────────────┘
                          │
                          └──────────────────────────────────────────┘
```

### ARMED State Details

The ARMED state provides a safety window between arming and flight:

- **Countdown** - 60 seconds by default (tunable via `armed_countdown_s` parameter)
- **ABORT command** - Operator can abort during countdown, returns to IDLE
- **Telemetry** - Countdown remaining sent to ground station for display
- **No motor spin** - Motors armed but not spinning (ready for instant response)

```
ARMED ──timeout──> FLYING
  │
  └────ABORT────> IDLE (disarm motors)
```

If ABORT is received:
1. Call hal_disarm() to disable motor output
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
| **sensor_actor** | Hardware sensors | Read sensors (calibration done by flight_manager) |
| **estimator_actor** | State estimation | Fuse sensors into state estimate |
| **altitude_actor** | Vertical control | Altitude PID, landing detection |
| **position_actor** | Horizontal control | Position PID |
| **attitude_actor** | Angle control | Attitude PIDs |
| **rate_actor** | Angular rate control | Rate PIDs |
| **motor_actor** | Motor output | Gate outputs (arm/disarm done by flight_manager via HAL) |
| **waypoint_actor** | Mission | Track waypoint sequence |
| **flight_manager** | Flight phases | Orchestrate state machine, self-tests |
| **logger_actor** | Logs | Hive log + telemetry CSV lifecycle |
| **comms_actor** | Radio | Relay commands, read state_bus for telemetry, handle params |

### Preparation vs Self-Test

**Calibration** (flight_manager before RESET):
- flight_manager calls `hal_calibrate()` (gyro bias, baro zero, etc.)
- This is a system-level operation, not per-actor

**Preparation** (each actor on RESET):
- estimator_actor: reset filters, establish initial attitude/position
- PID actors: reset integrators
- logger_actor: truncate logs
- Each actor logs errors internally if prep fails

**Self-test** (flight_manager after RESET):
- System-level sanity checks
- Read state_bus, verify data is valid (not NaN, within bounds)
- Check state estimate is stable
- If any actor's prep failed, self-test catches bad data

## Message Flow

```
Ground Station
      │
      v
 comms_actor ─────────────────────────────────────┐
      │                                           │
      ├──GO/ABORT──> flight_manager               │ reads state_bus
      │                   │                       │ for telemetry
      │                   ├──RESET──> [all siblings]
      │                   │
      │                   ├──hal_arm()/hal_disarm()  (direct HAL calls)
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
| RESET | flight_manager | all siblings | notify (broadcast) |
| STATUS | comms_actor | flight_manager | request/reply |

All command messages are fire-and-forget notifications. STATUS is the only request/reply (optional feature for ground station display).

**Note:** Motor arming is done via direct HAL calls (`hal_arm()`/`hal_disarm()`), not IPC messages.

## RESET Notification

Flight manager sends `RESET` notification to all siblings during PREFLIGHT. Each actor prepares itself for a new flight. This is a fire-and-forget broadcast - no replies expected.

### Message Format

```c
#define MSG_TAG_RESET 0x5245  // "RE" in ASCII

// Flight manager broadcasts to all siblings:
for (size_t i = 0; i < sibling_count; i++) {
    hive_ipc_notify(siblings[i].id, MSG_TAG_RESET, NULL, 0);
}
```

### Actor Responses

Each actor handles RESET according to its own needs. Actors that have no state to reset simply ignore the message.

| Actor | Action on RESET |
|-------|-----------------|
| sensor_actor | Ignore (calibration done by flight_manager via hal_calibrate) |
| estimator_actor | Reset Kalman filter and complementary filter, set initial attitude/position |
| altitude_actor | Reset PID integrator, clear landing state |
| position_actor | Reset PID integrator (if any) |
| attitude_actor | Reset PID integrators |
| rate_actor | Reset PID integrators |
| motor_actor | Clear "started" flag, zero motor outputs |
| waypoint_actor | Reset waypoint index to 0, clear arrival state |
| logger_actor | Truncate hive log and telemetry CSV (see Log File Handling) |
| comms_actor | Ignore (stateless relay) |

### Implementation Pattern

```c
// In each actor's main loop, check for RESET:
hive_message_t msg;
if (HIVE_SUCCEEDED(hive_ipc_recv_match(HIVE_SENDER_ANY, HIVE_MSG_NOTIFY,
                                        MSG_TAG_RESET, &msg, 0))) {
    // Reset actor-specific state
    pid_reset(&my_pid);
    // ... other resets
}
```

Actors that use `hive_select()` for multiple event sources add RESET as a filter option.

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

If calibration or any actor's preparation fails (filter init, etc.), the state data will be invalid. flight_manager's self-test catches this:

- Self-test reads state_bus, checks for valid data (not NaN, within bounds)
- If invalid: log error, return to IDLE without arming
- Check logs for specific failure (each actor logs its own errors)
- Comms_actor can report failure to ground station via telemetry

### Flight Abort

If emergency cutoff triggers during FLYING:
- Motors are killed by altitude_actor (existing behavior)
- Flight manager detects armed=false, transitions to LANDED
- Normal LANDED cleanup, return to IDLE
- System ready for another GO (after operator investigates)

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

### Optional Enhancement: STATUS

Query current state for ground station display:

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

1. **flight_manager_actor.c** - Rewrite as state machine loop, call hal_calibrate() in PREFLIGHT, add self-test, add auto-GO for simulation
2. **sensor_actor.c** - No changes needed (calibration done by flight_manager)
3. **motor_actor.c** - Handle RESET (clear started flag)
4. **estimator_actor.c** - Handle RESET (reset filters)
5. **altitude_actor.c** - Handle RESET (reset PID, landing state)
6. **attitude_actor.c** - Handle RESET (reset PIDs)
7. **rate_actor.c** - Handle RESET (reset PIDs)
8. **waypoint_actor.c** - Handle RESET (reset index)
9. **telemetry_logger_actor.c** - Rename to logger_actor, handle RESET (truncate both hive log and telemetry CSV), move hive log open and early flush from pilot.c to init
10. **pilot.c** - Remove hive log open/flush (now in logger_actor)
11. **pid.c** - Add `pid_reset()` function if not present
12. **tunable_params.h/c** - Add `armed_countdown_s` and `auto_go_delay_s` parameters

### Actors Unchanged

- sensor_actor.c (calibration done by flight_manager)
- position_actor.c (minimal state, or add RESET if needed)
- comms_actor.c (stateless relay)

## Testing

1. **Single flight** - Verify normal IDLE -> PREFLIGHT -> ARMED -> FLYING -> LANDING -> LANDED -> IDLE
2. **Multiple flights** - Send GO, complete flight, send GO again, verify clean state
3. **Preflight failure** - Inject bad sensor data, verify self-test fails, return to IDLE
4. **Armed countdown** - Verify 60s countdown in ARMED state, then transition to FLYING
5. **Abort during armed** - Send ABORT during countdown, verify return to IDLE, motors disarmed
6. **Abort in wrong state** - Send ABORT in IDLE or FLYING, verify ignored with warning
7. **Emergency cutoff** - Trigger emergency cutoff mid-flight, verify return to IDLE
8. **Log truncation** - Verify logs are fresh after each GO (no data from previous run)
9. **Parameter tuning** - Change params in IDLE, verify new values used in next flight
