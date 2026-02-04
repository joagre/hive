# Flight Manager Lifecycle

Specification for multi-run test support without power cycling.

## Problem

Currently, flight manager exits after completing a flight. To run another test requires a power cycle. We want to send GO multiple times in a single power-on session.

## Solution

Flight manager becomes a looping state machine. After landing, it returns to IDLE and waits for the next GO. Before each flight, it requests RESET from all siblings so actors start with clean state.

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
                          │ GO received
                          v
    ┌──────────────────────────────────────────────────────────────┐
    │                       PREFLIGHT                              │
    │  1. Request RESET from all siblings (calibrate, reset PIDs)  │
    │  2. If any RESET failed: return to IDLE                      │
    │  3. Request ARM from motor_actor                             │
    │  4. If ARM failed: return to IDLE                            │
    └──────────────────────────────────────────────────────────────┘
                          │
                          │ Preflight complete
                          v
    ┌──────────────────────────────────────────────────────────────┐
    │                        ARMED                                 │
    │  - 60 second countdown (tunable: armed_countdown_s)          │
    │  - ABORT returns to IDLE                                     │
    └──────────────────────────────────────────────────────────────┘
                          │
                          │ Countdown complete
                          v
    ┌──────────────────────────────────────────────────────────────┐
    │                        FLYING                                │
    │  - Execute flight profile                                    │
    │  - Wait for flight duration timer                            │
    └──────────────────────────────────────────────────────────────┘
                          │
                          │ Timer expires
                          v
    ┌──────────────────────────────────────────────────────────────┐
    │                       LANDING                                │
    │  - Notify altitude_actor to land                             │
    │  - Wait for LANDED notification                              │
    └──────────────────────────────────────────────────────────────┘
                          │
                          │ Landed confirmed
                          v
    ┌──────────────────────────────────────────────────────────────┐
    │                       LANDED                                 │
    │  - Request DISARM from motor_actor                           │
    │  - Transition to IDLE                                        │
    └──────────────────────────────────────────────────────────────┘
                          │
                          └────────────────────────────────────────┘
```

## Message Types

| Message | From | To | Pattern | Tag |
|---------|------|----|---------|-----|
| GO | comms_actor | flight_manager | notify | 0x474F |
| ABORT | comms_actor | flight_manager | notify | 0x4142 |
| RESET | flight_manager | all siblings | request/reply | - |
| ARM | flight_manager | motor_actor | request/reply | - |
| DISARM | flight_manager | motor_actor | request/reply | - |
| LANDING | flight_manager | altitude_actor | notify | 0x4C44 |
| LANDED | altitude_actor | flight_manager | notify | 0x444E |
| STATUS | comms_actor | flight_manager | request/reply | - |

Request/reply messages use auto-generated tags. Actors dispatch by context (flight phase), not by tag.

## Message Flow

```
Ground Station
      │
      v
 comms_actor ─────────────────────────────────────┐
      │                                           │
      ├──GO/ABORT──> flight_manager               │ reads state_bus
      │                   │                       │ for telemetry
      ├──STATUS──> flight_manager ──reply──┐      │
      │<───────────────────────────────────┘      │
      │                   │                       │
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
      └──PARAM_SET/GET──> (handled directly via tunable_params)
```

## Actor Responsibilities

| Actor | RESET Action | Other Messages |
|-------|--------------|----------------|
| sensor_actor | hal_calibrate() | - |
| estimator_actor | Reset filters | - |
| altitude_actor | Reset PID, clear landing state | Handle LANDING, send LANDED |
| position_actor | Reset PID | - |
| attitude_actor | Reset PIDs | - |
| rate_actor | Reset PIDs | - |
| motor_actor | Clear started flag, zero outputs | ARM (hal_arm), DISARM (hal_disarm) |
| waypoint_actor | Reset index | - |
| logger_actor | Truncate logs | - |
| comms_actor | No-op | Relay GO/ABORT/STATUS |
| flight_manager | - | Orchestrate all |

RESET replies: REPLY_OK (0x00) or REPLY_FAIL (0x01). If any fails, abort preflight.

## Ground Station Commands

| Command | ID | Action |
|---------|-----|--------|
| GO | 0x20 | Start flight (IDLE only) |
| ABORT | 0x21 | Cancel countdown (ARMED only) |
| STATUS | 0x22 | Query state and countdown |

STATUS response:
```c
typedef struct {
    uint8_t state;          // 0=IDLE, 1=PREFLIGHT, 2=ARMED, 3=FLYING, 4=LANDING, 5=LANDED
    uint8_t countdown_s;    // Seconds remaining (ARMED only)
} __attribute__((packed)) status_resp_t;
```

## Tunable Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `armed_countdown_s` | 60.0 | Seconds in ARMED before flight |
| `auto_go_delay_s` | 0.0 | Auto-GO delay (0 = wait for command) |

Simulation mode: `auto_go_delay_s = 2.0` in Webots (no radio available).

## Implementation Notes

1. **Sibling lookup** - Use `hive_find_sibling(siblings, sibling_count, "name")` to get actor IDs before main loop

2. **Request dispatch** - `hive_ipc_request()` auto-generates tags, so receivers use `HIVE_TAG_ANY` and dispatch by context (flight phase determines expected request type)

3. **Log truncation** - logger_actor owns both hive log and telemetry CSV; truncates on RESET via close/reopen

4. **Emergency cutoff** - altitude_actor sends LANDED for both normal landing and emergency cutoff; flight_manager handles both the same way

## Implementation (Complete)

All components implemented:

1. **flight_manager_actor.c** - State machine with IDLE → PREFLIGHT → ARMED → FLYING → LANDING → LANDED loop
2. **sensor_actor.c** - RESET handler calls hal_calibrate()
3. **motor_actor.c** - Handles RESET, ARM, DISARM requests and START/STOP notifications
4. **estimator_actor.c** - RESET handler reinitializes filters
5. **altitude_actor.c** - Handles RESET, LANDING; sends LANDED
6. **attitude_actor.c** - RESET handler resets PIDs
7. **rate_actor.c** - RESET handler resets PIDs
8. **position_actor.c** - RESET handler clears state
9. **waypoint_actor.c** - RESET handler resets waypoint index
10. **telemetry_logger_actor.c** - RESET handler truncates log file
11. **comms_actor.c** - CMD_GO, CMD_ABORT, CMD_STATUS handlers
12. **tunable_params.h/c** - Added armed_countdown_s (45.0), auto_go_delay_s (0.0, 2.0 in Webots)
13. **pilot.c** - Removed hal_calibrate/hal_arm calls, flight_manager is PERMANENT

## Testing

1. Single flight - IDLE → PREFLIGHT → ARMED → FLYING → LANDING → LANDED → IDLE
2. Multiple flights - GO, complete, GO again
3. RESET failure - hal_calibrate() fails, return to IDLE
4. ARM failure - hal_arm() fails, return to IDLE
5. Abort during armed - DISARM, return to IDLE
6. Emergency cutoff - Return to IDLE via LANDED
7. Log truncation - Fresh logs after each GO
8. STATUS polling - Correct state and countdown in each state
