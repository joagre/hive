# Flight Manager Lifecycle

Specification for multi-run test support without power cycling.

## Problem

Currently, flight manager exits after completing a flight. To run another test requires a power cycle. We want to send GO multiple times in a single power-on session.

## Solution

Flight manager becomes a looping state machine. After landing, it returns to IDLE and waits for the next GO. Before each flight, it notifies all siblings to RESET so actors start with clean state.

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
    │  1. Call hal_calibrate() (sensor calibration)                │
    │  2. Notify RESET to all siblings (reset PIDs, clear state)   │
    │  3. Call hal_arm() (enable motors)                           │
    └──────────────────────────────────────────────────────────────┘
                          │
                          │ Preflight complete
                          v
    ┌──────────────────────────────────────────────────────────────┐
    │                        ARMED                                 │
    │  - 10 second countdown (tunable: armed_countdown_s)          │
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
    │  - Call hal_disarm() (disable motors)                        │
    │  - Transition to IDLE                                        │
    └──────────────────────────────────────────────────────────────┘
                          │
                          └────────────────────────────────────────┘
```

## Message Types

| Message | From | To | Pattern | Tag |
|---------|------|----|---------|-----|
| GO | comms_actor | flight_manager | notify | NOTIFY_GO |
| ABORT | comms_actor | flight_manager | notify | NOTIFY_ABORT |
| RESET | flight_manager | all siblings | notify | NOTIFY_RESET |
| START | flight_manager | motor, waypoint | notify | NOTIFY_FLIGHT_START |
| STOP | flight_manager | motor | notify | NOTIFY_FLIGHT_STOP |
| LANDING | flight_manager | altitude_actor | notify | NOTIFY_LANDING |
| LANDED | altitude_actor | flight_manager | notify | NOTIFY_FLIGHT_LANDED |
| STATUS | comms_actor | flight_manager | request/reply | - |

HAL calls (hal_calibrate, hal_arm, hal_disarm) are made directly by flight_manager.

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
      │                   ├──RESET──> [all siblings] (fire-and-forget)
      │                   │
      │                   ├──START──> motor_actor, waypoint_actor
      │                   ├──STOP───> motor_actor
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
| sensor_actor | Log only (calibration done by flight_manager) | - |
| estimator_actor | Reset filters | - |
| altitude_actor | Reset PID, clear landing state | Handle LANDING, send LANDED |
| position_actor | Reset state | - |
| attitude_actor | Reset PIDs | - |
| rate_actor | Reset PIDs | - |
| motor_actor | Clear started flag, zero outputs | Handle START/STOP |
| waypoint_actor | Reset index | Handle START |
| logger_actor | Truncate CSV, sync hive log | - |
| comms_actor | No-op | Relay GO/ABORT/STATUS |
| flight_manager | hal_calibrate, hal_arm, hal_disarm | Orchestrate all |

RESET is fire-and-forget. HAL calls (calibrate, arm, disarm) are made directly by flight_manager.

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
| `armed_countdown_s` | 10.0 | Seconds in ARMED before flight (5.0 in sim) |
| `auto_go_delay_s` | 0.0 | Auto-GO delay (0 = wait for command) |

Simulation mode: `auto_go_delay_s = 2.0` in Webots (no radio available).

## Implementation Notes

1. **Sibling lookup** - Use `hive_find_sibling(siblings, sibling_count, "name")` to get actor IDs before main loop

2. **Fire-and-forget RESET** - RESET uses notify (not request/reply) for simplicity. Actors reset their internal state without acknowledgment.

3. **HAL ownership** - Flight manager owns HAL lifecycle calls (hal_calibrate, hal_arm, hal_disarm) to keep hardware coordination in one place.

4. **Log ownership** - logger_actor owns both hive log and telemetry CSV; opens at startup, syncs periodically, truncates CSV on RESET

5. **Emergency cutoff** - altitude_actor sends LANDED for both normal landing and emergency cutoff; flight_manager handles both the same way

## Implementation (Complete)

All components implemented:

1. **flight_manager_actor.c** - State machine with IDLE → PREFLIGHT → ARMED → FLYING → LANDING → LANDED loop. Calls hal_calibrate/hal_arm/hal_disarm directly.
2. **sensor_actor.c** - RESET handler logs acknowledgment
3. **motor_actor.c** - Handles RESET notification and START/STOP notifications
4. **estimator_actor.c** - RESET handler reinitializes filters
5. **altitude_actor.c** - Handles RESET notification, LANDING; sends LANDED
6. **attitude_actor.c** - RESET handler resets PIDs
7. **rate_actor.c** - RESET handler resets PIDs
8. **position_actor.c** - RESET handler clears state
9. **waypoint_actor.c** - RESET handler resets waypoint index
10. **logger_actor.c** - Owns hive log and CSV lifecycle, RESET truncates CSV
11. **comms_actor.c** - CMD_GO, CMD_ABORT, CMD_STATUS handlers
12. **tunable_params.h/c** - Added armed_countdown_s (45.0), auto_go_delay_s (0.0, 2.0 in Webots)
13. **pilot.c** - Removed hal_calibrate/hal_arm calls, flight_manager is PERMANENT

## Testing

1. Single flight - IDLE → PREFLIGHT → ARMED → FLYING → LANDING → LANDED → IDLE
2. Multiple flights - GO, complete, GO again
3. Abort during armed - DISARM, return to IDLE
4. Emergency cutoff - Return to IDLE via LANDED
5. Log truncation - Fresh logs after each GO
6. STATUS polling - Correct state and countdown in each state
