# Review Response - 2025-01-23

Response to reviewer comments in `review_250123.md`.

## Categorization

Each item is categorized as:
- **DONE** - Already implemented or documented
- **FIX** - Will fix now (code or documentation)
- **FUTURE** - Valid suggestion, added to Future Extensions in spec/internals.md
- **WONTFIX** - Outside scope or conflicts with design philosophy

---

## 1. Core Runtime Improvements

### 1.1 Mailboxes and Selective Receive

**Status: DONE**

The O(n) selective receive behavior is already documented in spec/design.md section "3. Selective Receive is O(n) per Mailbox Scan" (commit `d6e0aed`).

Clarifications:
- Mailbox bounds ARE compile-time via shared pool (`HIVE_MAILBOX_ENTRY_POOL_SIZE=256`)
- This is a global pool, not per-actor limit
- The "no save queue" optimization is documented as acceptable for embedded use

**Action:** Add clarification that pool size bounds total mailbox entries system-wide.

---

### 1.2 Guaranteed EXIT / Monitor Delivery

**Status: FUTURE**

Valid concern. Under pool exhaustion, EXIT messages compete with normal IPC.

Current mitigation:
- EXIT messages are small (minimal payload)
- Supervisors typically have shallow mailboxes (only receive EXIT)
- Pool exhaustion is a systemic error that should trigger shutdown anyway

Future option: Reserved EXIT slot per actor (adds ~8 bytes per actor).

**Action:** Add to Future Extensions in spec/internals.md. Document current behavior and risk.

---

### 1.3 I/O Dispatch Starvation

**Status: FUTURE**

Valid concern. If run queue never empties, `epoll_wait` is never called.

Current mitigation:
- Well-behaved actors yield regularly
- Flight-critical actors block on `recv` or `select`

Future option: Bounded dispatch (poll I/O every N actor runs).

**Action:** Add to Future Extensions in spec/internals.md. Document as known limitation.

---

### 1.4 Priority Starvation Guardrail

**Status: FUTURE**

Already acknowledged in spec/design.md as a trade-off. Strict priority is intentional for determinism.

Future option: Opt-in fairness mode (compile-time).

**Action:** Already documented. Add fairness option to Future Extensions.

---

### 1.5 File I/O Semantics

**Status: DONE**

Already documented in CLAUDE.md and spec/api.md:
> "Safety-critical caveat: Restrict file I/O to initialization, shutdown, or non-time-critical phases"

**Action:** Verify documentation is prominent. No code changes needed.

---

## 2. Pilot Example Improvements

### 2.1 Motor Deadman Watchdog

**Status: DONE** ✓

Implemented in `motor_actor.c` using `hive_select()` with timeout.

Implementation:
- Motor actor uses `hive_select()` with `MOTOR_DEADMAN_TIMEOUT_MS` (50ms)
- On timeout: zero all motors, log warning
- Documented in examples/pilot/spec/

**Commits:** `0b3326f`

---

### 2.2 Bus Retention on Subscribe

**Status: FUTURE**

Current behavior: Late subscribers see no data until next publish.

This is acceptable for the pilot because:
- Sensor actor publishes at 250Hz (4ms)
- New subscriber gets data within 4ms
- Supervisor restart is ordered (publishers first)

Future option: "retain latest" flag for buses with `max_entries=1`.

**Action:** Document current behavior explicitly. Add to Future Extensions.

---

### 2.3 Supervision Semantics Clarification

**Status: DONE** ✓

Documented in examples/pilot/spec/design.md "Supervision Semantics" section:
- ONE_FOR_ALL restart resets all state (integrators, filters)
- Why ONE_FOR_ALL is correct for flight controller
- Comms actor isolation via TEMPORARY restart type

**Commits:** (this batch)

---

### 2.4 Error Handling Rules

**Status: DONE** ✓

Documented in spec/api.md "Actor Exit Semantics" section:
- Returning from actor function = `hive_exit(CRASH)`
- `HIVE_ERR_TIMEOUT` is not a crash (normal control flow)
- `HIVE_ERR_NOMEM` is systemic error (should be surfaced)

**Commits:** `11a267a`

---

### 2.5 Priority and Blocking Table

**Status: DONE** ✓

Added "Actor Priority and Blocking Table" to examples/pilot/spec/design.md showing:
- Actor name, priority, primary blocking point, yield behavior

**Commits:** `11a267a`

---

## 3. Documentation Improvements

### 3.1 Guarantees vs Non-Guarantees Page

**Status: DONE** ✓

Added "Guarantees and Non-Guarantees" section to spec/design.md with:
- Table of guaranteed properties
- Table of NOT guaranteed properties
- "Honest Positioning" subsection (Hive is / is not)

**Commits:** `11a267a`

---

### 3.2 Failure Modes and Backpressure Patterns

**Status: DONE** ✓

Added "Failure Modes Summary" section to spec/design.md with:
- Pool exhaustion behavior table (IPC vs bus differences)
- Recommended patterns table (check and retry, backpressure, drop, escalate)
- Error code classification table

**Commits:** (this batch)

---

### 3.3 Memory Sizing Guide

**Status: DONE** ✓

Added "Memory Sizing Guide" section to spec/internals.md with:
- Fixed overhead calculations
- Pool memory calculations
- Sizing formulae table
- Flight controller example config
- Server application example config

**Commits:** (this batch)

---

### 3.4 Simulation Invariant

**Status: DONE** ✓

Added "Simulation Invariant" section to spec/design.md with:
- Contract that actors MUST block
- Correct and incorrect code patterns
- Why this matters (simulation vs production difference)
- Diagnosis guidance

**Commits:** (this batch)

---

## 4. Positioning Summary

**Status: DONE** ✓

Added "Hive is / Hive is not" positioning to README.md introduction.

**Commits:** `11a267a`

---

## Summary

| Section | Status | Action Required |
|---------|--------|-----------------|
| 1.1 Selective Receive | DONE | Minor clarification |
| 1.2 EXIT Delivery | FUTURE | ✓ Added to Future Extensions |
| 1.3 I/O Starvation | FUTURE | ✓ Added to Future Extensions |
| 1.4 Priority Starvation | FUTURE | ✓ Added to Future Extensions |
| 1.5 File I/O | DONE | Verify docs |
| 2.1 Motor Deadman | **DONE** | ✓ Implemented |
| 2.2 Bus Retention | FUTURE | ✓ Added to Future Extensions |
| 2.3 Supervision Semantics | **DONE** | ✓ Documented |
| 2.4 Error Handling | **DONE** | ✓ Documented |
| 2.5 Priority Table | **DONE** | ✓ Documented |
| 3.1 Guarantees Page | **DONE** | ✓ Documented |
| 3.2 Failure Modes | **DONE** | ✓ Documented |
| 3.3 Memory Sizing | **DONE** | ✓ Documented |
| 3.4 Simulation Invariant | **DONE** | ✓ Documented |
| 4. Positioning | **DONE** | ✓ Documented |

**All items complete!**
- Code changes: 1 (motor deadman)
- Documentation: 10 items
- Future extensions: 4 items

---

## Execution Order

1. ~~**Motor deadman watchdog** (code)~~ ✓ Done
2. ~~**Guarantees vs Non-Guarantees** (doc)~~ ✓ Done
3. ~~**Error handling rules** (doc)~~ ✓ Done
4. ~~**Pilot priority/blocking table** (doc)~~ ✓ Done
5. ~~**Future Extensions section**~~ ✓ Done
6. ~~**Positioning summary** (README)~~ ✓ Done
7. ~~**Supervision Semantics** (2.3)~~ ✓ Done
8. ~~**Failure Modes** (3.2)~~ ✓ Done
9. ~~**Memory Sizing Guide** (3.3)~~ ✓ Done
10. ~~**Simulation Invariant** (3.4)~~ ✓ Done

**All reviewer feedback addressed!**

