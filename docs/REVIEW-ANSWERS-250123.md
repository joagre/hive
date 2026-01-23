# Review Response - 2025-01-23

Response to reviewer comments in `REVIEW-250123.md`.

## Categorization

Each item is categorized as:
- **DONE** - Already implemented or documented
- **FIX** - Will fix now (code or documentation)
- **FUTURE** - Valid suggestion, added to Future Extensions in SPEC.md
- **WONTFIX** - Outside scope or conflicts with design philosophy

---

## 1. Core Runtime Improvements

### 1.1 Mailboxes and Selective Receive

**Status: DONE**

The O(n) selective receive behavior is already documented in SPEC.md section "3. Selective Receive is O(n) per Mailbox Scan" (commit `d6e0aed`).

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

**Action:** Add to Future Extensions in SPEC.md. Document current behavior and risk.

---

### 1.3 I/O Dispatch Starvation

**Status: FUTURE**

Valid concern. If run queue never empties, `epoll_wait` is never called.

Current mitigation:
- Well-behaved actors yield regularly
- Flight-critical actors block on `recv` or `select`

Future option: Bounded dispatch (poll I/O every N actor runs).

**Action:** Add to Future Extensions in SPEC.md. Document as known limitation.

---

### 1.4 Priority Starvation Guardrail

**Status: FUTURE**

Already acknowledged in SPEC.md as a trade-off. Strict priority is intentional for determinism.

Future option: Opt-in fairness mode (compile-time).

**Action:** Already documented. Add fairness option to Future Extensions.

---

### 1.5 File I/O Semantics

**Status: DONE**

Already documented in CLAUDE.md and SPEC.md:
> "Safety-critical caveat: Restrict file I/O to initialization, shutdown, or non-time-critical phases"

**Action:** Verify documentation is prominent. No code changes needed.

---

## 2. Pilot Example Improvements

### 2.1 Motor Deadman Watchdog

**Status: FIX**

Valid safety concern. Motors should zero if no command received within timeout.

Implementation:
- Motor actor uses `hive_ipc_recv` with timeout
- On timeout: zero all motors, log warning
- Timeout period: configurable, e.g., 50ms (12-13 ticks at 4ms)

**Action:** Implement in motor actor. Update pilot SPEC.md.

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

**Status: FIX**

Need to document:
- ONE_FOR_ALL restart resets all state (integrators, filters)
- Comms actor is outside flight-critical supervision tree

**Action:** Add section to pilot SPEC.md.

---

### 2.4 Error Handling Rules

**Status: FIX**

Need to document explicitly:
- Returning from actor function = `hive_exit(CRASH)`
- `HIVE_ERR_TIMEOUT` is not a crash (normal control flow)
- `HIVE_ERR_NOMEM` is systemic error (should be surfaced)

**Action:** Add to runtime SPEC.md and pilot SPEC.md.

---

### 2.5 Priority and Blocking Table

**Status: FIX**

Good documentation addition. Shows scheduling design is intentional.

**Action:** Add table to pilot SPEC.md showing:
- Actor name, priority, primary blocking point, yield behavior

---

## 3. Documentation Improvements

### 3.1 Guarantees vs Non-Guarantees Page

**Status: FIX**

Excellent suggestion. Add clear, testable list.

**Action:** Add section to SPEC.md.

---

### 3.2 Failure Modes and Backpressure Patterns

**Status: FIX**

Document:
- Pool exhaustion behavior (returns HIVE_ERR_NOMEM, does not block)
- Recommended patterns (retry with backoff, drop oldest, escalate)

**Action:** Add section to SPEC.md.

---

### 3.3 Memory Sizing Guide

**Status: FIX**

Provide formulae and example configs.

**Action:** Add section to SPEC.md.

---

### 3.4 Simulation Invariant

**Status: FIX**

Document that `hive_run_until_blocked()` requires actors to eventually block.

**Action:** Add to SPEC.md simulation section.

---

## 4. Positioning Summary

**Status: FIX**

The "Hive is / Hive is not" framing is good. The tagline is accurate.

**Action:** Add to README.md introduction.

---

## Summary

| Section | Status | Action Required |
|---------|--------|-----------------|
| 1.1 Selective Receive | DONE | Minor clarification |
| 1.2 EXIT Delivery | FUTURE | Document + future extension |
| 1.3 I/O Starvation | FUTURE | Document + future extension |
| 1.4 Priority Starvation | FUTURE | Already documented, add future option |
| 1.5 File I/O | DONE | Verify docs |
| 2.1 Motor Deadman | FIX | **Implement in code** |
| 2.2 Bus Retention | FUTURE | Document + future extension |
| 2.3 Supervision Semantics | FIX | Document in pilot SPEC |
| 2.4 Error Handling | FIX | Document in both SPECs |
| 2.5 Priority Table | FIX | Document in pilot SPEC |
| 3.1 Guarantees Page | FIX | Add to SPEC |
| 3.2 Failure Modes | FIX | Add to SPEC |
| 3.3 Memory Sizing | FIX | Add to SPEC |
| 3.4 Simulation Invariant | FIX | Add to SPEC |
| 4. Positioning | FIX | Add to README |

**Code changes:** 1 (motor deadman)
**Documentation changes:** 13
**Future extensions:** 4

---

## Execution Order

1. **Motor deadman watchdog** (code) - Most important for credibility
2. **Guarantees vs Non-Guarantees** (doc) - Sets honest expectations
3. **Error handling rules** (doc) - Clarifies crash semantics
4. **Pilot priority/blocking table** (doc) - Shows scheduling intent
5. **Remaining documentation** - In order of section numbers
6. **Future Extensions section** - Collect all FUTURE items

