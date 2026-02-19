# Forth Actor Specification

Bytecode Forth VM running as a Hive actor. Scripts uploaded over radio
compose native maneuver primitives into aerial acrobatics - flips, orbits,
barrel rolls.

The VM executes the script. The hard real-time work (publishing setpoints
at 250Hz, thrust modulation during flips) happens in native C ROM words
that the script calls. See
[acrobatics_pilot_design.md](acrobatics_pilot_design.md)
for the ROM word contract.

## Goals

1. **Extreme minimalism** - Interpreter under 2KB flash, under 512 bytes RAM overhead
2. **Actor integration** - Scripts are actors with full bus/IPC access
3. **Safe** - Script crash triggers supervisor restart, doesn't affect flight
4. **Uploadable** - Scripts compiled on ground station, uploaded as bytecode over radio, stored in flash
5. **No time-sharing complexity** - Hive scheduler provides cooperative multitasking

## Architecture

```
┌──────────────────────────────────────────────────────────────┐
│                        forth_actor                            │
│                                                              │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐      │
│  │ Data Stack  │    │ Return Stack│    │ Dictionary  │      │
│  │ (32 cells)  │    │ (16 cells)  │    │ (built-in)  │      │
│  └─────────────┘    └─────────────┘    └─────────────┘      │
│         │                  │                  │              │
│         v                  v                  v              │
│  ┌──────────────────────────────────────────────────────┐   │
│  │              Inner Interpreter                        │   │
│  │  - Fetch word from bytecode                          │   │
│  │  - Execute primitive or call secondary               │   │
│  │  - Yield every N instructions or on YIELD word       │   │
│  └──────────────────────────────────────────────────────┘   │
│                           │                                  │
│                           v                                  │
│  ┌──────────────────────────────────────────────────────┐   │
│  │              Hive Primitive Words                     │   │
│  │  BUS-READ  BUS-PUB  IPC-NOTIFY  SLEEP  PARAM@  YIELD │   │
│  └──────────────────────────────────────────────────────┘   │
│                           │                                  │
│                           v                                  │
│  ┌──────────────────────────────────────────────────────┐   │
│  │              ROM Words (native C, 0x60-0x7F)         │   │
│  │  GOTO  HOVER  ORBIT  FLIP-ROLL  BANK  FENCE  LAND   │   │
│  └──────────────────────────────────────────────────────┘   │
└──────────────────────────────────────────────────────────────┘
```

## Design Decisions

### Bytecode vs Source

**Bytecode** - Scripts compiled on ground station, uploaded as bytecode.

Rationale:
- No compiler on device (saves ~1KB flash)
- Faster execution (no parsing)
- Smaller scripts (1 byte per word vs string)
- Ground station can validate before upload

### Cell Size

**32-bit cells** - Matches ARM word size, holds floats and pointers.

```c
typedef int32_t cell_t;
typedef uint32_t ucell_t;
```

### Stack Sizes

| Stack | Cells | Bytes | Rationale |
|-------|-------|-------|-----------|
| Data | 32 | 128 | Enough for complex expressions |
| Return | 16 | 64 | Enough for 16-deep call nesting |

Stack overflow/underflow triggers actor crash (supervisor restarts).

### Yield Strategy

**Instruction counting** - Yield after N instructions (default 100).

```c
void forth_run(forth_state_t *f) {
    int count = 0;
    while (f->ip < f->code_end && count < FORTH_YIELD_INTERVAL) {
        forth_execute_one(f);
        count++;
    }
    // Returns to scheduler, will be called again next tick
}
```

Also yield explicitly via `YIELD` word for event waiting.

### No User Definitions

**Built-in dictionary only** - No `:` definitions, no `CREATE`, no `VARIABLE`.

The ground station compiler handles all abstraction. The on-device VM is
a pure bytecode executor - no dictionary management, no `FIND`, no
compilation state. This saves ~500 bytes and eliminates an entire class
of runtime errors.

If needed later: allow bytecode to define secondary words as subroutines
(call/return using the return stack, no dictionary needed).

## Primitive Words

### Stack Manipulation (10 words)

| Word | Stack | Description |
|------|-------|-------------|
| `DUP` | ( a -- a a ) | Duplicate top |
| `DROP` | ( a -- ) | Discard top |
| `SWAP` | ( a b -- b a ) | Swap top two |
| `OVER` | ( a b -- a b a ) | Copy second to top |
| `ROT` | ( a b c -- b c a ) | Rotate three |
| `NIP` | ( a b -- b ) | Drop second |
| `TUCK` | ( a b -- b a b ) | Copy top below second |
| `>R` | ( a -- ) R:( -- a ) | Move to return stack |
| `R>` | R:( a -- ) ( -- a ) | Move from return stack |
| `R@` | R:( a -- a ) ( -- a ) | Copy from return stack |

### Arithmetic (10 words)

| Word | Stack | Description |
|------|-------|-------------|
| `+` | ( a b -- a+b ) | Add |
| `-` | ( a b -- a-b ) | Subtract |
| `*` | ( a b -- a*b ) | Multiply |
| `/` | ( a b -- a/b ) | Divide |
| `MOD` | ( a b -- a%b ) | Modulo |
| `NEGATE` | ( a -- -a ) | Negate |
| `ABS` | ( a -- |a| ) | Absolute value |
| `MIN` | ( a b -- min ) | Minimum |
| `MAX` | ( a b -- max ) | Maximum |
| `*/` | ( a b c -- a*b/c ) | Scale (no overflow) |

### Floating Point (8 words)

| Word | Stack | Description |
|------|-------|-------------|
| `F+` | ( a b -- a+b ) | Float add |
| `F-` | ( a b -- a-b ) | Float subtract |
| `F*` | ( a b -- a*b ) | Float multiply |
| `F/` | ( a b -- a/b ) | Float divide |
| `F>S` | ( f -- n ) | Float to int |
| `S>F` | ( n -- f ) | Int to float |
| `FABS` | ( f -- |f| ) | Float absolute |
| `FSQRT` | ( f -- sqrt(f) ) | Square root |

### Comparison (8 words)

| Word | Stack | Description |
|------|-------|-------------|
| `=` | ( a b -- flag ) | Equal |
| `<>` | ( a b -- flag ) | Not equal |
| `<` | ( a b -- flag ) | Less than |
| `>` | ( a b -- flag ) | Greater than |
| `<=` | ( a b -- flag ) | Less or equal |
| `>=` | ( a b -- flag ) | Greater or equal |
| `0=` | ( a -- flag ) | Equal to zero |
| `0<` | ( a -- flag ) | Less than zero |

### Logic (4 words)

| Word | Stack | Description |
|------|-------|-------------|
| `AND` | ( a b -- a&b ) | Bitwise and |
| `OR` | ( a b -- a|b ) | Bitwise or |
| `XOR` | ( a b -- a^b ) | Bitwise xor |
| `NOT` | ( a -- ~a ) | Bitwise invert |

### Control Flow (6 words)

| Word | Stack | Description |
|------|-------|-------------|
| `IF` | ( flag -- ) | Begin conditional |
| `ELSE` | ( -- ) | Alternative branch |
| `THEN` | ( -- ) | End conditional |
| `BEGIN` | ( -- ) | Begin loop |
| `UNTIL` | ( flag -- ) | Loop until true |
| `AGAIN` | ( -- ) | Loop forever |

Bytecode uses relative jumps. No `DO`/`LOOP` (complexity not worth it).

### Hive Integration (12 words)

| Word | Stack | Description |
|------|-------|-------------|
| `YIELD` | ( -- ) | Yield to scheduler |
| `SLEEP` | ( ms -- ) | Sleep for milliseconds |
| `BUS-READ` | ( bus-id -- ptr len flag ) | Read from bus |
| `BUS-PUB` | ( ptr len bus-id -- flag ) | Publish to bus |
| `IPC-NOTIFY` | ( ptr len tag actor -- flag ) | Send notification |
| `PARAM@` | ( id -- f ) | Get tunable param |
| `PARAM!` | ( f id -- flag ) | Set tunable param |
| `TIME` | ( -- ms ) | Current time in ms |
| `PRINT` | ( n -- ) | Debug print integer |
| `PRINT-F` | ( f -- ) | Debug print float |
| `SELF` | ( -- actor-id ) | Own actor ID |
| `EXIT` | ( code -- ) | Exit actor |

### Memory (4 words)

| Word | Stack | Description |
|------|-------|-------------|
| `@` | ( addr -- n ) | Fetch from address |
| `!` | ( n addr -- ) | Store to address |
| `C@` | ( addr -- c ) | Fetch byte |
| `C!` | ( c addr -- ) | Store byte |

**Safety** - Memory access restricted to script's local buffer (256 bytes).

### Literals (2 opcodes)

| Opcode | Encoding | Description |
|--------|----------|-------------|
| `LIT` | `0x80 + 4 bytes` | Push 32-bit literal |
| `SLIT` | `0x81 + 1 byte` | Push small literal (-64 to 63) |

## Bytecode Format

### Header

```c
typedef struct {
    uint8_t magic[2];      // "FH" (Forth Hive)
    uint8_t version;       // Bytecode version
    uint8_t flags;         // Reserved
    uint16_t code_size;    // Bytecode length
    uint16_t data_size;    // Local data buffer size (max 256)
    uint32_t checksum;     // CRC32 of code
} forth_header_t;
```

### Opcodes

Single-byte opcodes. Three ranges:

| Range | Purpose | Count |
|-------|---------|-------|
| 0x00-0x5F | VM primitives (stack, arithmetic, control, hive) | ~60 |
| 0x60-0x7F | ROM words (native C maneuver primitives) | 32 slots |
| 0x80+ | Literals and extended encodings | variable |

ROM words are registered by the maneuver actor at init. The VM's inner
interpreter dispatches them through a function pointer table - no
special handling needed beyond the table lookup. See
[acrobatics_pilot_design.md](acrobatics_pilot_design.md)
for the complete ROM word reference.

```c
enum {
    OP_DUP = 0x00,
    OP_DROP = 0x01,
    OP_SWAP = 0x02,
    // ... stack ops 0x00-0x0F

    OP_ADD = 0x10,
    OP_SUB = 0x11,
    // ... arithmetic 0x10-0x1F

    OP_EQ = 0x20,
    OP_LT = 0x21,
    // ... comparison 0x20-0x2F

    OP_IF = 0x30,
    OP_ELSE = 0x31,
    OP_THEN = 0x32,
    // ... control flow 0x30-0x3F

    OP_YIELD = 0x40,
    OP_SLEEP = 0x41,
    OP_BUS_READ = 0x42,
    // ... hive integration 0x40-0x5F

    // ROM words 0x60-0x7F (see acrobatics_pilot_design.md)
    OP_ROM_BASE = 0x60,

    OP_LIT = 0x80,      // followed by 4 bytes
    OP_SLIT = 0x81,     // followed by 1 byte (signed)
    OP_JUMP = 0x82,     // followed by 2 bytes (relative)
    OP_JUMP0 = 0x83,    // jump if zero
};
```

## Actor Integration

### Initialization

```c
typedef struct {
    pilot_buses_t *buses;
    tunable_params_t *params;
    const uint8_t *bytecode;
    size_t bytecode_len;
} forth_actor_args_t;

void forth_actor(void *args, const hive_spawn_info_t *siblings, size_t n);
```

### State

```c
typedef struct {
    // Stacks
    cell_t dstack[FORTH_DSTACK_SIZE];  // Data stack
    cell_t rstack[FORTH_RSTACK_SIZE];  // Return stack
    int dsp;                            // Data stack pointer
    int rsp;                            // Return stack pointer

    // Execution
    const uint8_t *code;               // Bytecode
    const uint8_t *code_end;           // End of bytecode
    const uint8_t *ip;                 // Instruction pointer

    // Local memory
    uint8_t data[FORTH_DATA_SIZE];     // Local buffer (256 bytes)

    // Hive integration
    pilot_buses_t *buses;
    tunable_params_t *params;
    hive_actor_id_t self;
} forth_state_t;
```

### Main Loop

```c
void forth_actor(void *args, const hive_spawn_info_t *siblings, size_t n) {
    forth_actor_args_t *a = args;
    forth_state_t f;

    if (!forth_init(&f, a->bytecode, a->bytecode_len)) {
        HIVE_LOG_ERROR("[FORTH] Invalid bytecode");
        hive_exit(HIVE_EXIT_REASON_CRASH);
    }

    f.buses = a->buses;
    f.params = a->params;
    f.self = hive_self();

    while (true) {
        int result = forth_run(&f, FORTH_YIELD_INTERVAL);

        if (result == FORTH_DONE) {
            HIVE_LOG_INFO("[FORTH] Script completed");
            return;  // Normal exit
        }
        if (result == FORTH_ERROR) {
            HIVE_LOG_ERROR("[FORTH] Runtime error");
            hive_exit(HIVE_EXIT_REASON_CRASH);
        }
        // FORTH_YIELD - continue next tick
        hive_yield();
    }
}
```

## Script Upload Protocol

### Radio Commands

| Command | ID | Payload | Description |
|---------|-----|---------|-------------|
| `CMD_SCRIPT_START` | 0x40 | header (12 bytes) | Begin script upload |
| `CMD_SCRIPT_CHUNK` | 0x41 | offset + data (28 bytes) | Script chunk |
| `CMD_SCRIPT_END` | 0x42 | checksum | Finalize and verify |
| `CMD_SCRIPT_RUN` | 0x43 | slot | Start script actor |
| `CMD_SCRIPT_STOP` | 0x44 | slot | Stop script actor |
| `CMD_SCRIPT_LIST` | 0x45 | - | List stored scripts |

### Storage

Scripts stored in flash (up to 4 slots, 1KB each).

```c
#define FORTH_SCRIPT_SLOTS 4
#define FORTH_SCRIPT_MAX_SIZE 1024

// Flash layout (4KB total)
// 0x08090000: Script 0
// 0x08090400: Script 1
// 0x08090800: Script 2
// 0x08090C00: Script 3
```

## Example Scripts

These use ROM words from
[acrobatics_pilot_design.md](acrobatics_pilot_design.md).
The VM executes the sequencing; ROM words handle the real-time control.

### Orbit Mission

```forth
-5 S>F -5 S>F 0 S>F 5 S>F 5 S>F 3 S>F FENCE
1 OVERRIDE
HOVER

( Takeoff to 1m )
0 S>F 0 S>F 1 S>F 0 S>F GOTO
2 WAIT-UNTIL DROP

( Orbit: 2m diameter, 0.5 rad/s )
POS-X@ POS-Y@ 1 S>F 500 1000 */ S>F ORBIT

( Land )
HOVER 0 WAIT-UNTIL DROP
1 RELEASE
LAND DROP
```

### Barrel Roll Mission

```forth
-3 S>F -3 S>F 0 S>F 3 S>F 3 S>F 3 S>F FENCE
1 OVERRIDE
HOVER

( Takeoff to 1.5m - need margin for altitude loss during flip )
0 S>F 0 S>F 1500 1000 */ S>F 0 S>F GOTO
2 WAIT-UNTIL DROP

( Flip )
200 S>F 1800 S>F ASSERT-ALT
4 OVERRIDE
1 FLIP-ROLL
4 RELEASE

( Stabilize and land )
HOVER 1000 WAIT-MS
1 RELEASE
LAND DROP
```

See the acrobatics design doc for two more examples (flip with recovery,
figure-8) and the full ROM word reference with stack effects.

## Implementation Estimate

| Component | Lines | Flash |
|-----------|-------|-------|
| Inner interpreter | 150 | 400B |
| Stack primitives | 100 | 200B |
| Arithmetic | 80 | 200B |
| Float ops | 60 | 300B |
| Comparison/logic | 60 | 150B |
| Control flow | 80 | 200B |
| Hive integration | 150 | 400B |
| Memory ops | 40 | 100B |
| **Total** | **~720** | **~2KB** |

Plus forth_actor.c wrapper: ~100 lines, ~300B flash.

## Future Extensions

1. **User definitions** - Allow `: name ... ;` in bytecode (call/return via return stack, adds ~300B)
2. **Watchdog** - Kill script if it doesn't yield within timeout
3. **Multiple scripts** - Run several forth_actors concurrently
4. **String support** - `." text"` for debug output
5. **Ground station compiler** - Forth source to bytecode, with validation
