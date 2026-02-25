# RC522 Driver – Code Quality Report

## Summary

Overall the driver is well-structured and covers the full MIFARE Classic card workflow
(init, detect, authenticate, read, write, halt). The public API is clean and the register
map is clearly documented through `#define` constants. The areas below can be strengthened
to improve correctness, safety, and maintainability.

---

## Issues Found and Fixed in This PR

### 1. Critical – Missing `RC522_SelectCard` in `RC522_CheckForCard`

**Severity:** High (functional bug)

The MIFARE protocol requires three steps before authentication:
`REQA → AntiCollision → Select`. The Select step was absent, so every call to
`RC522_ReadCardBlock` / `RC522_WriteCardBlock` would attempt to authenticate a card
that had never been selected, causing auth to fail on real hardware.

**Fix:** Added `RC522_SelectCard` as the third step inside `RC522_CheckForCard`.

---

### 2. Bug – `RC522_DeInit` Did Not Reset `initialized` Flag

**Severity:** Medium

After `RC522_DeInit` completed, `RC->initialized` remained `true`. Any caller checking
this flag before reusing the struct would see stale state.

**Fix:** Added `RC->initialized = false` before returning.

---

### 3. Dead Code – Commented-Out `do-while` Loop in `RC522_Transceive`

**Severity:** Low (maintainability)

A complete alternative wait-loop implementation was left commented out (9 lines).
Dead code increases cognitive load and can confuse future maintainers.

**Fix:** Removed the commented-out block.

---

### 4. Dead Code – Unreachable `else` Branch in `RC522_Auth`

**Severity:** Low (maintainability)

An early-return guard already ensures `keyType` is `'A'` or `'B'` before the
`if/else if/else` chain. The final `else { return STATUS_ERROR; }` was therefore
unreachable.

**Fix:** Simplified to `if/else`.

---

### 5. Missing `const` on Read-Only Pointer Parameters

**Severity:** Low (correctness / API clarity)

Several functions accepted `uint8_t *` for data they only read (e.g. `key`, `uid`,
`data_in`, `sendData`). This prevents callers from passing `const`-qualified pointers
and hides intent.

**Fix:** Added `const` qualifiers to all input-only pointer parameters throughout
`RC522.h` and `RC522.c`.

---

### 6. Comment Typos

**Severity:** Low (readability)

Multiple spelling errors appeared in code comments:

| Location | Original | Corrected |
|---|---|---|
| `RC522_Read_Reg` / `RC522_Write_Reg` | `Left Shit` | `Left Shift` |
| `RC522_ReqA` | `Detecct` | `Detect` |
| `RC522_ReqA` / `RC522_AntiCol` | `recieved` | `received` |
| `RC522_Auth` | `succesfull` | `successful` |
| `RC522_CRC` | `bitmanuplation` | removed / reworded |
| `RC522_Transceive` | `recieved amt` | `received amount` |

---

## Remaining Recommendations (Not Yet Implemented)

### A. Expand Status Codes

The driver returns only `STATUS_OK` or `STATUS_ERROR`. Callers have no way to distinguish
a timeout from a CRC mismatch or an authentication failure. Consider:

```c
typedef enum {
    STATUS_OK = 0,
    STATUS_ERROR_TIMEOUT,
    STATUS_ERROR_CRC,
    STATUS_ERROR_AUTH,
    STATUS_ERROR_INVALID_ARG
} RC522_STATUS_TypeDef;
```

### B. `RC522_CRC` Has No Timeout Error Return

If the CRC engine hangs, `RC522_CRC` silently returns zeros for `msb`/`lsb`. Changing
the return type to `RC522_STATUS_TypeDef` (or passing a success flag) would let callers
detect this condition.

### C. Multi-Size UID Support

`RC522_AntiCol` only handles single-cascade (4-byte) UIDs. MIFARE cards with 7-byte or
10-byte UIDs (double/triple cascade, cascade bytes `0x88`) are not supported. The select
command would need additional cascade levels.

### D. Sector Trailer Write Protection

`RC522_Write_Card` does not block writes to sector trailer blocks (block 3 of each
sector, i.e. blocks 3, 7, 11, …). Writing to a sector trailer with incorrect key/access
bit data can permanently lock a card.

### E. Missing Null-Check in `RC522_HALT`

`RC522_HALT` dereferences `RC` without checking for `NULL`. All other public functions
validate their `RC` argument; this one should too.

### F. Inconsistent Indentation Style

`RC522_Read_Reg` and `RC522_Write_Reg` use no indentation inside the function body,
while all other functions use 4-space indentation. Applying a consistent style (e.g. via
`clang-format`) across the file would improve readability.

### G. No CI / Build Pipeline

There is no `Makefile`, CMake configuration, or CI workflow. Adding a basic build check
(even a syntax-only compile with a mock HAL stub) would catch regressions early.

### H. No Unit Tests

The driver has no test suite. A lightweight mock of the STM32 HAL SPI functions would
allow the protocol logic (CRC, BCC, frame construction, state machine) to be validated
without real hardware.
