# Bramble RP2040 Emulator - Roadmap to Full Pico Emulation

## Current State: v0.6.0

| Category | Coverage | Notes |
|----------|----------|-------|
| Instructions | ~75% | 65+ Thumb-1; 32-bit: BL, MSR, MRS, DSB/DMB/ISB |
| Memory Map | ~55% | Flash + SRAM; shared RAM fixed; no ROM, no aliases |
| Peripherals | ~20% | GPIO, Timer, NVIC+SysTick, UART Tx, SPI/I2C/PWM stubs |
| Exceptions | ~70% | Entry/return, priority preemption, SysTick, PendSV |
| Boot | ~30% | Vector table only; no ROM/boot2 simulation |

---

## Phase 1: Core Correctness (Run simple SDK programs) -- COMPLETE

### 1.1 32-bit Thumb Instructions [COMPLETE]
~~Any unrecognized 32-bit instruction halts the CPU.~~

**Note**: Cortex-M0+ (ARMv6-M) only supports BL, MSR, MRS, and DSB/DMB/ISB as 32-bit
instructions. MOVW/MOVT, LDR.W/STR.W, and B.W are Thumb-2 (ARMv7-M) and are NOT valid
on M0+. The original roadmap incorrectly listed these.

- **MSR** (0xF380 8800) - write PRIMASK, CONTROL, APSR flags, MSP
- **MRS** (0xF3EF 8xxx) - read PRIMASK, CONTROL, xPSR, APSR, IPSR, EPSR, MSP
- **DSB/DMB/ISB** (0xF3BF 8Fxx) - memory barrier NOPs (correct for emulator)
- **BL** (0xF000 + 0xF800) - already implemented in v0.1.0

### 1.2 Fix Shared RAM Overlap [COMPLETE]
~~Core 1 RAM (0x20021000-0x20042000) shadows shared RAM (0x20040000+).~~
- Reordered checks in `mem_read32_dual()` and `mem_write32_dual()`: shared RAM range
  (>= 0x20040000) checked before per-core RAM, so addresses resolve correctly

### 1.3 SysTick Timer [COMPLETE]
~~SDK `sleep_ms()` / `sleep_us()` rely on SysTick. Without it, all delays hang.~~
- Registers: SYST_CSR, SYST_RVR, SYST_CVR, SYST_CALIB fully implemented
- COUNTFLAG (bit 16) set on wrap, cleared on CSR read
- TICKINT (bit 1) pends SysTick exception when counter reaches zero
- Priority-based delivery through normal NVIC path

### 1.4 NVIC Priority Preemption [COMPLETE]
~~Currently no check whether pending IRQ can preempt active exception.~~
- `nvic_get_exception_priority()` returns effective priority for any vector number
- Pending IRQ only delivered if its priority < active exception's priority
- SCB_SHPR2/SHPR3 configure SVCall, PendSV, and SysTick priorities
- 4 priority levels via bits [7:6] (M0+ compliant)

### 1.5 MSR/MRS Implementation [COMPLETE]
~~Currently stubs.~~
- PRIMASK: read/write via MSR/MRS
- CONTROL: read/write bits [1:0] (nPRIV, SPSEL)
- xPSR: APSR (flags in bits [31:28]), IPSR (vector in bits [5:0]), EPSR (T bit)
- MSP: read/write via SYSm 0x08

---

## Phase 2: SDK Boot Path (Run Pico SDK `hello_world`)

### 2.1 Resets Peripheral (0x4000C000) [CRITICAL]
SDK calls `reset_block()` / `unreset_block_wait()` during init for every peripheral.
Without this, SDK init loops forever waiting for peripheral to come out of reset.
- RESET register: write 1 to hold peripheral in reset
- RESET_DONE register: read 1 when peripheral is out of reset
- Simplest impl: RESET_DONE always returns all-ones (everything ready)

### 2.2 Clocks Peripheral (0x40008000) [CRITICAL]
SDK configures clocks before anything else.
- CLK_REF, CLK_SYS, CLK_PERI, CLK_USB, CLK_ADC, CLK_RTC
- Each has CTRL, DIV, SELECTED registers
- Stub: SELECTED returns non-zero (clock source selected), DIV returns 1:0

### 2.3 XOSC (0x40024000) + PLL_SYS (0x40028000) [HIGH]
SDK enables crystal oscillator, then configures PLL for 125MHz.
- XOSC STATUS: return STABLE=1 (bit 31)
- PLL: CS register return LOCK=1 (bit 31)

### 2.4 Watchdog (0x40058000) [MEDIUM]
SDK configures watchdog tick for SysTick reference clock.
- TICK register: stores tick divider
- CTRL: enable/disable
- Stub: accept writes, no actual watchdog reset

### 2.5 ROM Function Table [MEDIUM]
SDK calls ROM utility functions (memcpy, popcount, etc.) via table at 0x00000018.
- Minimal ROM: provide function table pointer and stub implementations
- Key functions: `rom_func_lookup()`, `_memcpy4`, `_memset4`

---

## Phase 3: Peripheral Emulation (Run real applications)

### 3.1 UART Full (0x40034000 / 0x40038000) [HIGH]
- Rx data register: return buffered input or empty flag
- FIFO status: TXFF, RXFE, busy flags
- Baud rate, line control registers (accept writes)

### 3.2 SPI Full (0x4003C000 / 0x40040000) [MEDIUM]
- Data register read/write with simple FIFO
- Status register already returns idle
- Control registers for frame format

### 3.3 I2C Full (0x40044000 / 0x40048000) [MEDIUM]
- Target address register, data register
- Status: TX empty, RX full, busy
- Simple transaction completion

### 3.4 PWM (0x40050000) [MEDIUM]
- 8 PWM slices, each with counter, CC, TOP
- Interrupt on wrap
- No actual waveform output needed

### 3.5 DMA (0x50000000) [MEDIUM]
- 12 channels with source, dest, count, control
- Auto-copy between memory regions on trigger
- Interrupt on completion

### 3.6 ADC (0x4004C000) [LOW]
- Return configurable analog values
- FIFO support
- Temperature sensor channel

### 3.7 PIO (0x50200000 / 0x50300000) [LOW]
- State machine instruction execution
- Most complex peripheral - defer unless needed

### 3.8 USB (0x50110000) [LOW]
- Device mode endpoint handling
- Very complex - defer unless needed

---

## Phase 4: Advanced Features (Full compatibility)

### 4.1 SRAM Aliasing
- Map 0x21000000-0x25FFFFFF to 0x20000000 with XOR/SET/CLR semantics

### 4.2 XIP Cache Control (0x14000000)
- Cache flush, invalidate registers

### 4.3 ROM Bootloader
- Full boot2 simulation or skip-to-application shortcut

### 4.4 Cycle-Accurate Timing
- Configurable cycles-per-microsecond ratio
- Instruction timing table

### 4.5 GDB Remote Stub
- TCP server implementing GDB RSP
- Hardware breakpoints, single-step, register read/write

---

## Priority Matrix

| Task | Blocks SDK hello_world? | Effort | Impact | Status |
|------|------------------------|--------|--------|--------|
| ~~32-bit instructions (MSR/MRS/barriers)~~ | ~~YES~~ | ~~Medium~~ | ~~Critical~~ | DONE v0.6.0 |
| Resets peripheral stub | YES | Small | Critical | Phase 2 |
| Clocks peripheral stub | YES | Small | Critical | Phase 2 |
| XOSC/PLL stubs | YES | Small | Critical | Phase 2 |
| ~~SysTick timer~~ | ~~YES (sleep_ms)~~ | ~~Medium~~ | ~~Critical~~ | DONE v0.6.0 |
| ~~Shared RAM fix~~ | ~~No~~ | ~~Small~~ | ~~High~~ | DONE v0.6.0 |
| ~~NVIC preemption~~ | ~~No~~ | ~~Medium~~ | ~~High~~ | DONE v0.6.0 |
| ~~MSR/MRS full impl~~ | ~~Maybe~~ | ~~Small~~ | ~~Medium~~ | DONE v0.6.0 |
| Watchdog stub | Maybe | Small | Medium | Phase 2 |
| UART Rx | No | Medium | Medium | Phase 3 |
| DMA stub | Some programs | Medium | Medium | Phase 3 |
| ROM function table | Some programs | Medium | Medium | Phase 2 |
