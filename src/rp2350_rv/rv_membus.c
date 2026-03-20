/*
 * RP2350 Memory Bus for RISC-V Hazard3
 *
 * Routes memory accesses for the RISC-V execution path.
 * RP2350-specific regions (520KB SRAM, CLINT, RP2350 peripherals, SIO)
 * are handled here. Shared peripherals fall through to the RP2040 membus.
 */

#include <string.h>
#include <stdio.h>
#include "rp2350_rv/rv_membus.h"
#include "rp2350_rv/rp2350_memmap.h"
#include "emulator.h"

/* ========================================================================
 * Initialization
 * ======================================================================== */

void rv_membus_init(rv_membus_state_t *bus, uint8_t *flash, uint32_t flash_size,
                    uint32_t cycles_per_us) {
    memset(bus->sram, 0, RV_SRAM_SIZE);
    memset(bus->rom, 0, sizeof(bus->rom));
    bus->flash = flash;
    bus->flash_size = flash_size;
    bus->rom_size = 32 * 1024;
    bus->is_riscv = 1;
    bus->hart1_launch_pending = 0;
    bus->gpio_hi_in = 0x3E;  /* CS high + data pulled up (same as RP2040 QSPI) */
    rv_clint_init(&bus->clint, cycles_per_us);
    rp2350_periph_init(&bus->periph);
}

/* ========================================================================
 * Hart 1 Launch Check
 * ======================================================================== */

int rv_membus_check_hart1_launch(rv_membus_state_t *bus, uint32_t *entry,
                                  uint32_t *sp, uint32_t *arg) {
    if (!bus->hart1_launch_pending) return 0;
    bus->hart1_launch_pending = 0;
    *entry = bus->hart1_entry;
    *sp = bus->hart1_sp;
    *arg = bus->hart1_arg;
    return 1;
}

/* ========================================================================
 * RP2350 SIO Handler (0xD0000000)
 * Different from RP2040 SIO: CPUID returns RP2350, hart launch mailbox,
 * GPIO_HI for pins 32-47, additional spinlocks (0-31 instead of 0-31).
 * ======================================================================== */

static uint32_t rv_sio_read(rv_membus_state_t *bus, uint32_t offset) {
    switch (offset) {
    case RV_SIO_CPUID:
        /* RP2350 CPUID: differs from RP2040 */
        return 0x00000002;  /* RP2350 identifier */

    /* GPIO low (pins 0-31) — fall through to RP2040 */
    case 0x04: /* GPIO_IN */
    case 0x08: /* GPIO_HI_IN */
    case 0x10: /* GPIO_OUT */
    case 0x20: /* GPIO_OE */
        break;  /* Will fall through */

    /* GPIO high (pins 32-47) — RP2350-specific */
    case 0x30: return bus->gpio_hi_out;   /* GPIO_HI_OUT */
    case 0x40: return bus->gpio_hi_oe;    /* GPIO_HI_OE */
    case 0x38: return bus->gpio_hi_in;    /* GPIO_HI_IN (same offset as RP2040 QSPI in) */

    /* Hart 1 launch mailbox */
    case RV_SIO_HART1_BOOT_ENTRY:  return bus->hart1_entry;
    case RV_SIO_HART1_BOOT_SP:     return bus->hart1_sp;
    case RV_SIO_HART1_BOOT_ARG:    return bus->hart1_arg;
    case RV_SIO_HART1_BOOT_LAUNCH: return 0;

    default: break;
    }
    /* Fall through for standard SIO registers */
    return 0xDEAD0000 | offset;  /* Marker for unhandled — will be overridden by fallthrough */
}

static int rv_sio_write(rv_membus_state_t *bus, uint32_t offset, uint32_t val) {
    switch (offset) {
    /* GPIO high (pins 32-47) */
    case 0x30: bus->gpio_hi_out = val & 0xFFFF; return 1;  /* GPIO_HI_OUT */
    case 0x34: bus->gpio_hi_out |= (val & 0xFFFF); return 1; /* GPIO_HI_OUT_SET */
    case 0x38: bus->gpio_hi_out &= ~(val & 0xFFFF); return 1; /* GPIO_HI_OUT_CLR */
    case 0x3C: bus->gpio_hi_out ^= (val & 0xFFFF); return 1; /* GPIO_HI_OUT_XOR */
    case 0x40: bus->gpio_hi_oe = val & 0xFFFF; return 1;    /* GPIO_HI_OE */
    case 0x44: bus->gpio_hi_oe |= (val & 0xFFFF); return 1; /* GPIO_HI_OE_SET */
    case 0x48: bus->gpio_hi_oe &= ~(val & 0xFFFF); return 1; /* GPIO_HI_OE_CLR */
    case 0x4C: bus->gpio_hi_oe ^= (val & 0xFFFF); return 1; /* GPIO_HI_OE_XOR */

    /* Hart 1 launch mailbox */
    case RV_SIO_HART1_BOOT_ENTRY: bus->hart1_entry = val; return 1;
    case RV_SIO_HART1_BOOT_SP:    bus->hart1_sp = val; return 1;
    case RV_SIO_HART1_BOOT_ARG:   bus->hart1_arg = val; return 1;
    case RV_SIO_HART1_BOOT_LAUNCH:
        if (val & 1) {
            bus->hart1_launch_pending = 1;
            fprintf(stderr, "[RV-SIO] Hart 1 launch: entry=0x%08X SP=0x%08X arg=0x%08X\n",
                    bus->hart1_entry, bus->hart1_sp, bus->hart1_arg);
        }
        return 1;

    default: break;
    }
    return 0;  /* Not handled — fall through to RP2040 SIO */
}

/* ========================================================================
 * 32-bit Access
 * ======================================================================== */

uint32_t rv_mem_read32(rv_membus_state_t *bus, uint32_t addr) {
    uint32_t val;

    /* SRAM: 0x20000000 - 0x20082000 (520KB) */
    if (addr >= RP2350_SRAM_BASE && addr < RP2350_SRAM_END) {
        memcpy(&val, &bus->sram[addr - RP2350_SRAM_BASE], 4);
        return val;
    }

    /* SRAM alias: 0x21000000 */
    if (addr >= RP2350_SRAM_ALIAS_BASE && addr < RP2350_SRAM_ALIAS_BASE + RV_SRAM_SIZE) {
        memcpy(&val, &bus->sram[addr - RP2350_SRAM_ALIAS_BASE], 4);
        return val;
    }

    /* ROM: 0x00000000 - 0x00007FFF (32KB) */
    if (addr < bus->rom_size) {
        memcpy(&val, &bus->rom[addr], 4);
        return val;
    }

    /* Flash: 0x10000000+ */
    if (addr >= RP2350_FLASH_BASE && addr < RP2350_FLASH_BASE + bus->flash_size) {
        memcpy(&val, &bus->flash[addr - RP2350_FLASH_BASE], 4);
        return val;
    }

    /* XIP aliases */
    if (addr >= RP2350_XIP_NOALLOC_BASE && addr < RP2350_XIP_NOALLOC_BASE + bus->flash_size) {
        memcpy(&val, &bus->flash[addr - RP2350_XIP_NOALLOC_BASE], 4);
        return val;
    }
    if (addr >= RP2350_XIP_NOCACHE_BASE && addr < RP2350_XIP_NOCACHE_BASE + bus->flash_size) {
        memcpy(&val, &bus->flash[addr - RP2350_XIP_NOCACHE_BASE], 4);
        return val;
    }
    if (addr >= RP2350_XIP_NOCACHE_NOALLOC && addr < RP2350_XIP_NOCACHE_NOALLOC + bus->flash_size) {
        memcpy(&val, &bus->flash[addr - RP2350_XIP_NOCACHE_NOALLOC], 4);
        return val;
    }

    /* CLINT registers (in SIO space) */
    if (rv_clint_match(addr))
        return rv_clint_read(&bus->clint, addr - RV_CLINT_BASE);

    /* RP2350-specific peripherals */
    if (rp2350_periph_match(addr))
        return rp2350_periph_read32(&bus->periph, addr);

    /* SIO: handle RP2350-specific registers, fall through for standard */
    if (addr >= RP2350_SIO_BASE && addr < RP2350_SIO_BASE + 0x200) {
        uint32_t offset = addr - RP2350_SIO_BASE;
        uint32_t sio_val = rv_sio_read(bus, offset);
        /* If CPUID or GPIO_HI or hart launch, return directly */
        if (offset == RV_SIO_CPUID || offset == 0x30 || offset == 0x38 ||
            offset == 0x40 ||
            (offset >= RV_SIO_HART1_BOOT_ENTRY && offset <= RV_SIO_HART1_BOOT_LAUNCH))
            return sio_val;
        /* Otherwise fall through to RP2040 SIO */
    }

    /* Fall through to shared RP2040 peripheral bus */
    return mem_read32(addr);
}

void rv_mem_write32(rv_membus_state_t *bus, uint32_t addr, uint32_t val) {
    /* SRAM */
    if (addr >= RP2350_SRAM_BASE && addr < RP2350_SRAM_END) {
        memcpy(&bus->sram[addr - RP2350_SRAM_BASE], &val, 4);
        return;
    }
    if (addr >= RP2350_SRAM_ALIAS_BASE && addr < RP2350_SRAM_ALIAS_BASE + RV_SRAM_SIZE) {
        memcpy(&bus->sram[addr - RP2350_SRAM_ALIAS_BASE], &val, 4);
        return;
    }

    /* ROM and flash are read-only */
    if (addr < bus->rom_size) return;
    if (addr >= RP2350_FLASH_BASE && addr < RP2350_FLASH_BASE + bus->flash_size) return;
    if (addr >= RP2350_XIP_NOALLOC_BASE && addr < RP2350_XIP_NOALLOC_BASE + bus->flash_size) return;
    if (addr >= RP2350_XIP_NOCACHE_BASE && addr < RP2350_XIP_NOCACHE_BASE + bus->flash_size) return;
    if (addr >= RP2350_XIP_NOCACHE_NOALLOC && addr < RP2350_XIP_NOCACHE_NOALLOC + bus->flash_size) return;

    /* CLINT */
    if (rv_clint_match(addr)) {
        rv_clint_write(&bus->clint, addr - RV_CLINT_BASE, val);
        return;
    }

    /* RP2350-specific peripherals */
    if (rp2350_periph_match(addr)) {
        rp2350_periph_write32(&bus->periph, addr, val);
        return;
    }

    /* SIO: try RP2350-specific first */
    if (addr >= RP2350_SIO_BASE && addr < RP2350_SIO_BASE + 0x200) {
        uint32_t offset = addr - RP2350_SIO_BASE;
        if (rv_sio_write(bus, offset, val))
            return;
        /* Fall through to RP2040 SIO */
    }

    /* Fall through to shared peripheral bus */
    mem_write32(addr, val);
}

/* ========================================================================
 * 16-bit Access
 * ======================================================================== */

uint16_t rv_mem_read16(rv_membus_state_t *bus, uint32_t addr) {
    uint16_t val;
    if (addr >= RP2350_SRAM_BASE && addr < RP2350_SRAM_END) {
        memcpy(&val, &bus->sram[addr - RP2350_SRAM_BASE], 2);
        return val;
    }
    if (addr >= RP2350_SRAM_ALIAS_BASE && addr < RP2350_SRAM_ALIAS_BASE + RV_SRAM_SIZE) {
        memcpy(&val, &bus->sram[addr - RP2350_SRAM_ALIAS_BASE], 2);
        return val;
    }
    if (addr < bus->rom_size) {
        memcpy(&val, &bus->rom[addr], 2);
        return val;
    }
    if (addr >= RP2350_FLASH_BASE && addr < RP2350_FLASH_BASE + bus->flash_size) {
        memcpy(&val, &bus->flash[addr - RP2350_FLASH_BASE], 2);
        return val;
    }
    if (addr >= RP2350_XIP_NOALLOC_BASE && addr < RP2350_XIP_NOALLOC_BASE + bus->flash_size) {
        memcpy(&val, &bus->flash[addr - RP2350_XIP_NOALLOC_BASE], 2);
        return val;
    }
    if (addr >= RP2350_XIP_NOCACHE_BASE && addr < RP2350_XIP_NOCACHE_BASE + bus->flash_size) {
        memcpy(&val, &bus->flash[addr - RP2350_XIP_NOCACHE_BASE], 2);
        return val;
    }
    if (addr >= RP2350_XIP_NOCACHE_NOALLOC && addr < RP2350_XIP_NOCACHE_NOALLOC + bus->flash_size) {
        memcpy(&val, &bus->flash[addr - RP2350_XIP_NOCACHE_NOALLOC], 2);
        return val;
    }
    return mem_read16(addr);
}

void rv_mem_write16(rv_membus_state_t *bus, uint32_t addr, uint16_t val) {
    if (addr >= RP2350_SRAM_BASE && addr < RP2350_SRAM_END) {
        memcpy(&bus->sram[addr - RP2350_SRAM_BASE], &val, 2);
        return;
    }
    if (addr >= RP2350_SRAM_ALIAS_BASE && addr < RP2350_SRAM_ALIAS_BASE + RV_SRAM_SIZE) {
        memcpy(&bus->sram[addr - RP2350_SRAM_ALIAS_BASE], &val, 2);
        return;
    }
    if (addr < bus->rom_size) return;
    if (addr >= RP2350_FLASH_BASE && addr < RP2350_FLASH_BASE + bus->flash_size) return;
    mem_write16(addr, val);
}

/* ========================================================================
 * 8-bit Access
 * ======================================================================== */

uint8_t rv_mem_read8(rv_membus_state_t *bus, uint32_t addr) {
    if (addr >= RP2350_SRAM_BASE && addr < RP2350_SRAM_END)
        return bus->sram[addr - RP2350_SRAM_BASE];
    if (addr >= RP2350_SRAM_ALIAS_BASE && addr < RP2350_SRAM_ALIAS_BASE + RV_SRAM_SIZE)
        return bus->sram[addr - RP2350_SRAM_ALIAS_BASE];
    if (addr < bus->rom_size)
        return bus->rom[addr];
    if (addr >= RP2350_FLASH_BASE && addr < RP2350_FLASH_BASE + bus->flash_size)
        return bus->flash[addr - RP2350_FLASH_BASE];
    if (addr >= RP2350_XIP_NOALLOC_BASE && addr < RP2350_XIP_NOALLOC_BASE + bus->flash_size)
        return bus->flash[addr - RP2350_XIP_NOALLOC_BASE];
    if (addr >= RP2350_XIP_NOCACHE_BASE && addr < RP2350_XIP_NOCACHE_BASE + bus->flash_size)
        return bus->flash[addr - RP2350_XIP_NOCACHE_BASE];
    if (addr >= RP2350_XIP_NOCACHE_NOALLOC && addr < RP2350_XIP_NOCACHE_NOALLOC + bus->flash_size)
        return bus->flash[addr - RP2350_XIP_NOCACHE_NOALLOC];
    /* RP2350 peripherals byte access */
    if (rp2350_periph_match(addr))
        return rp2350_periph_read8(&bus->periph, addr);
    return mem_read8(addr);
}

void rv_mem_write8(rv_membus_state_t *bus, uint32_t addr, uint8_t val) {
    if (addr >= RP2350_SRAM_BASE && addr < RP2350_SRAM_END) {
        bus->sram[addr - RP2350_SRAM_BASE] = val;
        return;
    }
    if (addr >= RP2350_SRAM_ALIAS_BASE && addr < RP2350_SRAM_ALIAS_BASE + RV_SRAM_SIZE) {
        bus->sram[addr - RP2350_SRAM_ALIAS_BASE] = val;
        return;
    }
    if (addr < bus->rom_size) return;
    if (addr >= RP2350_FLASH_BASE && addr < RP2350_FLASH_BASE + bus->flash_size) return;
    /* RP2350 peripherals byte access */
    if (rp2350_periph_match(addr)) {
        rp2350_periph_write8(&bus->periph, addr, val);
        return;
    }
    mem_write8(addr, val);
}
