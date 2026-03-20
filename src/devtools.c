/*
 * Developer Tools for Bramble RP2040 Emulator
 *
 * Implementation of semihosting, coverage, hotspots, trace,
 * SYSCFG, and TBMAN peripherals.
 */

#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include "emulator.h"
#include "devtools.h"

/* ========================================================================
 * ARM Semihosting
 * ======================================================================== */

int semihosting_enabled = 0;
int semihost_exit_requested = 0;
int semihost_exit_code = 0;

void semihosting_init(void) {
    semihosting_enabled = 1;
    semihost_exit_requested = 0;
    semihost_exit_code = 0;
}

/* Read a uint32 from guest memory at addr */
static uint32_t sh_read32(uint32_t addr) {
    return mem_read32(addr);
}

/* Read a byte from guest memory */
static uint8_t sh_read8(uint32_t addr) {
    return mem_read8(addr);
}

int semihosting_handle(void) {
    if (!semihosting_enabled) return 0;

    uint32_t op = cpu.r[0];
    uint32_t param = cpu.r[1];

    switch (op) {
    case SEMIHOST_SYS_WRITEC: {
        /* R1 points to a single character */
        char c = (char)sh_read8(param);
        fputc(c, stdout);
        fflush(stdout);
        cpu.r[0] = 0;
        break;
    }
    case SEMIHOST_SYS_WRITE0: {
        /* R1 points to a null-terminated string */
        for (int i = 0; i < 4096; i++) {
            char c = (char)sh_read8(param + (uint32_t)i);
            if (c == '\0') break;
            fputc(c, stdout);
        }
        fflush(stdout);
        cpu.r[0] = 0;
        break;
    }
    case SEMIHOST_SYS_WRITE: {
        /* param[0]=fd, param[1]=data_ptr, param[2]=len */
        uint32_t fd   = sh_read32(param);
        uint32_t data = sh_read32(param + 4);
        uint32_t len  = sh_read32(param + 8);
        FILE *out = (fd == 1) ? stdout : (fd == 2) ? stderr : stdout;
        for (uint32_t i = 0; i < len && i < 65536; i++) {
            fputc(sh_read8(data + i), out);
        }
        fflush(out);
        cpu.r[0] = 0;  /* 0 = all bytes written */
        break;
    }
    case SEMIHOST_SYS_READC: {
        int c = fgetc(stdin);
        cpu.r[0] = (c == EOF) ? (uint32_t)-1 : (uint32_t)c;
        break;
    }
    case SEMIHOST_SYS_EXIT: {
        /* ADP_Stopped_ApplicationExit = 0x20026 */
        semihost_exit_code = (param == 0x20026) ? 0 : (int)param;
        semihost_exit_requested = 1;
        cpu.r[15] = 0xFFFFFFFF;  /* Halt the core */
        break;
    }
    case SEMIHOST_SYS_EXIT_EXTENDED: {
        /* param[0] = reason, param[1] = exit code */
        semihost_exit_code = (int)sh_read32(param + 4);
        semihost_exit_requested = 1;
        cpu.r[15] = 0xFFFFFFFF;
        break;
    }
    case SEMIHOST_SYS_ERRNO:
        cpu.r[0] = 0;
        break;
    case SEMIHOST_SYS_ELAPSED:
        /* Return emulated time in centiseconds */
        cpu.r[0] = 0;
        break;
    case SEMIHOST_SYS_TICKFREQ:
        cpu.r[0] = 100;  /* 100 Hz */
        break;
    case SEMIHOST_SYS_OPEN:
    case SEMIHOST_SYS_CLOSE:
    case SEMIHOST_SYS_READ:
    case SEMIHOST_SYS_SEEK:
    case SEMIHOST_SYS_FLEN:
        /* Unimplemented file ops: return -1 */
        cpu.r[0] = (uint32_t)-1;
        break;
    default:
        /* Unknown operation */
        cpu.r[0] = (uint32_t)-1;
        break;
    }

    /* Advance PC past the BKPT instruction */
    cpu.r[15] += 2;
    return 1;
}

/* ========================================================================
 * Code Coverage
 * ======================================================================== */

int coverage_enabled = 0;
uint8_t *coverage_bitmap = NULL;
uint8_t *coverage_ram_bitmap = NULL;

void coverage_init(void) {
    coverage_bitmap = calloc(1, COVERAGE_BITMAP_BYTES);
    coverage_ram_bitmap = calloc(1, (RAM_SIZE / 2 + 7) / 8);
    if (coverage_bitmap && coverage_ram_bitmap) {
        coverage_enabled = 1;
    }
}

void coverage_cleanup(void) {
    free(coverage_bitmap);
    free(coverage_ram_bitmap);
    coverage_bitmap = NULL;
    coverage_ram_bitmap = NULL;
    coverage_enabled = 0;
}

void coverage_dump(const char *path) {
    if (!coverage_bitmap) return;
    FILE *f = fopen(path, "wb");
    if (!f) {
        fprintf(stderr, "[Coverage] Failed to open %s for writing\n", path);
        return;
    }
    /* Header: magic, flash_base, flash_size, ram_base, ram_size */
    uint32_t hdr[5] = { 0x434F5642, FLASH_BASE, FLASH_SIZE, RAM_BASE, RAM_SIZE }; /* "BVOC" */
    fwrite(hdr, 4, 5, f);
    fwrite(coverage_bitmap, 1, COVERAGE_BITMAP_BYTES, f);
    fwrite(coverage_ram_bitmap, 1, (RAM_SIZE / 2 + 7) / 8, f);
    fclose(f);
    fprintf(stderr, "[Coverage] Written to %s\n", path);
}

void coverage_report(void) {
    if (!coverage_bitmap) return;
    uint32_t flash_covered = 0;
    for (uint32_t i = 0; i < COVERAGE_BITMAP_BYTES; i++) {
        flash_covered += (uint32_t)__builtin_popcount(coverage_bitmap[i]);
    }
    uint32_t flash_total = FLASH_SIZE / 2;
    uint32_t ram_covered = 0;
    uint32_t ram_bytes = (RAM_SIZE / 2 + 7) / 8;
    for (uint32_t i = 0; i < ram_bytes; i++) {
        ram_covered += (uint32_t)__builtin_popcount(coverage_ram_bitmap[i]);
    }
    fprintf(stderr, " Coverage: %u/%u flash halfwords (%.1f%%), %u RAM halfwords executed\n",
            flash_covered, flash_total,
            flash_total > 0 ? (double)flash_covered / flash_total * 100.0 : 0.0,
            ram_covered);
}

/* ========================================================================
 * Hotspot Profiling
 * ======================================================================== */

int hotspots_enabled = 0;
int hotspots_top_n = 20;
hotspot_entry_t *hotspot_map = NULL;

void hotspots_init(void) {
    hotspot_map = calloc(HOTSPOT_MAP_SIZE, sizeof(hotspot_entry_t));
    if (hotspot_map) {
        hotspots_enabled = 1;
    }
}

void hotspots_cleanup(void) {
    free(hotspot_map);
    hotspot_map = NULL;
    hotspots_enabled = 0;
}

/* Comparison for qsort (descending by count) */
static int hotspot_cmp(const void *a, const void *b) {
    const hotspot_entry_t *ha = a, *hb = b;
    if (hb->count > ha->count) return 1;
    if (hb->count < ha->count) return -1;
    return 0;
}

void hotspots_report(void) {
    if (!hotspot_map) return;

    /* Collect non-zero entries */
    int count = 0;
    for (int i = 0; i < HOTSPOT_MAP_SIZE; i++) {
        if (hotspot_map[i].count > 0) count++;
    }
    if (count == 0) return;

    /* Copy to sortable array */
    int n = count < hotspots_top_n ? count : hotspots_top_n;
    hotspot_entry_t *sorted = malloc((size_t)count * sizeof(hotspot_entry_t));
    if (!sorted) return;

    int si = 0;
    for (int i = 0; i < HOTSPOT_MAP_SIZE; i++) {
        if (hotspot_map[i].count > 0) {
            sorted[si++] = hotspot_map[i];
        }
    }
    qsort(sorted, (size_t)count, sizeof(hotspot_entry_t), hotspot_cmp);

    fprintf(stderr, " Hotspots (top %d of %d unique PCs):\n", n, count);
    fprintf(stderr, "   %-12s  %-12s  %s\n", "Address", "Count", "Region");
    for (int i = 0; i < n; i++) {
        const char *region = "???";
        uint32_t pc = sorted[i].pc;
        if (pc >= FLASH_BASE && pc < FLASH_BASE + FLASH_SIZE)
            region = "flash";
        else if (pc >= RAM_BASE && pc < RAM_BASE + RAM_SIZE)
            region = "ram";
        else if (pc < 0x4000)
            region = "rom";
        fprintf(stderr, "   0x%08X  %-12u  %s\n", pc, sorted[i].count, region);
    }
    free(sorted);
}

/* ========================================================================
 * Instruction Trace
 * ======================================================================== */

int trace_enabled = 0;
FILE *trace_file = NULL;

void trace_init(const char *path) {
    trace_file = fopen(path, "wb");
    if (trace_file) {
        trace_enabled = 1;
        /* Write header: magic + version */
        uint32_t hdr[2] = { 0x54524342, 1 }; /* "BCRT" v1 */
        fwrite(hdr, 4, 2, trace_file);
        fprintf(stderr, "[Trace] Writing instruction trace to %s\n", path);
    } else {
        fprintf(stderr, "[Trace] Failed to open %s\n", path);
    }
}

void trace_cleanup(void) {
    if (trace_file) {
        fclose(trace_file);
        trace_file = NULL;
    }
    trace_enabled = 0;
}

/* ========================================================================
 * Exit Code Hook
 * ======================================================================== */

int exit_code_enabled = 0;
uint32_t exit_code_addr = 0;

/* ========================================================================
 * Timeout Enforcement
 * ======================================================================== */

int timeout_seconds = 0;
volatile int timeout_expired = 0;

static void timeout_handler(int sig) {
    (void)sig;
    timeout_expired = 1;
}

void timeout_start(int seconds) {
    timeout_seconds = seconds;
    timeout_expired = 0;
    signal(SIGALRM, timeout_handler);
    alarm((unsigned)seconds);
    fprintf(stderr, "[Init] Timeout: %d seconds\n", seconds);
}

void timeout_cancel(void) {
    alarm(0);
    signal(SIGALRM, SIG_DFL);
}

/* ========================================================================
 * SYSCFG Peripheral (0x40004000)
 *
 * RP2040 datasheet section 2.22. Registers:
 *   0x00 PROC0_NMI_MASK      R/W  NMI mask for proc0
 *   0x04 PROC1_NMI_MASK      R/W  NMI mask for proc1
 *   0x08 PROC_CONFIG          RO   Processor configuration
 *   0x0C PROC_IN_SYNC_BYPASS  R/W  Input sync bypass
 *   0x10 PROC_IN_SYNC_BYPASS_HI R/W
 *   0x14 DBGFORCE             R/W  Debug force
 *   0x18 MEMPOWERDOWN         R/W  Memory power-down control
 * ======================================================================== */

static uint32_t syscfg_regs[SYSCFG_SIZE / 4];

int syscfg_match(uint32_t addr) {
    uint32_t base = addr & ~0x3000u;
    return (base >= SYSCFG_BASE && base < SYSCFG_BASE + SYSCFG_SIZE);
}

uint32_t syscfg_read(uint32_t offset) {
    offset &= 0xFFF;
    if (offset >= SYSCFG_SIZE) return 0;
    if (offset == 0x08) {
        /* PROC_CONFIG: both processors out of reset */
        return 0x00000000;
    }
    return syscfg_regs[offset / 4];
}

void syscfg_write(uint32_t offset, uint32_t val) {
    offset &= 0xFFF;
    if (offset >= SYSCFG_SIZE) return;
    if (offset == 0x08) return; /* PROC_CONFIG is read-only */
    syscfg_regs[offset / 4] = val;
}

/* ========================================================================
 * TBMAN Peripheral (0x4006C000)
 *
 * RP2040 datasheet section 2.21. Registers:
 *   0x00 PLATFORM   RO  Bit 0 = ASIC, Bit 1 = FPGA
 *
 * In a real chip, PLATFORM reads 0x01 (ASIC). In simulation/emulation
 * it should read 0x02 (FPGA/sim) or a custom value.  We return 0x01
 * to match real hardware behavior (firmware expects ASIC).
 * ======================================================================== */

int tbman_match(uint32_t addr) {
    uint32_t base = addr & ~0x3000u;
    return (base >= TBMAN_BASE && base < TBMAN_BASE + TBMAN_SIZE);
}

uint32_t tbman_read(uint32_t offset) {
    offset &= 0xFFF;
    if (offset == 0x00) {
        return 0x01;  /* ASIC platform */
    }
    return 0;
}

void tbman_write(uint32_t offset, uint32_t val) {
    (void)offset;
    (void)val;
    /* All TBMAN registers are read-only */
}
