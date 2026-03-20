/*
 * Hazard3 RISC-V CPU Engine — RV32I Base Integer
 *
 * Phase 1: Complete RV32I (37 instructions)
 *   - R-type: ADD, SUB, SLL, SLT, SLTU, XOR, SRL, SRA, OR, AND
 *   - I-type: ADDI, SLTI, SLTIU, XORI, ORI, ANDI, SLLI, SRLI, SRAI
 *   - I-type loads: LB, LH, LW, LBU, LHU
 *   - S-type stores: SB, SH, SW
 *   - B-type branches: BEQ, BNE, BLT, BGE, BLTU, BGEU
 *   - U-type: LUI, AUIPC
 *   - J-type: JAL, JALR
 *   - System: ECALL, EBREAK, FENCE
 *   - CSR: CSRRW, CSRRS, CSRRC, CSRRWI, CSRRSI, CSRRCI
 */

#include <stdio.h>
#include <string.h>
#include "rp2350_rv/rv_cpu.h"
#include "emulator.h"

/* ========================================================================
 * RV32I Opcode Map (bits [6:0])
 * ======================================================================== */

#define OP_LUI      0x37
#define OP_AUIPC    0x17
#define OP_JAL      0x6F
#define OP_JALR     0x67
#define OP_BRANCH   0x63
#define OP_LOAD     0x03
#define OP_STORE    0x23
#define OP_IMM      0x13
#define OP_REG      0x33
#define OP_FENCE    0x0F
#define OP_SYSTEM   0x73

/* ========================================================================
 * Initialization
 * ======================================================================== */

void rv_cpu_init(rv_cpu_state_t *cpu, int hart_id) {
    memset(cpu, 0, sizeof(*cpu));
    cpu->hart_id = hart_id;
    cpu->is_halted = 1;  /* Start halted until reset */

    /* Initialize misa: RV32IMAC */
    cpu->csr[CSR_MISA] = (1u << 30)   /* MXL=1 (32-bit) */
                        | (1u << 0)    /* A - Atomics */
                        | (1u << 2)    /* C - Compressed */
                        | (1u << 8)    /* I - Base integer */
                        | (1u << 12);  /* M - Multiply/divide */

    /* Hazard3 vendor/arch/impl IDs */
    cpu->csr[CSR_MVENDORID] = 0;       /* Non-commercial */
    cpu->csr[CSR_MARCHID]   = 0;
    cpu->csr[CSR_MIMPID]    = 0;
    cpu->csr[CSR_MHARTID]   = (uint32_t)hart_id;

    /* mstatus: machine mode, interrupts disabled */
    cpu->csr[CSR_MSTATUS] = MSTATUS_MPP;  /* MPP = M-mode */
}

void rv_cpu_reset(rv_cpu_state_t *cpu, uint32_t entry_pc) {
    cpu->pc = entry_pc;
    cpu->is_halted = 0;
    cpu->is_wfi = 0;
    cpu->step_count = 0;
    cpu->cycle_count = 0;
    cpu->instret_count = 0;
    memset(cpu->x, 0, sizeof(cpu->x));

    if (cpu->debug_enabled)
        fprintf(stderr, "[RV-CORE%d] Reset to PC=0x%08X\n", cpu->hart_id, entry_pc);
}

int rv_cpu_is_halted(rv_cpu_state_t *cpu) {
    return cpu->is_halted;
}

/* ========================================================================
 * CSR Access
 * ======================================================================== */

uint32_t rv_csr_read(rv_cpu_state_t *cpu, uint16_t addr) {
    switch (addr) {
    case CSR_MCYCLE:    return (uint32_t)cpu->cycle_count;
    case CSR_MCYCLEH:   return (uint32_t)(cpu->cycle_count >> 32);
    case CSR_MINSTRET:  return (uint32_t)cpu->instret_count;
    case CSR_MINSTRETH: return (uint32_t)(cpu->instret_count >> 32);
    case CSR_MHARTID:   return (uint32_t)cpu->hart_id;
    default:
        if (addr < RV_CSR_COUNT)
            return cpu->csr[addr];
        return 0;
    }
}

void rv_csr_write(rv_cpu_state_t *cpu, uint16_t addr, uint32_t val) {
    switch (addr) {
    case CSR_MCYCLE:    cpu->cycle_count = (cpu->cycle_count & 0xFFFFFFFF00000000ULL) | val; break;
    case CSR_MCYCLEH:   cpu->cycle_count = (cpu->cycle_count & 0xFFFFFFFF) | ((uint64_t)val << 32); break;
    case CSR_MINSTRET:  cpu->instret_count = (cpu->instret_count & 0xFFFFFFFF00000000ULL) | val; break;
    case CSR_MINSTRETH: cpu->instret_count = (cpu->instret_count & 0xFFFFFFFF) | ((uint64_t)val << 32); break;
    case CSR_MHARTID:   break;  /* Read-only */
    case CSR_MISA:      break;  /* Read-only */
    case CSR_MVENDORID: break;  /* Read-only */
    case CSR_MARCHID:   break;  /* Read-only */
    case CSR_MIMPID:    break;  /* Read-only */
    default:
        if (addr < RV_CSR_COUNT)
            cpu->csr[addr] = val;
        break;
    }
}

/* ========================================================================
 * Trap Handling
 * ======================================================================== */

void rv_trap_enter(rv_cpu_state_t *cpu, uint32_t cause, uint32_t tval) {
    /* Save current state */
    cpu->csr[CSR_MEPC] = cpu->pc;
    cpu->csr[CSR_MCAUSE] = cause;
    cpu->csr[CSR_MTVAL] = tval;

    /* Save and clear MIE */
    uint32_t mstatus = cpu->csr[CSR_MSTATUS];
    if (mstatus & MSTATUS_MIE)
        mstatus |= MSTATUS_MPIE;
    else
        mstatus &= ~MSTATUS_MPIE;
    mstatus &= ~MSTATUS_MIE;        /* Disable interrupts */
    mstatus = (mstatus & ~MSTATUS_MPP) | MSTATUS_MPP;  /* MPP = M-mode */
    cpu->csr[CSR_MSTATUS] = mstatus;

    /* Jump to trap handler */
    uint32_t mtvec = cpu->csr[CSR_MTVEC];
    uint32_t mode = mtvec & 3;
    uint32_t base = mtvec & ~3u;

    if (mode == 1 && (cause & MCAUSE_INTERRUPT_BIT)) {
        /* Vectored mode for interrupts */
        cpu->pc = base + (cause & ~MCAUSE_INTERRUPT_BIT) * 4;
    } else {
        /* Direct mode (or exceptions in vectored mode) */
        cpu->pc = base;
    }

    cpu->in_trap = 1;

    if (cpu->debug_enabled)
        fprintf(stderr, "[RV-CORE%d] TRAP: cause=0x%08X mepc=0x%08X -> handler=0x%08X\n",
                cpu->hart_id, cause, cpu->csr[CSR_MEPC], cpu->pc);
}

void rv_trap_return(rv_cpu_state_t *cpu) {
    /* Restore state from mepc */
    cpu->pc = cpu->csr[CSR_MEPC];

    /* Restore MIE from MPIE */
    uint32_t mstatus = cpu->csr[CSR_MSTATUS];
    if (mstatus & MSTATUS_MPIE)
        mstatus |= MSTATUS_MIE;
    else
        mstatus &= ~MSTATUS_MIE;
    mstatus |= MSTATUS_MPIE;   /* Set MPIE */
    cpu->csr[CSR_MSTATUS] = mstatus;

    cpu->in_trap = 0;

    if (cpu->debug_enabled)
        fprintf(stderr, "[RV-CORE%d] MRET -> PC=0x%08X\n", cpu->hart_id, cpu->pc);
}

/* ========================================================================
 * Write-back helper (x0 is hardwired to zero)
 * ======================================================================== */

static inline void rv_write_rd(rv_cpu_state_t *cpu, uint32_t rd, uint32_t val) {
    if (rd != 0) cpu->x[rd] = val;
}

/* ========================================================================
 * RV32I Instruction Execution
 * ======================================================================== */

int rv_cpu_step(rv_cpu_state_t *cpu) {
    if (cpu->is_halted) return -1;

    uint32_t pc = cpu->pc;

    /* Fetch instruction (supports both 32-bit and 16-bit compressed) */
    uint32_t instr;
    uint16_t lo = mem_read16(pc);

    int is_compressed = ((lo & 3) != 3);
    if (is_compressed) {
        /* C extension: 16-bit instruction (Phase 3 — stub for now) */
        if (cpu->debug_enabled)
            fprintf(stderr, "[RV-CORE%d] Compressed instruction at 0x%08X: 0x%04X (not yet implemented)\n",
                    cpu->hart_id, pc, lo);
        rv_trap_enter(cpu, MCAUSE_ILLEGAL_INSTR, lo);
        return 0;
    }

    /* 32-bit instruction */
    uint16_t hi = mem_read16(pc + 2);
    instr = (uint32_t)lo | ((uint32_t)hi << 16);

    uint32_t opcode = RV_OPCODE(instr);
    uint32_t rd     = RV_RD(instr);
    uint32_t funct3 = RV_FUNCT3(instr);
    uint32_t rs1    = RV_RS1(instr);
    uint32_t rs2    = RV_RS2(instr);
    uint32_t funct7 = RV_FUNCT7(instr);

    uint32_t next_pc = pc + 4;

    if (cpu->debug_enabled) {
        fprintf(stderr, "[RV-CORE%d] PC=0x%08X instr=0x%08X op=0x%02X rd=%u rs1=%u rs2=%u f3=%u f7=0x%02X\n",
                cpu->hart_id, pc, instr, opcode, rd, rs1, rs2, funct3, funct7);
    }

    switch (opcode) {

    /* ================================================================
     * LUI: rd = imm_u
     * ================================================================ */
    case OP_LUI:
        rv_write_rd(cpu, rd, (uint32_t)rv_imm_u(instr));
        break;

    /* ================================================================
     * AUIPC: rd = pc + imm_u
     * ================================================================ */
    case OP_AUIPC:
        rv_write_rd(cpu, rd, pc + (uint32_t)rv_imm_u(instr));
        break;

    /* ================================================================
     * JAL: rd = pc + 4; pc = pc + imm_j
     * ================================================================ */
    case OP_JAL:
        rv_write_rd(cpu, rd, pc + 4);
        next_pc = pc + (uint32_t)rv_imm_j(instr);
        if (next_pc & 1) {
            rv_trap_enter(cpu, MCAUSE_INSTR_MISALIGNED, next_pc);
            return 0;
        }
        break;

    /* ================================================================
     * JALR: rd = pc + 4; pc = (rs1 + imm_i) & ~1
     * ================================================================ */
    case OP_JALR:
        if (funct3 != 0) goto illegal;
        {
            uint32_t target = (cpu->x[rs1] + (uint32_t)rv_imm_i(instr)) & ~1u;
            rv_write_rd(cpu, rd, pc + 4);
            next_pc = target;
        }
        break;

    /* ================================================================
     * Branches: BEQ, BNE, BLT, BGE, BLTU, BGEU
     * ================================================================ */
    case OP_BRANCH: {
        int32_t a = (int32_t)cpu->x[rs1];
        int32_t b = (int32_t)cpu->x[rs2];
        uint32_t ua = cpu->x[rs1];
        uint32_t ub = cpu->x[rs2];
        int taken = 0;

        switch (funct3) {
        case 0: taken = (ua == ub); break;  /* BEQ */
        case 1: taken = (ua != ub); break;  /* BNE */
        case 4: taken = (a < b);    break;  /* BLT */
        case 5: taken = (a >= b);   break;  /* BGE */
        case 6: taken = (ua < ub);  break;  /* BLTU */
        case 7: taken = (ua >= ub); break;  /* BGEU */
        default: goto illegal;
        }

        if (taken) {
            next_pc = pc + (uint32_t)rv_imm_b(instr);
            if (next_pc & 1) {
                rv_trap_enter(cpu, MCAUSE_INSTR_MISALIGNED, next_pc);
                return 0;
            }
        }
        break;
    }

    /* ================================================================
     * Loads: LB, LH, LW, LBU, LHU
     * ================================================================ */
    case OP_LOAD: {
        uint32_t addr = cpu->x[rs1] + (uint32_t)rv_imm_i(instr);
        uint32_t val;

        switch (funct3) {
        case 0: /* LB */
            val = (uint32_t)(int32_t)(int8_t)mem_read8(addr);
            break;
        case 1: /* LH */
            if (addr & 1) { rv_trap_enter(cpu, MCAUSE_LOAD_MISALIGNED, addr); return 0; }
            val = (uint32_t)(int32_t)(int16_t)mem_read16(addr);
            break;
        case 2: /* LW */
            if (addr & 3) { rv_trap_enter(cpu, MCAUSE_LOAD_MISALIGNED, addr); return 0; }
            val = mem_read32(addr);
            break;
        case 4: /* LBU */
            val = (uint32_t)mem_read8(addr);
            break;
        case 5: /* LHU */
            if (addr & 1) { rv_trap_enter(cpu, MCAUSE_LOAD_MISALIGNED, addr); return 0; }
            val = (uint32_t)mem_read16(addr);
            break;
        default: goto illegal;
        }

        rv_write_rd(cpu, rd, val);
        break;
    }

    /* ================================================================
     * Stores: SB, SH, SW
     * ================================================================ */
    case OP_STORE: {
        uint32_t addr = cpu->x[rs1] + (uint32_t)rv_imm_s(instr);
        uint32_t val = cpu->x[rs2];

        switch (funct3) {
        case 0: /* SB */
            mem_write8(addr, (uint8_t)val);
            break;
        case 1: /* SH */
            if (addr & 1) { rv_trap_enter(cpu, MCAUSE_STORE_MISALIGNED, addr); return 0; }
            mem_write16(addr, (uint16_t)val);
            break;
        case 2: /* SW */
            if (addr & 3) { rv_trap_enter(cpu, MCAUSE_STORE_MISALIGNED, addr); return 0; }
            mem_write32(addr, val);
            break;
        default: goto illegal;
        }
        break;
    }

    /* ================================================================
     * ALU immediate: ADDI, SLTI, SLTIU, XORI, ORI, ANDI, SLLI, SRLI, SRAI
     * ================================================================ */
    case OP_IMM: {
        int32_t imm = rv_imm_i(instr);
        uint32_t src = cpu->x[rs1];
        uint32_t result;

        switch (funct3) {
        case 0: result = src + (uint32_t)imm; break;                    /* ADDI */
        case 2: result = ((int32_t)src < imm) ? 1 : 0; break;          /* SLTI */
        case 3: result = (src < (uint32_t)imm) ? 1 : 0; break;         /* SLTIU */
        case 4: result = src ^ (uint32_t)imm; break;                    /* XORI */
        case 6: result = src | (uint32_t)imm; break;                    /* ORI */
        case 7: result = src & (uint32_t)imm; break;                    /* ANDI */
        case 1: /* SLLI */
            if (funct7 != 0) goto illegal;
            result = src << (rs2 & 0x1F);
            break;
        case 5: /* SRLI / SRAI */
            if (funct7 == 0x00)
                result = src >> (rs2 & 0x1F);               /* SRLI */
            else if (funct7 == 0x20)
                result = (uint32_t)((int32_t)src >> (rs2 & 0x1F));  /* SRAI */
            else goto illegal;
            break;
        default: goto illegal;
        }

        rv_write_rd(cpu, rd, result);
        break;
    }

    /* ================================================================
     * ALU register: ADD, SUB, SLL, SLT, SLTU, XOR, SRL, SRA, OR, AND
     * (Also M extension: MUL, MULH, MULHSU, MULHU, DIV, DIVU, REM, REMU)
     * ================================================================ */
    case OP_REG: {
        uint32_t a = cpu->x[rs1];
        uint32_t b = cpu->x[rs2];
        uint32_t result;

        if (funct7 == 0x01) {
            /* M extension (Phase 2) */
            switch (funct3) {
            case 0: result = a * b; break;                              /* MUL */
            case 1: result = (uint32_t)((int64_t)(int32_t)a * (int64_t)(int32_t)b >> 32); break; /* MULH */
            case 2: result = (uint32_t)((int64_t)(int32_t)a * (uint64_t)b >> 32); break;  /* MULHSU */
            case 3: result = (uint32_t)((uint64_t)a * (uint64_t)b >> 32); break;          /* MULHU */
            case 4: /* DIV */
                if (b == 0) result = 0xFFFFFFFF;
                else if ((int32_t)a == INT32_MIN && (int32_t)b == -1) result = a;
                else result = (uint32_t)((int32_t)a / (int32_t)b);
                break;
            case 5: /* DIVU */
                result = (b == 0) ? 0xFFFFFFFF : a / b;
                break;
            case 6: /* REM */
                if (b == 0) result = a;
                else if ((int32_t)a == INT32_MIN && (int32_t)b == -1) result = 0;
                else result = (uint32_t)((int32_t)a % (int32_t)b);
                break;
            case 7: /* REMU */
                result = (b == 0) ? a : a % b;
                break;
            default: goto illegal;
            }
        } else if (funct7 == 0x00) {
            switch (funct3) {
            case 0: result = a + b; break;                              /* ADD */
            case 1: result = a << (b & 0x1F); break;                   /* SLL */
            case 2: result = ((int32_t)a < (int32_t)b) ? 1 : 0; break; /* SLT */
            case 3: result = (a < b) ? 1 : 0; break;                   /* SLTU */
            case 4: result = a ^ b; break;                              /* XOR */
            case 5: result = a >> (b & 0x1F); break;                    /* SRL */
            case 6: result = a | b; break;                              /* OR */
            case 7: result = a & b; break;                              /* AND */
            default: goto illegal;
            }
        } else if (funct7 == 0x20) {
            switch (funct3) {
            case 0: result = a - b; break;                              /* SUB */
            case 5: result = (uint32_t)((int32_t)a >> (b & 0x1F)); break; /* SRA */
            default: goto illegal;
            }
        } else {
            goto illegal;
        }

        rv_write_rd(cpu, rd, result);
        break;
    }

    /* ================================================================
     * FENCE: memory ordering (NOP in emulator)
     * ================================================================ */
    case OP_FENCE:
        /* No reordering in emulator — treat as NOP */
        break;

    /* ================================================================
     * SYSTEM: ECALL, EBREAK, MRET, WFI, CSR instructions
     * ================================================================ */
    case OP_SYSTEM:
        if (funct3 == 0) {
            /* PRIV instructions encoded in imm_i */
            uint32_t funct12 = (instr >> 20) & 0xFFF;
            switch (funct12) {
            case 0x000: /* ECALL */
                rv_trap_enter(cpu, MCAUSE_ECALL_M, 0);
                return 0;
            case 0x001: /* EBREAK */
                rv_trap_enter(cpu, MCAUSE_BREAKPOINT, pc);
                return 0;
            case 0x302: /* MRET */
                rv_trap_return(cpu);
                return 0;
            case 0x105: /* WFI */
                cpu->is_wfi = 1;
                break;
            default:
                goto illegal;
            }
        } else {
            /* CSR instructions */
            uint16_t csr_addr = (uint16_t)((instr >> 20) & 0xFFF);
            uint32_t old_val = rv_csr_read(cpu, csr_addr);
            uint32_t new_val;

            switch (funct3) {
            case 1: /* CSRRW */
                new_val = cpu->x[rs1];
                rv_csr_write(cpu, csr_addr, new_val);
                rv_write_rd(cpu, rd, old_val);
                break;
            case 2: /* CSRRS */
                if (rs1 != 0)
                    rv_csr_write(cpu, csr_addr, old_val | cpu->x[rs1]);
                rv_write_rd(cpu, rd, old_val);
                break;
            case 3: /* CSRRC */
                if (rs1 != 0)
                    rv_csr_write(cpu, csr_addr, old_val & ~cpu->x[rs1]);
                rv_write_rd(cpu, rd, old_val);
                break;
            case 5: /* CSRRWI */
                rv_csr_write(cpu, csr_addr, rs1);  /* rs1 field is zimm */
                rv_write_rd(cpu, rd, old_val);
                break;
            case 6: /* CSRRSI */
                if (rs1 != 0)
                    rv_csr_write(cpu, csr_addr, old_val | rs1);
                rv_write_rd(cpu, rd, old_val);
                break;
            case 7: /* CSRRCI */
                if (rs1 != 0)
                    rv_csr_write(cpu, csr_addr, old_val & ~rs1);
                rv_write_rd(cpu, rd, old_val);
                break;
            default:
                goto illegal;
            }
        }
        break;

    default:
        goto illegal;
    }

    /* Advance PC and counters */
    cpu->pc = next_pc;
    cpu->x[0] = 0;  /* x0 always zero */
    cpu->step_count++;
    cpu->cycle_count++;
    cpu->instret_count++;
    return 0;

illegal:
    if (cpu->debug_enabled)
        fprintf(stderr, "[RV-CORE%d] ILLEGAL INSTRUCTION at PC=0x%08X: 0x%08X\n",
                cpu->hart_id, pc, instr);
    rv_trap_enter(cpu, MCAUSE_ILLEGAL_INSTR, instr);
    return 0;
}
