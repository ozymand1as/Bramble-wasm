/*
 * Picobin IMAGE_DEF Block Parser for RP2350
 *
 * Scans the first 4KB of flash for a picobin block and extracts
 * CPU type, entry point, and stack pointer.
 */

#include <string.h>
#include <stdio.h>
#include "rp2350_rv/picobin.h"

picobin_info_t picobin_scan(const uint8_t *flash, uint32_t scan_size) {
    picobin_info_t info;
    memset(&info, 0, sizeof(info));

    if (scan_size < 8) return info;

    /* Scan for block start marker (aligned to 4 bytes) */
    for (uint32_t pos = 0; pos + 8 <= scan_size; pos += 4) {
        uint32_t word;
        memcpy(&word, &flash[pos], 4);
        if (word != PICOBIN_BLOCK_START) continue;

        /* Found block start — parse items */
        uint32_t item_pos = pos + 4;

        fprintf(stderr, "[PICOBIN] Found block marker at flash offset 0x%04X\n", pos);

        while (item_pos + 4 <= scan_size) {
            uint32_t item_word;
            memcpy(&item_word, &flash[item_pos], 4);

            uint8_t type = item_word & 0xFF;
            uint8_t size = (item_word >> 8) & 0xFF;
            uint16_t flags = (uint16_t)(item_word >> 16);

            if (type == PICOBIN_ITEM_LAST) {
                /* End of items — check for block end marker */
                /* The LAST item's size field contains the block word count */
                /* After LAST: block_loop_offset (4 bytes) + end marker (4 bytes) */
                info.found = 1;
                fprintf(stderr, "[PICOBIN] Block parsed successfully\n");
                break;
            }

            if (size == 0) {
                /* Invalid — zero-length item, skip */
                break;
            }

            switch (type) {
            case PICOBIN_ITEM_IMAGE_TYPE:
                /* flags contains the image type info */
                if (flags & PICOBIN_IMAGE_TYPE_EXE) {
                    uint32_t cpu = (flags >> 8) & 0x7;
                    uint32_t chip = (flags >> 12) & 0x7;
                    info.is_riscv = (cpu == 1);
                    info.is_arm = (cpu == 0);
                    info.is_rp2350 = (chip == 1);
                    fprintf(stderr, "[PICOBIN] IMAGE_TYPE: %s on %s (flags=0x%04X)\n",
                            info.is_riscv ? "RISC-V" : "ARM",
                            info.is_rp2350 ? "RP2350" : "RP2040",
                            flags);
                }
                break;

            case PICOBIN_ITEM_ENTRY_POINT:
                /* size=3: header + entry_pc + entry_sp */
                if (size >= 3 && item_pos + 12 <= scan_size) {
                    memcpy(&info.entry_pc, &flash[item_pos + 4], 4);
                    memcpy(&info.entry_sp, &flash[item_pos + 8], 4);
                    fprintf(stderr, "[PICOBIN] ENTRY_POINT: PC=0x%08X SP=0x%08X\n",
                            info.entry_pc, info.entry_sp);
                }
                break;

            case PICOBIN_ITEM_VECTOR_TABLE:
                /* size=2: header + vtor address */
                if (size >= 2 && item_pos + 8 <= scan_size) {
                    memcpy(&info.vtor, &flash[item_pos + 4], 4);
                    fprintf(stderr, "[PICOBIN] VECTOR_TABLE: 0x%08X\n", info.vtor);
                }
                break;

            default:
                /* Unknown item — skip based on size */
                break;
            }

            item_pos += size * 4;
        }

        if (info.found) break;
    }

    if (!info.found) {
        fprintf(stderr, "[PICOBIN] No picobin block found in first %u bytes\n", scan_size);
    }

    return info;
}
