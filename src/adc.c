#include <stdio.h>
#include <string.h>
#include "adc.h"
#include "emulator.h"

/* Global ADC state */
adc_state_t adc_state;

/* Initialize ADC */
void adc_init(void) {
    adc_reset();
}

/* Reset ADC to power-on defaults */
void adc_reset(void) {
    memset(&adc_state, 0, sizeof(adc_state_t));

    /* Default channel values: 0 for GPIO channels */
    for (int i = 0; i < ADC_NUM_CHANNELS; i++) {
        adc_state.channel_values[i] = 0;
    }

    /*
     * Temperature sensor default: ~27C
     * RP2040 formula: T = 27 - (ADC_voltage - 0.706) / 0.001721
     * At 27C: voltage = 0.706V, ADC value = 0.706 / 3.3 * 4095 = ~876
     * Use 0x036C (876 decimal)
     */
    adc_state.channel_values[ADC_TEMP_CHANNEL] = 0x036C;
}

/* Set a channel's analog value externally */
void adc_set_channel_value(uint8_t channel, uint16_t value) {
    if (channel < ADC_NUM_CHANNELS) {
        adc_state.channel_values[channel] = value & 0x0FFF; /* 12-bit */
    }
}

/* Read ADC register */
uint32_t adc_read32(uint32_t addr) {
    /* Strip atomic aliases - ADC occupies 0x4004C000-0x4004FFFF */
    uint32_t offset = addr & 0xFFF;

    switch (offset) {
        case 0x00: /* CS */
            /* Always report READY when not actively converting */
            return (adc_state.cs & ~ADC_CS_READY) | ADC_CS_READY;

        case 0x04: { /* RESULT */
            uint32_t ainsel = (adc_state.cs & ADC_CS_AINSEL_MASK)
                               >> ADC_CS_AINSEL_SHIFT;
            if (ainsel < ADC_NUM_CHANNELS) {
                return adc_state.channel_values[ainsel];
            }
            return 0;
        }

        case 0x08: /* FCS */
            return adc_state.fcs;

        case 0x0C: { /* FIFO */
            /* Return current channel value as if from FIFO */
            uint32_t ainsel = (adc_state.cs & ADC_CS_AINSEL_MASK)
                               >> ADC_CS_AINSEL_SHIFT;
            if (ainsel < ADC_NUM_CHANNELS) {
                return adc_state.channel_values[ainsel];
            }
            return 0;
        }

        case 0x10: /* DIV */
            return adc_state.div;

        case 0x14: /* INTR */
            return adc_state.intr;

        case 0x18: /* INTE */
            return adc_state.inte;

        case 0x1C: /* INTF */
            return 0;

        case 0x20: /* INTS */
            return adc_state.intr & adc_state.inte;

        default:
            return 0;
    }
}

/* Write ADC register */
void adc_write32(uint32_t addr, uint32_t val) {
    /* Decode atomic alias */
    uint32_t offset_full = addr - (addr & ~0x3FFF);
    uint32_t alias = (offset_full >> 12) & 0x3;
    uint32_t offset = addr & 0xFFF;

    switch (offset) {
        case 0x00: { /* CS */
            uint32_t new_cs;
            switch (alias) {
                case 0: new_cs = val; break;
                case 1: new_cs = adc_state.cs ^ val; break;
                case 2: new_cs = adc_state.cs | val; break;
                case 3: new_cs = adc_state.cs & ~val; break;
                default: new_cs = val; break;
            }
            adc_state.cs = new_cs & (ADC_CS_EN | ADC_CS_TS_EN |
                                     ADC_CS_START_ONCE | ADC_CS_START_MANY |
                                     ADC_CS_AINSEL_MASK | ADC_CS_RROBIN_MASK);
            /* START_ONCE is auto-clearing - "conversion" completes instantly */
            if (adc_state.cs & ADC_CS_START_ONCE) {
                adc_state.cs &= ~ADC_CS_START_ONCE;
            }
            break;
        }

        case 0x08: /* FCS */
            switch (alias) {
                case 0: adc_state.fcs = val; break;
                case 1: adc_state.fcs ^= val; break;
                case 2: adc_state.fcs |= val; break;
                case 3: adc_state.fcs &= ~val; break;
            }
            break;

        case 0x10: /* DIV */
            switch (alias) {
                case 0: adc_state.div = val; break;
                case 1: adc_state.div ^= val; break;
                case 2: adc_state.div |= val; break;
                case 3: adc_state.div &= ~val; break;
            }
            break;

        case 0x18: /* INTE */
            switch (alias) {
                case 0: adc_state.inte = val; break;
                case 1: adc_state.inte ^= val; break;
                case 2: adc_state.inte |= val; break;
                case 3: adc_state.inte &= ~val; break;
            }
            break;

        default:
            break;
    }
}
