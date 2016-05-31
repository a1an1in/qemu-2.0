#ifndef _HW_AT91SAM9260_H_
#define _HW_AT91SAM9260_H_

#include "qemu-common.h"
#include "exec/memory.h"

/* base periph addresses */
#define AT91_LCDC_BASE        0x00700000
#define AT91_PERIPH_BASE     0xF0000000
#define AT91_TC012_BASE      0xFFFA0000
#define AT91_USART0_BASE     0xFFFB0000
#define AT91_USART1_BASE     0xFFFB4000
#define AT91_USART2_BASE     0xFFFB8000
#define AT91_USART3_BASE     0xFFFD0000
#define AT91_EMAC_BASE       0xFFFC4000
#define AT91_SPI0_BASE       0xFFFC8000
#define AT91_SDRAMC0_BASE    0xFFFFEA00
#define AT91_SMC0_BASE       0xFFFFEC00
#define AT91_ECC1_BASE       0xFFFFE800
#define AT91_BUS_MATRIX_BASE 0xFFFFEE00
#define AT91_CCFG_BASE       0xFFFFEF10
#define AT91_DBGU_BASE       0xFFFFF200
#define AT91_AIC_BASE        0xFFFFF000
#define AT91_PIOA_BASE       0xFFFFF400
#define AT91_PIOB_BASE       0xFFFFF600
#define AT91_PIOC_BASE       0xFFFFF800
#define AT91_PMC_BASE        0xFFFFFC00
#define AT91_RSTC_BASE       0xFFFFFD00
#define AT91_PITC_BASE       0xFFFFFD30
#define AT91_WDT_BASE        0xFFFFFD40

/*nand flash*/
#define AT91_NAND_BASE       0x40000000
/* matrix */

// *****************************************************************************
//              SOFTWARE API DEFINITION  FOR AHB Matrix Interface
// *****************************************************************************
// *** Register offset in AT91S_MATRIX structure ***
#define MATRIX_MCFG0    ( 0) //  Master Configuration Register 0
#define MATRIX_MCFG1    ( 4) //  Master Configuration Register 1
#define MATRIX_MCFG2    ( 8) //  Master Configuration Register 2
#define MATRIX_MCFG3    (12) //  Master Configuration Register 3
#define MATRIX_MCFG4    (16) //  Master Configuration Register 4
#define MATRIX_MCFG5    (20) //  Master Configuration Register 5
#define MATRIX_MCFG6    (24) //  Master Configuration Register 6
#define MATRIX_MCFG7    (28) //  Master Configuration Register 7
#define MATRIX_MCFG8    (32) //  Master Configuration Register 8
#define MATRIX_SCFG0    (64) //  Slave Configuration Register 0
#define MATRIX_SCFG1    (68) //  Slave Configuration Register 1
#define MATRIX_SCFG2    (72) //  Slave Configuration Register 2
#define MATRIX_SCFG3    (76) //  Slave Configuration Register 3
#define MATRIX_SCFG4    (80) //  Slave Configuration Register 4
#define MATRIX_SCFG5    (84) //  Slave Configuration Register 5
#define MATRIX_SCFG6    (88) //  Slave Configuration Register 6
#define MATRIX_SCFG7    (92) //  Slave Configuration Register 7
#define MATRIX_PRAS0    (128) //  PRAS0
#define MATRIX_PRBS0    (132) //  PRBS0
#define MATRIX_PRAS1    (136) //  PRAS1
#define MATRIX_PRBS1    (140) //  PRBS1
#define MATRIX_PRAS2    (144) //  PRAS2
#define MATRIX_PRBS2    (148) //  PRBS2
#define MATRIX_PRAS3    (152) //  PRAS3
#define MATRIX_PRBS3    (156) //  PRBS3
#define MATRIX_PRAS4    (160) //  PRAS4
#define MATRIX_PRBS4    (164) //  PRBS4
#define MATRIX_PRAS5    (168) //  PRAS5
#define MATRIX_PRBS5    (172) //  PRBS5
#define MATRIX_PRAS6    (176) //  PRAS6
#define MATRIX_PRBS6    (180) //  PRBS6
#define MATRIX_PRAS7    (184) //  PRAS7
#define MATRIX_PRBS7    (188) //  PRBS7
#define MATRIX_MRCR     (256) //  Master Remp Control Register

#define AT91C_MATRIX_RCA926I      (0x1 <<  0) // (MATRIX) Remap Command Bit for ARM926EJ-S Instruction
#define AT91C_MATRIX_RCA926D      (0x1 <<  1) // (MATRIX) Remap Command Bit for ARM926EJ-S Data
#define AT91C_MATRIX_RCB2         (0x1 <<  2) // (MATRIX) Remap Command Bit for PDC
#define AT91C_MATRIX_RCB3         (0x1 <<  3) // (MATRIX) Remap Command Bit for LCD
#define AT91C_MATRIX_RCB4         (0x1 <<  4) // (MATRIX) Remap Command Bit for 2DGC
#define AT91C_MATRIX_RCB5         (0x1 <<  5) // (MATRIX) Remap Command Bit for ISI
#define AT91C_MATRIX_RCB6         (0x1 <<  6) // (MATRIX) Remap Command Bit for DMA
#define AT91C_MATRIX_RCB7         (0x1 <<  7) // (MATRIX) Remap Command Bit for EMAC
#define AT91C_MATRIX_RCB8         (0x1 <<  8) // (MATRIX) Remap Command Bit for USB

/*pitc */
#define AT91_PTIC_MR_PITEN  (1 << 24)
#define AT91_PTIC_MR_PITIEN (1 << 25)
#define AT91_PITC_MR      0
#define AT91_PITC_SR   (0x4 / sizeof(uint32_t))
#define AT91_PITC_PIVR (0x8 / sizeof(uint32_t))
#define AT91_PITC_PIIR (0xC / sizeof(uint32_t))

/*AIC registers*/
#define AT91_AIC_SVR0  (0x80  / sizeof(uint32_t))
#define AT91_AIC_ISR   (0x108 / sizeof(uint32_t))
#define AT91_AIC_IECR  (0x120 / sizeof(uint32_t))
#define AT91_AIC_EOICR (0x130 / sizeof(uint32_t))
#define AT91_AIC_IVR   (0x100 / sizeof(uint32_t))
#define AT91_AIC_IDCR  (0x124 / sizeof(uint32_t))

#define AT91_PERIPH_SYS_ID 1

#endif//!_HW_AT91SAM9263_DEFS_H_
