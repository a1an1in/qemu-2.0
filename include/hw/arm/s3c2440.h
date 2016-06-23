#ifndef _S3C2440_H_
#define _S3C2440_H_

/*rom memory address map*/
#define SROM_nGCS0 0x00000000
#define SROM_nGCS1 0x08000000
#define SROM_nGCS2 0x10000000
#define SROM_nGCS3 0x18000000
#define SROM_nGCS4 0x20000000
#define SROM_nGCS5 0x28000000

/*sdram memory address map*/
#define SDRAM_nGCS6 0x30000000
#define SDRAM_nGCS7 0x38000000

/*s3c2440 interrupt controller address base*/
#define S3C2440_INTC 0x4a000000
/*s3c2440 nand controller address base */
#define S3C2440_NFCONF 0x4e000000
#endif
