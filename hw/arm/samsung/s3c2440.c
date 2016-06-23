/*
 *  s3c2440 (mini2440 board) simulator
 *
 *   Copyright (C) 2016 yanl.
 *   yanl, <yanl1229@163.com>
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License as published by the
 *  Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful, but WITHOUT
 *  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *  FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 *  for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, see <http://www.gnu.org/licenses/>.
 */
#include "hw/boards.h"
#include "sysemu/sysemu.h"
#include "hw/sysbus.h"
#include "hw/arm/arm.h"
#include "hw/loader.h"
#include "hw/usb/hcd-ehci.h"
#include "hw/arm/s3c2440.h"
#undef DEBUG

//#define DEBUG

#ifdef DEBUG
    #undef PRINT_DEBUG
    #define  PRINT_DEBUG(fmt, args...) \
        do { \
            fprintf(stderr, "  [%s:%d]   "fmt, __func__, __LINE__, ##args); \
        } while (0)
#else
    #define  PRINT_DEBUG(fmt, args...)  do {} while (0)
#endif


static struct arm_boot_info s3c2440_binfo = {
    .loader_start     = 0,
    .smp_loader_start = 0,
    .nb_cpus          = 1,
};

typedef struct S3C24xxState {
	ARMCPU *cpu;
	qemu_irq *irq_table;
	MemoryRegion sdram;
	MemoryRegion rom;
}S3C24xxState;

static S3C24xxState *s3c2440_mem_init(MemoryRegion *system_mem, unsigned long ram_size)
{
    ObjectClass *cpu;
    Error *err = NULL;
    DeviceState *dev;
    qemu_irq pic[32];
    qemu_irq pic1[32];
	int i;

	S3C24xxState *s = g_new(S3C24xxState, 1); 
	cpu = cpu_class_by_name(TYPE_ARM_CPU, "arm920");
	assert(cpu);
	Object *cpuobj = object_new(object_class_get_name(cpu));
    s->cpu = ARM_CPU(cpuobj);
    object_property_set_bool(cpuobj, true, "realized", &err);
    if (err) {
		error_report("%s", error_get_pretty(err));
	    exit(1);
	}
	/*s3c2440 intc*/
    dev = sysbus_create_varargs("s3c_intc", S3C2440_INTC,
                                qdev_get_gpio_in(DEVICE(cpuobj), ARM_CPU_IRQ),
                                qdev_get_gpio_in(DEVICE(cpuobj), ARM_CPU_FIQ),
                                NULL);
    for (i = 0; i < 32; i++) {
        pic[i] = qdev_get_gpio_in(dev, i);
    }

    dev = sysbus_create_simple("s3c_intor", -1, pic[1]);
    for (i = 0; i < 32; i++) {
        pic1[i] = qdev_get_gpio_in(dev, i);
    }

	/*sdramc*/
    memory_region_init_ram(&s->sdram, NULL, "s3c_sdram",0x8000000);
    vmstate_register_ram_global(&s->sdram);
    memory_region_add_subregion(system_mem,SDRAM_nGCS6, &s->sdram);
	/*rom*/
    memory_region_init_ram(&s->rom, NULL, "s3c_rom",0x28000000);
    vmstate_register_ram_global(&s->rom);
    memory_region_set_readonly(&s->rom, true);
    memory_region_add_subregion(system_mem,SROM_nGCS0, &s->rom);
	/*create nand flash*/
   sysbus_create_varargs("s3c_nand", S3C2440_NFCONF, pic1[0]);
	return s;
	return NULL;
}

static void s3c2440_init(QEMUMachineInitArgs *args)
{
	s3c2440_binfo.ram_size = args->ram_size;
	s3c2440_binfo.board_id = 0;
	s3c2440_binfo.kernel_filename = args->kernel_filename;
	s3c2440_binfo.initrd_filename = args->initrd_filename;
	s3c2440_binfo.kernel_cmdline = args->kernel_cmdline;
    s3c2440_mem_init(get_system_memory(), s3c2440_binfo.ram_size);

    arm_load_kernel(ARM_CPU(first_cpu), &s3c2440_binfo);
}

static QEMUMachine s3c2440_machine = {
        .name = "s3c2440",
        .desc = "s3c2440 (mini2440 board)",
        .init = s3c2440_init,
        .max_cpus = 1,
};

static void s3c2440_machine_init(void)
{
    qemu_register_machine(&s3c2440_machine);
}

machine_init(s3c2440_machine_init);
