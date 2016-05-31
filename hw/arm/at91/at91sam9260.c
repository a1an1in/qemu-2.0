/*
 *  at91sam9260x board simulator
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
#include "hw/arm/at91sam9260-reg.h"
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


static struct arm_boot_info at91sam9260_binfo = {
    .loader_start     = 0,
    .smp_loader_start = 0,
    .nb_cpus          = 1,
};

typedef struct at91sam9260_state {
	ARMCPU *cpu;
	MemoryRegion internal_sram;
	qemu_irq *irq_table;
	MemoryRegion boot_rom;
	MemoryRegion sdramc;
	MemoryRegion rom;
}at91sam9260_state;

static at91sam9260_state *at91_mem_init(MemoryRegion *system_mem, unsigned long ram_size)
{
#if 0
    DeviceState *dev;
    DeviceState *pit;
    DeviceState *pmc;
    DeviceState *spi;
#endif
    ObjectClass *cpu;
    Error *err = NULL;
#if 0
    qemu_irq pic[32];
    qemu_irq pic1[32];
	int i;
#endif

	at91sam9260_state *s = g_new(at91sam9260_state, 1); 
	cpu = cpu_class_by_name(TYPE_ARM_CPU, "arm926");
	assert(cpu);
	Object *cpuobj = object_new(object_class_get_name(cpu));
    s->cpu = ARM_CPU(cpuobj);
    object_property_set_bool(cpuobj, true, "realized", &err);
    if (err) {
		error_report("%s", error_get_pretty(err));
	    exit(1);
	}
#if 0
	/*at91 aic*/
    dev = sysbus_create_varargs("at91.aic", AT91_AIC_BASE,
                                qdev_get_gpio_in(DEVICE(cpuobj), ARM_CPU_IRQ),
                                qdev_get_gpio_in(DEVICE(cpuobj), ARM_CPU_FIQ),
                                NULL);
    for (i = 0; i < 32; i++) {
        pic[i] = qdev_get_gpio_in(dev, i);
    }

    dev = sysbus_create_simple("at91.intor", -1, pic[1]);
    for (i = 0; i < 32; i++) {
        pic1[i] = qdev_get_gpio_in(dev, i);
    }

    sysbus_create_simple("at91.dbgu", AT91_DBGU_BASE, pic1[0]);
    pmc = sysbus_create_simple("at91.pmc", AT91_PMC_BASE, pic1[1]);
    qdev_prop_set_uint32(pmc, "mo_freq", 16000000);
    pit = sysbus_create_simple("at91.pit", AT91_PITC_BASE, pic1[3]);
    sysbus_create_varargs("at91.tc", AT91_TC012_BASE, pic[19], pic[19], pic[19], NULL);
    qdev_prop_set_uint32(pit, "pic", 0);
    spi = sysbus_create_simple("at91,spi", AT91_SPI0_BASE, pic[14]);
    qdev_prop_set_uint32(spi, "spi", 0);
    spi = sysbus_create_simple("at91.spi", AT91_SPI0_BASE, pic[14]);
	sysbus_create_varargs("at91.matrix", AT91_BUS_MATRIX_BASE, NULL);
    sysbus_create_simple("at91.pio", AT91_PIOA_BASE, pic[2]);
    sysbus_create_simple("at91.pio", AT91_PIOB_BASE, pic[3]);
    sysbus_create_simple("at91,pio", AT91_PIOC_BASE, pic[4]);
    sysbus_create_varargs("at91.rstc", AT91_RSTC_BASE, NULL);

    sysbus_create_simple("at91.emac", AT91_EMAC_BASE, pic[21]);
    sysbus_create_simple("at91.lcdc", AT91_LCDC_BASE, pic[26]);
#endif

	/*internal sram memory*/
    memory_region_init_ram(&s->internal_sram, NULL, "at91.isram",80*1024);
    vmstate_register_ram_global(&s->internal_sram);
    memory_region_add_subregion(system_mem,0x00300000, &s->internal_sram);
	/*boot ram */
    memory_region_init_ram(&s->boot_rom, NULL, "at91.boot_rom",0x100000);
    vmstate_register_ram_global(&s->boot_rom);
    memory_region_add_subregion(system_mem,0x00000000, &s->boot_rom);
	/*sdramc*/
    memory_region_init_ram(&s->sdramc, NULL, "at91.sdramc",0xFFFFFFF);
    vmstate_register_ram_global(&s->sdramc);
    memory_region_add_subregion(system_mem,0x20000000, &s->sdramc);
	/*rom*/
    memory_region_init_ram(&s->rom, NULL, "at91.rom",32*1024);
    vmstate_register_ram_global(&s->rom);
    memory_region_set_readonly(&s->rom, true);
    memory_region_add_subregion(system_mem,0x00100000, &s->rom);

	/*create at19 debug unit*/
    sysbus_create_varargs("at91sam9260_debug", AT91_DBGU_BASE, NULL);
	/*create nand flash*/
    //sysbus_create_varargs("nand", AT91_NAND_BASE, NULL);
	return s;
}

static void at91sam9260_init(QEMUMachineInitArgs *args)
{
	at91sam9260_binfo.ram_size = args->ram_size;
	at91sam9260_binfo.board_id = 0;
	at91sam9260_binfo.kernel_filename = args->kernel_filename;
	at91sam9260_binfo.initrd_filename = args->initrd_filename;
	at91sam9260_binfo.kernel_cmdline = args->kernel_cmdline;
    at91_mem_init(get_system_memory(), at91sam9260_binfo.ram_size);

    arm_load_kernel(ARM_CPU(first_cpu), &at91sam9260_binfo);
}

static QEMUMachine at91sam9260_machine = {
        .name = "at91sam9260",
        .desc = "at92sam9260 board",
        .init = at91sam9260_init,
        .max_cpus = 1,
};

static void at91sam9260_machine_init(void)
{
    qemu_register_machine(&at91sam9260_machine);
}

machine_init(at91sam9260_machine_init);
