/*
 * AT91SAM9260ek Soc board clock simulation.
 *
 * Copyright (C) 2016 Yanl.
 * Written by Yanl <yanl1229@163.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#include "qemu-common.h"
#include "hw/sysbus.h"
#include "sysemu/sysemu.h"
#include "hw/hw.h"

/*#define AT91_DEBUG 1*/

#ifdef AT91_DEBUG
#define at91_debug(fs,...)					\
    fprintf(stderr,"at91: %s: "fs,__func__,##__VA_ARGS__)
#else
#define at91_debug(fs,...)
#endif

#define TYPE_AT91_PMC "at91_pmc"
#define AT91_PMC(obj) OBJECT_CHECK(AT91PMCState, (obj), TYPE_AT91_PMC)

/*watch dog register offset define*/
#define AT91PMC_REGS	0xFC
#define AT91_PMC_REGS_SIZE AT91PMC_REGS 
/*pmc register offset define*/
#define PMC_SCER 0x00
#define PMC_SCDR 0x04
#define PMC_SCSR 0x08
#define PMC_PCER 0x10
#define PMC_PCDR 0x14
#define PMC_PCSR 0x18
#define CKGR_MOR 0x20
#define CKGR_MCFR 0x24
#define CKGR_PLLAR 0x28
#define CKGR_PLLBR 0x2C
#define PMC_MCKR 0x30
#define PMC_PCK0 0x40
#define PMC_PCK1 0x44
#define PMC_IER  0x60
#define PMC_IDR  0x64
#define PMC_SR   0x68
#define PMC_IMR  0x6C
#define PMC_PLLICPR 0x80

#define AT91_PMC_MASK 0xFF 

/*ckgr_mor register field define*/
#define MOSCEN	(1 << 0)

/*pmc sr register field define*/
#define SR_MOSCS (1 << 0)

/*pmc ier regiser field define*/
#define IER_MOSCS  (1 << 0)

typedef struct AT91PMCstate {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
	uint32_t regs[AT91_PMC_REGS_SIZE];
} AT91PMCState;

static uint64_t at91_pmc_read(void *opaque, hwaddr offset, unsigned size)
{
	uint64_t ret = 0;

	switch (offset & AT91_PMC_MASK) {
	case PMC_SCSR:
		break;
	case PMC_PCSR:
		break;
	case CKGR_MCFR:
		break;
	case PMC_SR:
		break;
	case PMC_IMR:
		break;
	case CKGR_MOR:
		break;
	case CKGR_PLLAR:
		break;
	case CKGR_PLLBR:
		break;
	case PMC_MCKR:
		break;
	case PMC_PCK0:
		break;
	case PMC_PCK1:
		break;
	case PMC_PLLICPR:
		break;
	default:
		ret = 0;
		break;
	}
	
	return ret;
}

static void at91_pmc_write(void *opaque, hwaddr offset, uint64_t val, 
		unsigned size)
{

	switch (offset & AT91_PMC_MASK) {
	case PMC_SCER:
		break;
	case PMC_SCDR:
		break;
	case PMC_PCER:
		break;
	case PMC_PCDR:
		break;
	case PMC_IER:
		break;
	case PMC_IDR:
		break;
	case CKGR_MOR:
		break;
	case CKGR_PLLAR:
		break;
	case CKGR_PLLBR:
		break;
	case PMC_MCKR:
		break;
	case PMC_PCK0:
		break;
	case PMC_PCK1:
		break;
	case PMC_PLLICPR:
		break;
	default:
		break;
	}

}

static const MemoryRegionOps at91pmc_ops = {
    .read = at91_pmc_read,
    .write = at91_pmc_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .max_access_size = 4,
        .unaligned = false
    },
};

#if 0
static void at91_pmc_realize(DeviceState *dev, Error **errp)
{
    //AT91PMCState *s = AT91_PMC(dev);
}
#endif

static void at91_pmc_init(Object *obj)
{
	SysBusDevice *dev = SYS_BUS_DEVICE(obj);
    AT91PMCState *s = AT91_PMC(dev);
    /* memory mapping */
    memory_region_init_io(&s->iomem, OBJECT(s), &at91pmc_ops, s,
                          "at91_pmc", AT91_PMC_REGS_SIZE);
    sysbus_init_mmio(dev, &s->iomem);
	/*Todo:需要加入中断模式的支持*/

   // sysbus_init_irq(dev, &s->irq);
}


static void at91_pmc_reset(DeviceState *dev)
{
    AT91PMCState *s = AT91_PMC(dev);
	memset(&s->regs, 0, AT91_PMC_REGS_SIZE);
}


static const VMStateDescription vmstate_at91_pmc = {
    .name = "at91_pmc",
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .fields = (VMStateField[]) {
		VMSTATE_UINT32_ARRAY(regs, AT91PMCState, AT91PMC_REGS),
        VMSTATE_END_OF_LIST()
    }
};

static void at91_pmc_class_init(ObjectClass *klass, void *data)
{

    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->reset = at91_pmc_reset;
    //dc->realize = at91_pmc_realize;
    dc->vmsd = &vmstate_at91_pmc;
}

static const TypeInfo at91_pmc_info = {
    .name          = TYPE_AT91_PMC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AT91PMCState),
	.instance_init = at91_pmc_init,
    .class_init    = at91_pmc_class_init,
};

static void at91_pmc_register(void)
{
    type_register_static(&at91_pmc_info);
}

type_init(at91_pmc_register)
