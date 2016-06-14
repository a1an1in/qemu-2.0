/*
 * AT91sam9260ek board Advanced Interrupt Controller 
 * Copyright (c) 2016 yanl 
 * Written by yanl <yanl1229@163.com> 
 *
 * This code is licensed under the GPL.
 */

#include "hw/sysbus.h"

#define AT91AIC_SMR 32
#define AT91AIC_SVR 32

/*advanced interrupt register offset define*/
#define AT91AIC_SMRSTART	0x00
#define AT91AIC_SMREND		0x7C
#define AT91AIC_SVRSTART	0x80
#define AT91AIC_SVREND		0xFC
#define AT91AIC_IVR			0x100
#define AT91AIC_FVR			0x104
#define AT91AIC_ISR			0x108
#define AT91AIC_IPR			0x10C
#define AT91AIC_IMR			0x110
#define AT91AIC_CISR		0x114
/*reserved 0x11C*/
#define AT91AIC_IECR		0x120
#define AT91AIC_IDCR		0x124
#define AT91AIC_ICCR		0x128
#define AT91AIC_ISCR		0x12C
#define AT91AIC_EOICR		0x130
#define AT91AIC_SPU			0x134
#define AT91AIC_DCR			0x138
/*reserved 0x13c*/
#define AT91AIC_FFER		0x140
#define AT91AIC_FFDR		0x144
#define AT91AIC_FFSR		0x148
#define AT91AIC_REGMAX		0x148
#define AT91AIC_REGMASK		0x1FF

/*aic register field define*/
#define DCR_PROT        0x01 /* Protect Mode Enabled */
#define DCR_GMSK        0x02 /* General Mask */

typedef struct AT91AICRegs {
	uint32_t smr[AT91AIC_SMR];	
	uint32_t svr[AT91AIC_SVR];	
	uint32_t ivr;
	uint32_t fvr;
	uint32_t isr;
	uint32_t ipr;
	uint32_t imr;
	uint32_t cisr;
	uint32_t iecr;
	uint32_t idcr;
	uint32_t iccr;
	uint32_t iscr;
	uint32_t eoicr;
	uint32_t spu;
	uint32_t dcr;
	uint32_t ffer;
	uint32_t ffdr;
	uint32_t ffsr;
}AT91AICRegs;

#define AT91AIC_NUM_PRIO 8 

#define TYPE_AT91AIC "at91_aic"
#define AT91_AIC(obj) OBJECT_CHECK(AT91AICState, (obj), TYPE_AT91AIC)

typedef struct AT91AICState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
	/*current interrupt level irq and priority*/
    uint32_t level;
    uint32_t cur_irq;
    uint32_t priority;
    qemu_irq irq;
    qemu_irq fiq;
	uint8_t stack_irq[AT91AIC_NUM_PRIO];
	uint8_t stack_pri[AT91AIC_NUM_PRIO];
	int32_t stack_pos;
	/*aic control registers*/
	AT91AICRegs *regs;
} AT91AICState;


#define AIC_GET_PRIORITY(s, irq) \
	    ((int)((s)->regs->smr[(irq)] & 7))

#define AIC_IS_EDGE_TRIGGERED(s, irq) \
	    ((int)((s)->regs->smr[(irq)] & (1 << 5)))

static uint8_t at91aic_highest(AT91AICState *s, uint8_t *priority)
{
	uint32_t pending;
	uint8_t highest_interrupt = 0;
	uint8_t highest_priority = 0;
	int i;

    pending = s->regs->ipr & s->regs->imr & ~s->regs->ffsr;
    for (i = 31; i >= 1; i--) {
		if (pending & (1 << i)) {
			if (AIC_GET_PRIORITY(s, i) >= highest_priority) {
                highest_interrupt = i;
                highest_priority = AIC_GET_PRIORITY(s, i);
           }
		} 
	 }
    *priority = highest_priority;
    return highest_interrupt;
}


/* Update interrupts.  */
static void at91aic_update(AT91AICState *s)
{
	uint32_t pending = s->regs->ipr & s->regs->imr;
	uint32_t fiq_mask = 1 | s->regs->ffsr;

	if (s->regs->dcr & DCR_GMSK)
		s->regs->cisr = 0;
	else {
		s->regs->cisr = !!(pending & fiq_mask);
		if (pending & ~fiq_mask) {
			uint8_t highest;
			at91aic_highest(s, &highest);
			if (s->stack_pos < 0 || 
					s->stack_pri[s->stack_pos] < highest) 
				s->regs->cisr |= 2;
		}
	}
    qemu_set_irq(s->irq, s->regs->cisr & 2);
    qemu_set_irq(s->fiq, s->regs->cisr & 1);
}

static void at91aic_set_irq(void *opaque, int irq, int level)
{
	int mask = 1 << irq;
	AT91AICState *s = (AT91AICState *)opaque;
		    /* TODO: External egde-triggering */
    if (level)
        s->regs->ipr |= mask;
    else if (!AIC_IS_EDGE_TRIGGERED(s, irq))
        s->regs->ipr &= ~mask;
	at91aic_update(s);
}


static void at91_aic_irq_enter(AT91AICState *s, uint8_t irq, uint8_t priority)
{
    if (s->stack_pos < 7 && irq != 0) {
        s->stack_pos++;
		s->regs->isr = irq;
        s->stack_pri[s->stack_pos] = priority;

        if (AIC_IS_EDGE_TRIGGERED(s, irq)) {
            s->regs->ipr &= ~(1 << irq);
            at91aic_update(s);
        }
    }
}


static uint64_t at91aic_read(void *opaque, hwaddr offset,
                           unsigned size)
{
	int off = 0;
	uint64_t ret = 0;
    AT91AICState *s = (AT91AICState *)opaque;
    uint8_t current_irq;
    uint8_t current_pri;

	off = offset & AT91AIC_REGMASK; 
	if ((off >= AT91AIC_SMRSTART) && (off <= AT91AIC_SMREND)) {
		off -= AT91AIC_SMRSTART;
		ret = s->regs->smr[off];
	} else if ((off >= AT91AIC_SVRSTART) && (off <= AT91AIC_SVREND)) {
		off -= AT91AIC_SVRSTART;
		ret = s->regs->svr[off];
	}else switch (off) {
		case AT91AIC_IVR:
			 current_irq = at91aic_highest(s, &current_pri);
			 if (!(s->regs->dcr & DCR_PROT))
				 at91_aic_irq_enter(s, current_irq, current_pri);
			ret =  current_irq == 0 ? s->regs->spu : s->regs->svr[current_irq];
			break;
		case AT91AIC_FVR:
			if (s->regs->ipr & 1) {
				s->regs->ipr &= ~1;
			    at91aic_update(s);
			    ret = s->regs->svr[0];
			} else if (s->regs->ipr & s->regs->ffsr)
			    ret = s->regs->svr[0];
			else {
				ret = s->regs->spu;
				s->regs->ipr &= ~1;
			}
			break;
		case AT91AIC_ISR:
			ret = s->regs->isr;
			break;
		case AT91AIC_IPR:
			ret = s->regs->ipr;
			break;
		case AT91AIC_IMR:
			ret = s->regs->imr;
			break;
		case AT91AIC_CISR:
			ret = s->regs->cisr;
			break;
		case AT91AIC_SPU:
			ret = s->regs->spu;
			break;
		case AT91AIC_DCR:
			ret = s->regs->dcr;
			break;
		default:
			break;
	} 

	return ret;
}


static void at91aic_write(void *opaque, hwaddr offset,
                        uint64_t val, unsigned size)
{
	int off = 0;
	uint32_t irq;
	uint8_t current_irq;
	uint8_t current_pri;
    AT91AICState *s = (AT91AICState *)opaque;

	off = offset & AT91AIC_REGMASK; 
	if ((off >= AT91AIC_SMRSTART) && (off <= AT91AIC_SMREND)) {
		off -= AT91AIC_SMRSTART;
		s->regs->smr[off] = val&0xFF;
	} else if ((off >= AT91AIC_SVRSTART) && (off <= AT91AIC_SVREND)) {
		off -= AT91AIC_SVRSTART;
		s->regs->svr[off] = val;
	}else switch (off) {
    	case AT91AIC_IVR:
    		if (s->regs->dcr & DCR_PROT) {
    			current_irq = at91aic_highest(s, &current_pri);
    			at91_aic_irq_enter(s, current_irq, current_pri);
    		}
		case AT91AIC_IECR:
			s->regs->iecr |= val;
			s->regs->imr &= ~val;
			break;
		case AT91AIC_IDCR:
			s->regs->idcr |= val;
			s->regs->imr |= val;
			break;
		case AT91AIC_ICCR:
			s->regs->iccr |= val;
			 for (irq = 0; irq < 32; irq++) {
				 if (!AIC_IS_EDGE_TRIGGERED(s, irq))
					 val &= ~(1 << irq);
			        }
			 s->regs->ipr &= val;

			break;
		case AT91AIC_ISCR:
			s->regs->iscr |= val;
			for (irq = 0; irq < 32; irq++) {
				 if (!AIC_IS_EDGE_TRIGGERED(s, irq))
					 val &= ~(1 << irq);
			        }
			s->regs->ipr |= val;
			break;
		case AT91AIC_EOICR:
			s->regs->eoicr |= val;
			if (s->stack_pos >= 0)
				s->stack_pos--;
			break;
		case AT91AIC_SPU:
			s->regs->spu = val;
			break;
		case AT91AIC_DCR:
			s->regs->dcr |= (val & 3);
			break;
		case AT91AIC_FFER:
			s->regs->ffer |= (val & 0xFFFFFFFE);
			s->regs->ffsr |= (val & 0xFFFFFFFE);
			break;
		case AT91AIC_FFDR:
			s->regs->ffdr |= (val & 0xFFFFFFFE);
			s->regs->ffsr &= ~(val & 0xFFFFFFFE);
			break;
		default:
			break;
	} 
	 at91aic_update(s);

}

static const MemoryRegionOps at91aic_ops = {
    .read = at91aic_read,
    .write = at91aic_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void at91aic_reset(DeviceState *d)
{
    AT91AICState *s = AT91_AIC(d);
	memset(s->regs, 0, sizeof(*s->regs));
}

static int at91aic_init(SysBusDevice *sbd)
{
    DeviceState *dev = DEVICE(sbd);
    AT91AICState *s = AT91_AIC(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &at91aic_ops, s, "at91_aic", 0x1000);
    sysbus_init_mmio(sbd, &s->iomem);
    qdev_init_gpio_in(dev, at91aic_set_irq, 32);
    sysbus_init_irq(sbd, &s->irq);
    sysbus_init_irq(sbd, &s->fiq);
	s->regs = g_malloc(sizeof(AT91AICRegs));
	s->stack_pos = -1;
    return 0;
}

static const VMStateDescription vmstate_at91_regs = {
    .name = "at91_aic",
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(ivr, AT91AICRegs),
        VMSTATE_UINT32(fvr, AT91AICRegs),
        VMSTATE_UINT32(isr, AT91AICRegs),
        VMSTATE_UINT32(ipr, AT91AICRegs),
        VMSTATE_UINT32(imr, AT91AICRegs),
        VMSTATE_UINT32(cisr, AT91AICRegs),
        VMSTATE_UINT32(iecr, AT91AICRegs),
        VMSTATE_UINT32(idcr, AT91AICRegs),
        VMSTATE_UINT32(iccr, AT91AICRegs),
        VMSTATE_UINT32(iscr, AT91AICRegs),
        VMSTATE_UINT32(eoicr, AT91AICRegs),
        VMSTATE_UINT32(spu, AT91AICRegs),
        VMSTATE_UINT32(dcr, AT91AICRegs),
        VMSTATE_UINT32(ffer, AT91AICRegs),
        VMSTATE_UINT32(ffdr, AT91AICRegs),
        VMSTATE_UINT32(ffsr, AT91AICRegs),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_at91_aic = {
    .name = "at91_aic",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(level, AT91AICState),
        VMSTATE_UINT32(cur_irq, AT91AICState),
        VMSTATE_UINT32(priority, AT91AICState),
        VMSTATE_STRUCT_POINTER(regs, AT91AICState, 
                          vmstate_at91_regs, AT91AICRegs),
        VMSTATE_END_OF_LIST()
    }
};

static void at91aic_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    k->init = at91aic_init;
    dc->reset = at91aic_reset;
    dc->vmsd = &vmstate_at91_aic;
}

static const TypeInfo at91aic_info = {
    .name          = TYPE_AT91AIC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AT91AICState),
    .class_init    = at91aic_class_init,
};

static void at91aic_register_types(void)
{
    type_register_static(&at91aic_info);
}

type_init(at91aic_register_types)
