/*
 * AT91sam9260ek board Advanced Interrupt Controller 
 * Copyright (c) 2016 yanl 
 * Written by yanl <yanl1229@163.com> 
 *
 * This code is licensed under the GPL.
 */

#include "hw/sysbus.h"

typedef struct AT91AICStack {
	uint16_t level;
	uint16_t irq;
	struct At91AICStack *next;
}AT91AICStack;

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
#define AT91AIC_FFSR		0x14C
#define AT91AIC_REGMAX		0x14C

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
	AT91AICStack stack[AT91AIC_NUM_PRIO];
	/*aic control registers*/
	AT91AICRegs *regs;
} AT91AICState;

static const unsigned char at91aic_id[] =
{ 0x90, 0x11, 0x04, 0x00, 0x0D, 0xf0, 0x05, 0xb1 };

static inline uint32_t at91aic_irq_level(AT91AICState *s)
{
    return 0;
}

/* Update interrupts.  */
static void at91aic_update(AT91AICState *s)
{
    int set;
    set = 0;
    qemu_set_irq(s->irq, set);
    set = 0;
    qemu_set_irq(s->fiq, set);
}

static void at91aic_set_irq(void *opaque, int irq, int level)
{
    //at91aic_update(s);
}

static void at91aic_update_vectors(AT91AICState *s)
{
    at91aic_update(s);
}

static uint64_t at91aic_read(void *opaque, hwaddr offset,
                           unsigned size)
{
	return 0;
}

static void at91aic_write(void *opaque, hwaddr offset,
                        uint64_t val, unsigned size)
{
    AT91AICState *s = (AT91AICState *)opaque;

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
    at91aic_update_vectors(s);
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
