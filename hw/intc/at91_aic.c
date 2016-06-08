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
    int irq;
    int priority;
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
    return (s->level | s->soft_level) & s->irq_enable & ~s->fiq_select;
}

/* Update interrupts.  */
static void at91aic_update(AT91AICState *s)
{
    uint32_t level = at91aic_irq_level(s);
    int set;

    set = (level & s->prio_mask[s->priority]) != 0;
    qemu_set_irq(s->irq, set);
    set = ((s->level | s->soft_level) & s->fiq_select) != 0;
    qemu_set_irq(s->fiq, set);
}

static void at91aic_set_irq(void *opaque, int irq, int level)
{
    AT91AICState *s = (AT91AICState *)opaque;

    if (level)
        s->level |= 1u << irq;
    else
        s->level &= ~(1u << irq);
    at91aic_update(s);
}

static void at91aic_update_vectors(AT91AICState *s)
{
    uint32_t mask;
    int i;
    int n;

    mask = 0;
    for (i = 0; i < 16; i++)
      {
        s->prio_mask[i] = mask;
        if (s->vect_control[i] & 0x20)
          {
            n = s->vect_control[i] & 0x1f;
            mask |= 1 << n;
          }
      }
    s->prio_mask[16] = mask;
    at91aic_update(s);
}

static uint64_t at91aic_read(void *opaque, hwaddr offset,
                           unsigned size)
{
    AT91AICState *s = (AT91AICState *)opaque;
    int i;

    if (offset >= 0xfe0 && offset < 0x1000) {
        return at91aic_id[(offset - 0xfe0) >> 2];
    }
    if (offset >= 0x100 && offset < 0x140) {
        return s->vect_addr[(offset - 0x100) >> 2];
    }
    if (offset >= 0x200 && offset < 0x240) {
        return s->vect_control[(offset - 0x200) >> 2];
    }
    switch (offset >> 2) {
    case 0: /* IRQSTATUS */
        return at91aic_irq_level(s);
    case 1: /* FIQSATUS */
        return (s->level | s->soft_level) & s->fiq_select;
    case 2: /* RAWINTR */
        return s->level | s->soft_level;
    case 3: /* INTSELECT */
        return s->fiq_select;
    case 4: /* INTENABLE */
        return s->irq_enable;
    case 6: /* SOFTINT */
        return s->soft_level;
    case 8: /* PROTECTION */
        return s->protected;
    case 12: /* VECTADDR */
        /* Read vector address at the start of an ISR.  Increases the
         * current priority level to that of the current interrupt.
         *
         * Since an enabled interrupt X at priority P causes prio_mask[Y]
         * to have bit X set for all Y > P, this loop will stop with
         * i == the priority of the highest priority set interrupt.
         */
        for (i = 0; i < s->priority; i++) {
            if ((s->level | s->soft_level) & s->prio_mask[i + 1]) {
                break;
            }
        }

        /* Reading this value with no pending interrupts is undefined.
           We return the default address.  */
        if (i == AT91AIC_NUM_PRIO)
          return s->vect_addr[16];
        if (i < s->priority)
          {
            s->prev_prio[i] = s->priority;
            s->priority = i;
            at91aic_update(s);
          }
        return s->vect_addr[s->priority];
    case 13: /* DEFVECTADDR */
        return s->vect_addr[16];
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "at91aic_read: Bad offset %x\n", (int)offset);
        return 0;
    }
}

static void at91aic_write(void *opaque, hwaddr offset,
                        uint64_t val, unsigned size)
{
    AT91AICState *s = (AT91AICState *)opaque;

    if (offset >= 0x100 && offset < 0x140) {
        s->vect_addr[(offset - 0x100) >> 2] = val;
        at91aic_update_vectors(s);
        return;
    }
    if (offset >= 0x200 && offset < 0x240) {
        s->vect_control[(offset - 0x200) >> 2] = val;
        at91aic_update_vectors(s);
        return;
    }
    switch (offset >> 2) {
    case 0: /* SELECT */
        /* This is a readonly register, but linux tries to write to it
           anyway.  Ignore the write.  */
        break;
    case 3: /* INTSELECT */
        s->fiq_select = val;
        break;
    case 4: /* INTENABLE */
        s->irq_enable |= val;
        break;
    case 5: /* INTENCLEAR */
        s->irq_enable &= ~val;
        break;
    case 6: /* SOFTINT */
        s->soft_level |= val;
        break;
    case 7: /* SOFTINTCLEAR */
        s->soft_level &= ~val;
        break;
    case 8: /* PROTECTION */
        /* TODO: Protection (supervisor only access) is not implemented.  */
        s->protected = val & 1;
        break;
    case 12: /* VECTADDR */
        /* Restore the previous priority level.  The value written is
           ignored.  */
        if (s->priority < AT91AIC_NUM_PRIO)
            s->priority = s->prev_prio[s->priority];
        break;
    case 13: /* DEFVECTADDR */
        s->vect_addr[16] = val;
        break;
    case 0xc0: /* ITCR */
        if (val) {
            qemu_log_mask(LOG_UNIMP, "pl190: Test mode not implemented\n");
        }
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                     "at91aic_write: Bad offset %x\n", (int)offset);
        return;
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
    int i;

    for (i = 0; i < 16; i++) {
        s->vect_addr[i] = 0;
        s->vect_control[i] = 0;
    }
    s->vect_addr[16] = 0;
    s->prio_mask[17] = 0xffffffff;
    s->priority = AT91AIC_NUM_PRIO;
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
        VMSTATE_UINT32(irq, AT91AICState),
        VMSTATE_INT32(priority, AT91AICState),
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
