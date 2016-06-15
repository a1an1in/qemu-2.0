/*
 * AT91sam9260ek board Ethernet MAC 10/100
 * Copyright (c) 2016 yanl 
 * Written by yanl <yanl1229@163.com> 
 *
 * This code is licensed under the GPL.
 */

#include "hw/sysbus.h"

#define AT91EMAC_REGMAX 0xC4

#define AT91EMAC_FIFO	28

#define TYPE_AT91EMAC "at91_emac"
#define AT91_EMAC(obj) OBJECT_CHECK(AT91EMACState, (obj), TYPE_AT91EMAC)

typedef struct AT91EmacFifo {
	int write, read;
	char buf[AT91EMAC_FIFO];
}AT91EMACfifo;

typedef struct AT91EMACState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
	AT91EMACfifo receive;
	AT91EMACfifo transmit;
    qemu_irq irq;
	uint32_t regs[AT91EMAC_REGMAX];
} AT91EMACState;

typedef struct RecvBufferDesc {
	uint32_t addr;
	uint32_t status;
}RecvBufferDesc;

static void at91emac_fifo_init(AT91EMACfifo *fifo)
{
	fifo->write = 0;
	fifo->read = 0;
	memset(fifo->buf, 0, AT91EMAC_FIFO);
}

static int at91emac_fifo_write(AT91EMACfifo *fifo, char *buf, int count)
{

	if (buf == NULL || count < 0)
		return -1;

	if (fifo->write >= AT91EMAC_FIFO) {
		printf("eth transmit fifo overrun, reset \n");
		fifo->write = 0;
	}

	if ((fifo->write + count) > AT91EMAC_FIFO)
		count = AT91EMAC_FIFO - fifo->write;
	memcpy(fifo->buf + fifo->write, buf, count);
	fifo->write += count;
	return count;
}

static int at91emac_fifo_read(AT91EMACfifo *fifo, char *buf, int count)
{
	if (buf == NULL || count < 0)
		return -1;

	if (fifo->read >= AT91EMAC_FIFO) {
		printf("eth receive fifo overrun!, reset \n");
		fifo->read = 0;
	}
	if ((fifo->read + count) > AT91EMAC_FIFO)
		count = AT91EMAC_FIFO - fifo->write;
	memcpy(fifo->buf + fifo->read, buf, count);
	fifo->read += count;
	return count;
}

static uint64_t at91emac_read(void *opaque, hwaddr offset,
                           unsigned size)
{
	AT91EMACState *s = (AT91EMACState *)opaque;
	at91emac_fifo_read(&s->receive, NULL, 0);
	return 0;
}


static void at91emac_write(void *opaque, hwaddr offset,
                        uint64_t val, unsigned size)
{
	//cpu_physical_memory_read();
	AT91EMACState *s = (AT91EMACState *)opaque;
	at91emac_fifo_write(&s->transmit, NULL, 0);

}

static const MemoryRegionOps at91emac_ops = {
    .read = at91emac_read,
    .write = at91emac_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void at91emac_reset(DeviceState *d)
{
    AT91EMACState *s = AT91_EMAC(d);
	memset(s->regs, 0, sizeof(*s->regs));
	at91emac_fifo_init(&s->transmit);
	at91emac_fifo_init(&s->receive);
}

static int at91emac_init(SysBusDevice *sbd)
{
    DeviceState *dev = DEVICE(sbd);
    AT91EMACState *s = AT91_EMAC(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &at91emac_ops, s, 
			"at91_emac", 0x1000);
    sysbus_init_mmio(sbd, &s->iomem);
	at91emac_fifo_init(&s->transmit);
	at91emac_fifo_init(&s->receive);
    return 0;
}

static const VMStateDescription vmstate_at91_emac = {
    .name = "at91_emac",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_END_OF_LIST()
    }
};

static void at91emac_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    k->init = at91emac_init;
    dc->reset = at91emac_reset;
    dc->vmsd = &vmstate_at91_emac;
}

static const TypeInfo at91emac_info = {
    .name          = TYPE_AT91EMAC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AT91EMACState),
    .class_init    = at91emac_class_init,
};

static void at91emac_register_types(void)
{
    type_register_static(&at91emac_info);
}

type_init(at91emac_register_types)
