/*
 *  AT91SAM9260ek board debug Emulation
 *
 *  Copyright (C) 2016 yanl.
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
 *
 */

#include "hw/sysbus.h"
#include "sysemu/sysemu.h"
#include "sysemu/char.h"

#include "hw/arm/at91sam9260-reg.h"

#undef DEBUG_UART
#undef DEBUG_UART_EXTEND
#undef DEBUG_IRQ
#undef DEBUG_Rx_DATA
#undef DEBUG_Tx_DATA

#define DEBUG_UART            0
#define DEBUG_UART_EXTEND     0
#define DEBUG_IRQ             0
#define DEBUG_Rx_DATA         0
#define DEBUG_Tx_DATA         0

#if DEBUG_UART
#define  PRINT_DEBUG(fmt, args...)  \
        do { \
            fprintf(stderr, "  [%s:%d]   "fmt, __func__, __LINE__, ##args); \
        } while (0)

#if DEBUG_UART_EXTEND
#define  PRINT_DEBUG_EXTEND(fmt, args...) \
        do { \
            fprintf(stderr, "  [%s:%d]   "fmt, __func__, __LINE__, ##args); \
        } while (0)
#else
#define  PRINT_DEBUG_EXTEND(fmt, args...) \
        do {} while (0)
#endif /* EXTEND */

#else
#define  PRINT_DEBUG(fmt, args...)  \
        do {} while (0)
#define  PRINT_DEBUG_EXTEND(fmt, args...) \
        do {} while (0)
#endif

#define  PRINT_ERROR(fmt, args...) \
        do { \
            fprintf(stderr, "  [%s:%d]   "fmt, __func__, __LINE__, ##args); \
        } while (0)

/*
 *  Offsets for UART registers 
*/
#define DBGU_CR       0x0000	  /* Control Register */ 
#define DBGU_MR       0x0004	  /* Mode Register*/
#define DBGU_IER      0x0008	  /* Interrupt Enable Register*/
#define DBGU_IDR      0x000C	  /* Interrupt Disable Register*/
#define DBGU_IMR      0x0010    /* Interrupt Mask Register*/
#define DBGU_SR      0x0014    /* Channel Status Register*/
#define DBGU_RHR      0x0018    /* Receiver Holding Register*/ 
#define DBGU_THR      0x001C    /* TransmitterHolding Register*/
#define DBGU_BRGR     0x0020    /* Baud Rate Generator Register*/ 
/*0x24 ~ 0x3c Reserved*/
#define DBGU_CIDR     0x0040    /* FI DI Ratio Register*/ 
#define DBGU_EXID      0x0044    /* Number of Errors Register*/ 
/* 0x4c ~ 0xfc Reserved*/
/* 0x100 ~ 0x124 Reserved for PDC Registers*/
#define DBGU_MAX      0x0124

#define AT91SAM9260_DEBUG_REGS_MEM_SIZE    0x128

typedef struct at91sam9260_debug_regs {
	uint32_t dbgu_cr;
	uint32_t dbgu_mr;
	uint32_t dbgu_ier;
	uint32_t dbgu_idr;
	uint32_t dbgu_imr;
	uint32_t dbgu_sr;
	uint32_t dbgu_rhr;
	uint32_t dbgu_thr;
	uint32_t dbgu_brgr;
	uint32_t dbgu_cidr;
	uint32_t dbgu_exid;
} at91sam9260_debug_regs;


/*DBGU_CR register fields*/
#define CR_RSTRX	(1 << 2) /*reset receiver*/
#define CR_RSTTX    (1 << 3) /*reset transmitter*/
#define CR_RXEN     (1 << 4) /*receiver enable*/
#define CR_RXDIS    (1 << 5) /*receiver disable*/
#define CR_TXEN		(1 << 6) /*transmitter enable*/
#define CR_TXDIS    (1 << 7) /*transmitter disable*/
#define CR_RSTSTA   (1 << 8) /*reset status bits*/

/*DBGU_MR register fields*/
#define MR_MODE_SHIFT   14 

/*debug interrupt enable register fields*/
#define IER_RXRDY  (1 << 0)
#define IER_TXRDY  (1 << 1)
#define IER_ENDRX  (1 << 3)
#define IER_ENDTx  (1 << 4)
#define IER_OVER   (1 << 5)
#define IER_FRAME  (1 << 6)
#define IER_PARE   (1 << 7)
#define IER_TXEMPTY (1 << 9)
#define IER_TXBUFE (1 << 11)
#define IER_RXBUFE (1 << 12)

/*debug status register fields*/
#define SR_RXRDY  (1 << 0)
#define SR_TXRDY  (1 << 1)
#define SR_ENDRX  (1 << 3)
#define SR_ENDTX  (1 << 4)
#define SR_OVER   (1 << 5)
#define SR_FRAME  (1 << 6)
#define SR_PARE   (1 << 7)
#define SR_TXEMPTY (1 << 9)
#define SR_TXBUFE    (1 << 11)
#define SR_RXBUFE    (1 << 12)

/*debug character length*/
enum debug_character {
	CHAR_5BITS = 5,
	CHAR_6BITS,
	CHAR_7BITS,
	CHAR_8BITS,
	CHAR_9BITS,
};

enum debug_stopbits {
	STOPBIT_1 = 1,
	STOPBIT_2,
	STOPBIT_RESERVVED,
};

enum debug_parity {
	PAR_EVEN = 0,
	PAR_ODD,
};

typedef struct at91sam9260_debug {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
	at91sam9260_debug_regs regs;
    CharDriverState  *chr;
    qemu_irq irq;
} at91sam9260_debug;

#define TYPE_AT91SAM9260_DEBUG "at91sam9260_debug"

#define AT91SAM9260_DEBUG(obj) \
    OBJECT_CHECK(at91sam9260_debug, (obj), "at91sam9260_debug")

static int at91samdebug_irq_pending(at91sam9260_debug *s)
{
	return 0;
}

static void at91sam9260debug_irq(at91sam9260_debug *s)
{
    if (at91samdebug_irq_pending(s)) 
        qemu_irq_raise(s->irq);
     else 
        qemu_irq_lower(s->irq);
}

static void at91sam9260debug_set_parameters(at91sam9260_debug *s)
{
    QEMUSerialSetParams ssp;
	int speed, parity, data_bits, stop_bits;

	speed = 9600;
	parity = PAR_EVEN;
	data_bits = CHAR_8BITS;

	stop_bits = STOPBIT_1;	
    ssp.speed = speed;
    ssp.parity = parity;
    ssp.data_bits = data_bits;
    ssp.stop_bits = stop_bits;

    qemu_chr_fe_ioctl(s->chr, CHR_IOCTL_SERIAL_SET_PARAMS, &ssp);

    PRINT_DEBUG("UART%d: speed: %d, parity: %c, data: %d, stop: %d\n",
                s->channel, ssp.speed, ssp.parity, ssp.data_bits, ssp.stop_bits);
}

static int is_at91sam9260_recv_disable(at91sam9260_debug *s)
{
	return s->regs.dbgu_sr & CR_RXDIS || !s->regs.dbgu_brgr;
}

static int is_at91sam9260_send_disable(at91sam9260_debug *s)
{
	return s->regs.dbgu_sr & CR_TXDIS ||!s->regs.dbgu_brgr;
}

static void at91sam9260debug_write(void *opaque, hwaddr offset,
                               uint64_t val, unsigned size)
{
    at91sam9260_debug *s = (at91sam9260_debug *)opaque;

	if (size > sizeof(uint32_t))
		return ;

	if (offset > DBGU_MAX)
		return ;

    switch (offset) {
	case DBGU_CR:
		if (val & CR_RSTSTA) 
			s->regs.dbgu_sr &= ~(SR_PARE | SR_OVER); 

		if (val & CR_RXEN){
			if (s->regs.dbgu_cr & CR_RXDIS || val & CR_RXDIS) {
				s->regs.dbgu_cr &= ~(CR_RXEN);
				s->regs.dbgu_sr &= ~SR_RXRDY;
				val &= ~(CR_RXEN);
				val |= CR_RXDIS;
			}
			s->regs.dbgu_rhr = 0;
		}
		if (val & CR_TXEN){
			if (s->regs.dbgu_cr & CR_TXDIS || val & CR_TXDIS) {
				s->regs.dbgu_cr &= ~(CR_TXEN);
				s->regs.dbgu_sr &= ~SR_TXRDY;
				val &= ~(CR_TXEN);
				val |= CR_TXDIS;
			}
			s->regs.dbgu_thr = 0;
			s->regs.dbgu_sr |= SR_TXRDY;
		}
		if (val & CR_RSTRX) {
			s->regs.dbgu_cr |= CR_RXDIS;
			s->regs.dbgu_rhr = 0;
		}
		if (val & CR_RSTTX) {
			s->regs.dbgu_cr |= CR_TXDIS;
			s->regs.dbgu_thr = 0;
		}

		s->regs.dbgu_cr |= val;
		break;
	case DBGU_MR:
		s->regs.dbgu_mr |= val;
		at91sam9260debug_set_parameters(s);
		break;
	case DBGU_IER:
		s->regs.dbgu_ier |= val;
		break;
	case DBGU_IDR:
		s->regs.dbgu_idr |= val;
		s->regs.dbgu_imr |= s->regs.dbgu_idr;
		break;
	case DBGU_THR:  
		if (is_at91sam9260_send_disable(s))
			break;
		s->regs.dbgu_thr = val & 0xff;
		s->regs.dbgu_sr &= ~SR_TXRDY;
        if (s->chr)
            qemu_chr_fe_write(s->chr,(const uint8_t *)&s->regs.dbgu_thr, 1);
        s->regs.dbgu_sr |= SR_TXRDY;
		break;
	case DBGU_BRGR:
		s->regs.dbgu_brgr = val;
		break;
    default:
        break;
    }
}

static uint64_t at91sam9260debug_read(void *opaque, hwaddr offset,
                                  unsigned size)
{
	uint64_t ret = 0;
    at91sam9260_debug *s = (at91sam9260_debug *)opaque;

	if (size > sizeof(uint32_t))
		return 0;

	if (offset > DBGU_MAX)
		return 0;

    switch (offset) {
	case DBGU_CR:
		ret = s->regs.dbgu_cr;
		break;
	case DBGU_MR:
		ret = s->regs.dbgu_mr;
		break;
	case DBGU_IMR:
		ret = s->regs.dbgu_imr;
		break;
	case DBGU_SR:
		ret = s->regs.dbgu_sr;
		break;
	case DBGU_RHR:
		if (is_at91sam9260_recv_disable(s))
			break;
		ret = s->regs.dbgu_rhr;
		s->regs.dbgu_rhr = 0;
		s->regs.dbgu_sr &= ~SR_RXRDY;
        if (s->chr)
            qemu_chr_accept_input(s->chr);
        //s->regs.dbgu_sr |= SR_RXRDY;
		break;
	case DBGU_BRGR:
		ret = s->regs.dbgu_brgr;
		break;
    default:
		break;
    }
    return ret;
}

static const MemoryRegionOps at91sam9260debug_ops = {
    .read = at91sam9260debug_read,
    .write = at91sam9260debug_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .max_access_size = 4,
        .unaligned = false
    },
};

static void at91sam9260debug_receive(void *opaque, const uint8_t *buf, int size)
{
    at91sam9260_debug *s = (at91sam9260_debug *)opaque;

	if (!s->regs.dbgu_rhr && !(s->regs.dbgu_sr & SR_RXRDY)) {
		s->regs.dbgu_rhr = *((uint8_t*)buf);
		s->regs.dbgu_sr |= SR_RXRDY;
	} else {
		if (s->regs.dbgu_sr & SR_RXRDY)
			s->regs.dbgu_sr |= SR_OVER;
	}
    at91sam9260debug_irq(s);
}

static int at91sam9260debug_can_receive(void *opaque)
{
	at91sam9260_debug *s = (at91sam9260_debug *)opaque;
	if (is_at91sam9260_recv_disable(s))
		return 0;
	return 1;
}

/**
 * at91sam9260debug_event:handle debug uart event
 */
static void at91sam9260debug_event(void *opaque, int event)
{
	at91sam9260_debug *s = (at91sam9260_debug *)opaque;

	switch (event) {
	case CHR_EVENT_BREAK:
	case CHR_EVENT_FOCUS:
		memset(&s->regs, 0, sizeof(at91sam9260_debug_regs));
		break;
	default:
		break;
	}
}

static void at91sam9260debug_reset(DeviceState *dev)
{
    at91sam9260_debug *s = AT91SAM9260_DEBUG(dev);
	memset(&s->regs, 0, sizeof(s->regs));
}

static const VMStateDescription vmstate_at91sam9260debug_regs = {
    .name = "at91sam9260_debug",
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(dbgu_cr, at91sam9260_debug_regs),
        VMSTATE_UINT32(dbgu_mr, at91sam9260_debug_regs),
        VMSTATE_UINT32(dbgu_ier, at91sam9260_debug_regs),
        VMSTATE_UINT32(dbgu_idr, at91sam9260_debug_regs),
        VMSTATE_UINT32(dbgu_imr, at91sam9260_debug_regs),
        VMSTATE_UINT32(dbgu_sr, at91sam9260_debug_regs),
        VMSTATE_UINT32(dbgu_rhr, at91sam9260_debug_regs),
        VMSTATE_UINT32(dbgu_thr, at91sam9260_debug_regs),
        VMSTATE_UINT32(dbgu_brgr, at91sam9260_debug_regs),
        VMSTATE_END_OF_LIST()
    }
};
static const VMStateDescription vmstate_at91sam9260debug = { .name = "at91sam9260_debug",
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .fields = (VMStateField[]) {
        VMSTATE_STRUCT(regs, at91sam9260_debug, 1,
                          vmstate_at91sam9260debug_regs, at91sam9260_debug_regs),
        VMSTATE_END_OF_LIST()
    }
};

static void at91sam9260debug_realize(DeviceState *dev, Error **errp)
{
	at91sam9260_debug *s = AT91SAM9260_DEBUG(dev);

    s->chr = qemu_char_get_next_serial();

    if (s->chr) {
    	qemu_chr_add_handlers(s->chr, at91sam9260debug_can_receive,
    			at91sam9260debug_receive, at91sam9260debug_event, s);
    }
}

static void at91sam9260debug_init(Object *obj)
{
	SysBusDevice *dev = SYS_BUS_DEVICE(obj);
    at91sam9260_debug *s = AT91SAM9260_DEBUG(dev);

    /* memory mapping */
    memory_region_init_io(&s->iomem, OBJECT(s), &at91sam9260debug_ops, s,
                          "at91sam9260_debug", AT91SAM9260_DEBUG_REGS_MEM_SIZE);
    sysbus_init_mmio(dev, &s->iomem);
	/*Todo:需要加入中断模式的支持*/

   // sysbus_init_irq(dev, &s->irq);

	memset(&s->regs, 0, sizeof(at91sam9260_debug_regs));
}

static void at91sam9260debug_class_init(ObjectClass *klass, void *data)
{

    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->reset = at91sam9260debug_reset;
    dc->realize = at91sam9260debug_realize;
    dc->vmsd = &vmstate_at91sam9260debug;
}

static const TypeInfo at91sam9260debug_info = {
    .name          = TYPE_AT91SAM9260_DEBUG,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(at91sam9260_debug),
	.instance_init = at91sam9260debug_init,
    .class_init    = at91sam9260debug_class_init,
};

static void at91sam9260debug_register(void)
{
    type_register_static(&at91sam9260debug_info);
}

type_init(at91sam9260debug_register)
