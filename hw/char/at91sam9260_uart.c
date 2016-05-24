/*
 *  AT91SAM9260 UART Emulation
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
#define US_CR       0x0000	  /* Control Register */ 
#define US_MR       0x0004	  /* Mode Register*/
#define US_IER      0x0008	  /* Interrupt Enable Register*/
#define US_IDR      0x000C	  /* Interrupt Disable Register*/
#define US_IMR      0x0010    /* Interrupt Mask Register*/
#define US_CSR      0x0014    /* Channel Status Register*/
#define US_RHR      0x0018    /* Receiver Holding Register*/ 
#define US_THR      0x001C    /* TransmitterHolding Register*/
#define US_BRGR     0x0020    /* Baud Rate Generator Register*/ 
#define US_RTOR     0x0024    /* Receiver Time-out Register*/
#define US_TTGR     0x0028    /* Transmitter Timeguard Register*/ 
/*0x20 ~ 0x3c Reserved*/
#define US_FIDI     0x0040    /* FI DI Ratio Register*/ 
#define US_NER      0x0044    /* Number of Errors Register*/ 
#define US_RESVD    0x0048    /* Reserved*/ 
#define US_IF       0x004c    /* IrDA Filter Register*/ 
/* 0x5c ~ 0xfc Reserved*/
/* 0x100 ~ 0x128 Reserved for PDC Registers*/

#define REG(reg) (reg / sizeof(uint32_t))

typedef struct at91samUartReg {
    const char         *name; /* the only reason is the debug output */
    hwaddr  offset;
    uint32_t            reset_value;
} at91samUartReg;

/*FIXME:just do normal uart function*/
static at91samUartReg at91sam9260uart_regs[] = {
    {"US_CR",    US_CR,    0x00000000},
    {"US_MR",    US_MR,     0x00000000},
    {"US_IER",   US_IER,    0x00000000},
    {"US_IDR",   US_IDR,    0x00000000},
    {"US_IMR",   US_IMR,  0x00000000}, 
    {"US_CSR",   US_CSR,  0x00000000}, 
    {"US_RHR",   US_RHR,   0x00000000}, 
    {"US_THR",   US_THR,   0x00000000}, 
    {"US_BRGR",  US_BRGR,     0x00000000}, 
    {"US_RTOR",  US_RTOR,     0x00000000}, 
    {"US_TTGR",  US_TTGR,   0x00000000},
};

#define AT91SAM9260_UART_REGS_MEM_SIZE    0x128

/*US_CR register fields*/
#define CR_RSTRX	(1 << 2) /*reset receiver*/
#define CR_RSTTX    (1 << 3) /*reset transmitter*/
#define CR_RXEN     (1 << 4) /*receiver enable*/
#define CR_RXDIS    (1 << 5) /*receiver disable*/
#define CR_TXEN		(1 << 6) /*transmitter enable*/
#define CR_TXDIS    (1 << 7) /*transmitter disable*/
#define CR_RSTSTA   (1 << 8) /*reset status bits*/
#define CR_STTBRK   (1 << 9) /*start break*/
#define CR_STPBRK   (1 << 10) /*stop break*/
#define CR_STTTO    (1 << 11) /*start timeout*/
#define CR_SENDA    (1 << 12) /*send address*/
#define CR_RSTIT    (1 << 13) /*reset iterations*/
#define CR_RSTNACK  (1 << 14) /*reset no ack*/
#define CR_RETTO    (1 << 15) /*rearm timeout*/
#define CR_DTREN    (1 << 16) /*data terminal ready enable*/
#define CR_DTRDIS   (1 << 17) /*data terminal ready disable*/
#define CR_RTSEN    (1 << 18) /*request to send enable*/
#define CR_RTSDIS   (1 << 19) /*request to send disable*/

/*US_MR register fields*/
#define MR_ASYNC    (0 << 8)
#define MR_SYNC     (1 << 8)
#define MR_EVEN_PAR (0 << 9)
#define MR_ODD_PAR  (1 << 9)
#define MR_MSBF_LSB (0 << 16)
#define MR_MSBF_MSB (1 << 16)
#define MR_MODE9_CHRL (0 << 17)
#define MR_MODE9_LENGTH (1 << 17)
#define MR_CKLO_NSCK  (0 << 18)
#define MR_CKLO_SCK  (1 << 18)
#define MR_OVER_16   (0 << 19)
#define MR_OVER_8    (1 << 19)

/*usart interrupt enable register fields*/
#define IER_RXRDY  (1 << 0)
#define IER_TXRDY  (1 << 1)
#define IER_RXBRK  (1 << 2)
#define IER_ENDRX  (1 << 3)
#define IER_ENDTx  (1 << 4)
#define IER_OVER   (1 << 5)
#define IER_FRAME  (1 << 6)
#define IER_PARE   (1 << 7)
#define IER_TIMEOUT (1 << 8)
#define IER_TXEMPTY (1 << 9)
#define IER_ITERATION (1 << 10)
#define IER_TXBUFE (1 << 11)
#define IER_RXBUFE (1 << 12)
#define IER_NACK (1 << 13)
#define IER_RIIC (1 << 14)
#define IER_DSRIC (1 << 15)
#define IER_DCDIC (1 << 16)
#define IER_CTSIC (1 << 17)

/*usart mode*/
enum usart_mode {
	USART_MODE_NORMAL = 0, 
	USART_MODE_RS485,  
	USART_MODE_HANDSHAK, 
	USART_MODE_MODEM,
	USART_MODE_ISO7816T0,
	USART_MODE_RESERVED5,
	USART_MODE_ISO7816T1,
	USART_MODE_RESERVED6,
	USART_MODE_IRDA,
	USART_MODE_END,
};

/*usart clock selection*/
enum usart_clk {
	SCLK_MCK = 0,
	SCLK_MCK_DIV,
	SCLK_RESERVED,
	SCLK_SCK,
};

/*usart character length*/
enum usart_character {
	CHAR_5BITS = 0,
	CHAR_6BITS,
	CHAR_7BITS,
	CHAR_8BITS,
};

enum usart_stopbits {
	STOPBIT_1 = 0,
	STOPBIT_1_5,
	STOPBIT_2,
	STOPBIT_RESERVVED,
};

enum usart_channel_mode {
	CHMODE_NORMAL = 0,
	CHMODE_AUTO_ECHO,
	CHMODE_LOCAL_LOOPBACk,
	CHMODE_REMOTE_LOOPBACK,
};


#define TYPE_AT91SAM9260_UART "at91sam9260.uart"

#define AT91SAM9260_UART(obj) \
    OBJECT_CHECK(AT91sam9260Uart, (obj), TYPE_AT91SAM9260_UART)

typedef struct AT91sam9260Uart {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    uint32_t reg[AT91SAM9260_UART_REGS_MEM_SIZE / sizeof(uint32_t)];
    CharDriverState  *chr;
    qemu_irq irq;
    uint32_t channel;
} AT91sam9260Uart;

static void at91_writel(AT91sam9260Uart *s, uint32_t reg, uint32_t val)
{
	s->reg[REG(reg)] = val;
}

static uint32_t at91_readl(AT91sam9260Uart *s, uint32_t reg)
{
	return s->reg[REG(reg)]; 
}

static void at91sam9260uart_irq(AT91sam9260Uart *s)
{

    if (at91_readl(s, US_CR)) {
        qemu_irq_raise(s->irq);
    } else {
        qemu_irq_lower(s->irq);
    }
}

static void at91sam9260uart_set_parameters(AT91sam9260Uart *s)
{
    QEMUSerialSetParams ssp;
    ssp.speed     = 0;
    ssp.parity    = 0;
    ssp.data_bits = 0;
    ssp.stop_bits = 0;

    qemu_chr_fe_ioctl(s->chr, CHR_IOCTL_SERIAL_SET_PARAMS, &ssp);

    PRINT_DEBUG("UART%d: speed: %d, parity: %c, data: %d, stop: %d\n",
                s->channel, ssp.speed, ssp.parity, ssp.data_bits, ssp.stop_bits);
}

static void at91sam9260uart_write(void *opaque, hwaddr offset,
                               uint64_t val, unsigned size)
{
    AT91sam9260Uart *s = (AT91sam9260Uart *)opaque;

    switch (offset) {
	case US_CR:
		at91sam9260uart_set_parameters(s);
		break;
    default:
        at91_writel(s, offset, val);
        break;
    }
}

static uint64_t at91sam9260uart_read(void *opaque, hwaddr offset,
                                  unsigned size)
{
    AT91sam9260Uart *s = (AT91sam9260Uart *)opaque;

    switch (offset) {
	case US_CR:
		at91sam9260uart_irq(s);
		break;
    default:
        return at91_readl(s, offset);
    }

    return 0;
}

static const MemoryRegionOps at91sam9260uart_ops = {
    .read = at91sam9260uart_read,
    .write = at91sam9260uart_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .max_access_size = 4,
        .unaligned = false
    },
};

static void at91sam9260uart_receive(void *opaque, const uint8_t *buf, int size)
{
    AT91sam9260Uart *s = (AT91sam9260Uart *)opaque;
    at91sam9260uart_irq(s);
}

static int at91sam9260uart_can_receive(void *opaque)
{
	return 0;
}

/**
 * at91sam9260uart_event:串口发生的事件处理
 */
static void at91sam9260uart_event(void *opaque, int event)
{

}

static void at91sam9260uart_reset(DeviceState *dev)
{
    AT91sam9260Uart *s = AT91SAM9260_UART(dev);
    int i;

    for (i = 0; i < ARRAY_SIZE(at91sam9260uart_regs); i++) {
        s->reg[REG(at91sam9260uart_regs[i].offset)] =
                at91sam9260uart_regs[i].reset_value;
    }
    PRINT_DEBUG("UART%d: Rx FIFO size: %d\n", s->channel, s->rx.size);
}

static const VMStateDescription vmstate_at91sam9260uart = {
    .name = "at91sam9260.uart",
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(reg, AT91sam9260Uart,
                             AT91SAM9260_UART_REGS_MEM_SIZE / sizeof(uint32_t)),
        VMSTATE_END_OF_LIST()
    }
};

/**
 * at91sam9260uart_create:创建串口设备接口
 */
DeviceState *at91sam9260uart_create(hwaddr addr,
                                    int fifo_size,
                                    int channel,
                                    CharDriverState *chr,
                                    qemu_irq irq)
{
    DeviceState  *dev;
    SysBusDevice *bus;

    const char chr_name[] = "serial";
    char label[ARRAY_SIZE(chr_name) + 1];

    dev = qdev_create(NULL, TYPE_AT91SAM9260_UART);

    if (!chr) {
        if (channel >= MAX_SERIAL_PORTS) {
            hw_error("Only %d serial ports are supported by QEMU.\n",
                     MAX_SERIAL_PORTS);
        }
        chr = serial_hds[channel];
        if (!chr) {
            snprintf(label, ARRAY_SIZE(label), "%s%d", chr_name, channel);
            chr = qemu_chr_new(label, "null", NULL);
            if (!(chr)) {
                hw_error("Can't assign serial port to UART%d.\n", channel);
            }
        }
    }

    qdev_prop_set_chr(dev, "chardev", chr);
    qdev_prop_set_uint32(dev, "channel", channel);
    qdev_prop_set_uint32(dev, "rx-size", fifo_size);
    qdev_prop_set_uint32(dev, "tx-size", fifo_size);

    bus = SYS_BUS_DEVICE(dev);
    qdev_init_nofail(dev);
    if (addr != (hwaddr)-1) {
        sysbus_mmio_map(bus, 0, addr);
    }
    sysbus_connect_irq(bus, 0, irq);

    return dev;
}

static int at91sam9260uart_init(SysBusDevice *dev)
{
    AT91sam9260Uart *s = AT91SAM9260_UART(dev);

    /* memory mapping */
    memory_region_init_io(&s->iomem, OBJECT(s), &at91sam9260uart_ops, s,
                          "at91sam9260.uart", AT91SAM9260_UART_REGS_MEM_SIZE);
    sysbus_init_mmio(dev, &s->iomem);

    sysbus_init_irq(dev, &s->irq);

    qemu_chr_add_handlers(s->chr, at91sam9260uart_can_receive,
                          at91sam9260uart_receive, at91sam9260uart_event, s);

    return 0;
}

static Property at91sam9260uart_properties[] = {
    DEFINE_PROP_CHR("chardev", AT91sam9260Uart, chr),
    DEFINE_PROP_UINT32("channel", AT91sam9260Uart, channel, 0),
    DEFINE_PROP_END_OF_LIST(),
};

static void at91sam9260uart_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    k->init = at91sam9260uart_init;
    dc->reset = at91sam9260uart_reset;
    dc->props = at91sam9260uart_properties;
    dc->vmsd = &vmstate_at91sam9260uart;
}

static const TypeInfo at91sam9260uart_info = {
    .name          = TYPE_AT91SAM9260_UART,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AT91sam9260Uart),
    .class_init    = at91sam9260uart_class_init,
};

static void at91sam9260uart_register(void)
{
    type_register_static(&at91sam9260uart_info);
}

type_init(at91sam9260uart_register)
