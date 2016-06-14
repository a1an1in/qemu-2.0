/*
 * AT91SAM9260ek Soc board watchdog simulation.
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
#include "qemu/timer.h"
#include "hw/sysbus.h"
#include "sysemu/sysemu.h"
#include "sysemu/watchdog.h"
#include "hw/hw.h"

/*#define AT91_DEBUG 1*/

#ifdef AT91_DEBUG
#define at91_debug(fs,...)					\
    fprintf(stderr,"at91: %s: "fs,__func__,##__VA_ARGS__)
#else
#define at91_debug(fs,...)
#endif

#define TYPE_WDT_AT91 "wdt_at91"
#define WDT_AT91(obj) OBJECT_CHECK(AT91WDTState, (obj), TYPE_WDT_AT91)

/*watch dog register offset define*/
#define WDT_CR		0x00
#define WDT_MR		0x04
#define WDT_SR		0x08
#define WDT_REGS	3
#define WDT_AT91_REGS_SIZE (WDT_REGS * sizeof(uint32_t))

#define WDT_KEY 0xA5

/*wdt mr register field define*/
#define MR_WDDIS  (1 << 15)

typedef struct AT91WDTstate {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
	uint32_t regs[WDT_REGS];
    QEMUTimer *timer;
} AT91WDTState;


static uint32_t readl_reg(AT91WDTState *s, hwaddr offset)
{
	return s->regs[offset];	
}

static void writel_reg(AT91WDTState *s, hwaddr offset, uint32_t value)
{
	s->regs[offset] = value;	
}

/* This is called when the watchdog expires. */
static void at91_timer_expired(void *vp)
{
    AT91WDTState *s = vp;

    at91_debug("watchdog expired\n");
	writel_reg(s, WDT_SR, 1);
    watchdog_perform_action();
    timer_del(s->timer);
}


static void at91_update_timer(AT91WDTState *s, int wdv)
{
    int64_t timeout;
    timeout = (int64_t) wdv * get_ticks_per_sec();
    timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + timeout);
}

static void wdt_at91_update(AT91WDTState *s)
{
	uint32_t wdt_mr = readl_reg(s, WDT_MR);

	if (wdt_mr & MR_WDDIS) 
		timer_del(s->timer);
	else {
		int wdv = wdt_mr & 0xFFF;
		if (wdv)
			at91_update_timer(s, wdv);
	}
}

static void wdt_at91_restart(AT91WDTState *s)
{
    int64_t timeout, wdv = readl_reg(s, WDT_MR) & 0xFFF;
    timeout = (int64_t) wdv * get_ticks_per_sec();
    timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + timeout);
	writel_reg(s, WDT_SR, 0);
    at91_debug("watchdog restart\n");
}

static uint64_t wdt_at91_read(void *opaque, hwaddr offset, unsigned size)
{
	uint64_t  ret = 0;
	AT91WDTState *s = (AT91WDTState *)opaque;

	switch(offset & 0xFF) {
	case WDT_SR:
		ret = readl_reg(s, WDT_SR);
		break;
	case WDT_MR:
		ret = readl_reg(s, WDT_MR);
		break;
	default:
		ret = 0;
		break;

	}
	return ret;
}

static void wdt_at91_write(void *opaque, hwaddr offset, uint64_t val, 
		unsigned size)
{
	uint32_t key = WDT_KEY << 24;
	AT91WDTState *s = (AT91WDTState *)opaque;
	uint32_t wdd = (readl_reg(s, WDT_MR) >> 16)&0xFFF;
	uint32_t wdv = readl_reg(s, WDT_MR) &0xFFF;

	switch(offset &0xFF) {
	case WDT_CR:
		if (key == (val & 0xFF000000)) {
			writel_reg(s, WDT_CR, val&1);		
			if ((val & 1) && wdd >= wdv) 
				wdt_at91_restart(s);
			else if ((val &1) && wdd < wdv)
				writel_reg(s, WDT_SR, 1<< 2);
		}
		break;
	case WDT_MR:
		writel_reg(s, WDT_MR, val);
		wdt_at91_update(s);
		break;
	default:
		break;
	}
}

static const MemoryRegionOps at91wdt_ops = {
    .read = wdt_at91_read,
    .write = wdt_at91_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .max_access_size = 4,
        .unaligned = false
    },
};

static void wdt_at91_init(Object *obj)
{
	SysBusDevice *dev = SYS_BUS_DEVICE(obj);
    AT91WDTState *s = WDT_AT91(dev);
    /* memory mapping */
    memory_region_init_io(&s->iomem, OBJECT(s), &at91wdt_ops, s,
                          "wdt_at91", WDT_AT91_REGS_SIZE);
    sysbus_init_mmio(dev, &s->iomem);
	/*Todo:需要加入中断模式的支持*/

   // sysbus_init_irq(dev, &s->irq);
	memset(&s->regs, 0, WDT_AT91_REGS_SIZE);
	writel_reg(s, WDT_MR, 0xFFF);
    at91_debug("watchdog init\n");
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, at91_timer_expired, s);
    //timer_del(s->timer);
}

static void wdt_at91_reset(DeviceState *dev)
{
    AT91WDTState *s = WDT_AT91(dev);
	memset(&s->regs, 0, WDT_AT91_REGS_SIZE);
	writel_reg(s, WDT_MR, 0xFFF);
    at91_debug("watchdog reset\n");
   // timer_del(s->timer);
}

static WatchdogTimerModel model = {
    .wdt_name = "wdt_at91",
    .wdt_description = "AT91SAM9260EK WDT",
};

static const VMStateDescription vmstate_wdt_at91 = {
    .name = "wdt_at91",
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .fields = (VMStateField[]) {
		VMSTATE_UINT32_ARRAY(regs, AT91WDTState, WDT_REGS),
        VMSTATE_END_OF_LIST()
    }
};

static void wdt_at91_class_init(ObjectClass *klass, void *data)
{

    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->reset = wdt_at91_reset;
   // dc->realize = wdt_at91_realize;
    dc->vmsd = &vmstate_wdt_at91;
}

static const TypeInfo wdt_at91_info = {
    .name          = TYPE_WDT_AT91,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AT91WDTState),
	.instance_init = wdt_at91_init,
    .class_init    = wdt_at91_class_init,
};

static void wdt_at91_register(void)
{
    watchdog_add_model(&model);
    type_register_static(&wdt_at91_info);
}

type_init(wdt_at91_register)
