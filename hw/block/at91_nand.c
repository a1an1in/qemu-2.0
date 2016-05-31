#include "hw/hw.h"
#include "sysemu/sysemu.h"
#include "hw/sysbus.h"
#include "hw/devices.h"
#include "hw/block/flash.h"
#include "sysemu/blockdev.h"

typedef struct at91_nand {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    DeviceState *nand;
    uint8_t ctl;
    uint8_t manf_id;
    uint8_t chip_id;
    ECCState ecc;

}at91_nand;

#define TYPE_AT91_NAND "at91_nand"
#define AT91_NAND(obj) OBJECT_CHECK(at91_nand, (obj), TYPE_AT91_NAND)

#define AT91_NAND_SIZE 0x10000000

/*nand pin signal*/
#define CLE_EN 	1
#define CLE_DIS 0
#define ALE_EN  1
#define ALE_DIS 0
#define CE_EN   1
#define CE_DIS  0
#define WP_EN   1
#define WP_DIS  0
#define GND_EN  1
#define GND_DIS 0

static uint64_t at91_read(void *opaque, hwaddr addr, unsigned size)
{
	at91_nand *s = (at91_nand *)opaque;

	nand_setpins(s->nand, CLE_EN, ALE_EN, CE_EN, WP_EN, GND_EN);
	return nand_getio(s->nand);;
}

static void at91_write(void *opaque, hwaddr addr,
                     uint64_t value, unsigned size)
{
	at91_nand *s = (at91_nand *)opaque;

	nand_setpins(s->nand, CLE_EN, ALE_EN, CE_EN, WP_EN, GND_EN);
	nand_setio(s->nand, value & 0xFF);
}

static const MemoryRegionOps at91_ops = {
    .read = at91_read,
    .write = at91_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static int at91_nand_init(SysBusDevice *dev)
{
    at91_nand *s = AT91_NAND(dev);
    DriveInfo *nand;

    s->ctl = 0;
    nand = drive_get(IF_MTD, 0, 0);
    s->nand = nand_init(nand ? nand->bdrv : NULL, s->manf_id, s->chip_id);

    memory_region_init_io(&s->iomem, OBJECT(s), &at91_ops, s, "at91_nand", AT91_NAND_SIZE);
    sysbus_init_mmio(dev, &s->iomem);

    return 0;
}

static VMStateDescription vmstate_at91_nand_info = {
    .name = "at91_nand",
    .version_id = 0,
    .minimum_version_id = 0,
    .minimum_version_id_old = 0,
    .fields = (VMStateField []) {
        VMSTATE_UINT8(ctl, at91_nand),
        VMSTATE_STRUCT(ecc, at91_nand, 0, vmstate_ecc_state, ECCState),
        VMSTATE_END_OF_LIST(),
    },
};

static Property at91_nand_properties[] = {
    DEFINE_PROP_UINT8("manf_id", at91_nand, manf_id, NAND_MFR_SAMSUNG),
    DEFINE_PROP_UINT8("chip_id", at91_nand, chip_id, 0xf1),
    DEFINE_PROP_END_OF_LIST(),
};

static void at91_nand_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    k->init = at91_nand_init;
    dc->vmsd = &vmstate_at91_nand_info;
    dc->props = at91_nand_properties;
}

static const TypeInfo at91_nand_info = {
    .name          = TYPE_AT91_NAND,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(at91_nand),
    .class_init    = at91_nand_class_init,
};

static void at91sam9260nand_register(void)
{
    type_register_static(&at91_nand_info);
}

type_init(at91sam9260nand_register)
