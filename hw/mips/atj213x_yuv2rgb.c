#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu-common.h"
#include "hw/sysbus.h"
#include "exec/log.h"
#include "ui/console.h"
#include "ui/pixel_ops.h"
#include "trace.h"

#include "atj213x_yuv2rgb.h"

static uint64_t YUV2RGB_read(void *opaque, hwaddr  addr, unsigned size)
{
    AtjYUV2RGBState *s = opaque;

    addr >>= 2;
    switch (addr)
    {
        case YUV2RGB_CTL:
            /* pretend fifo is always empty */
            trace_atj_yuv2rgb_read(addr<<2, s->regs[addr] | 0x04);

            return s->regs[addr] | 0x04;

        default:
            break;
    }

    trace_atj_yuv2rgb_read(addr<<2, s->regs[addr]);
    return s->regs[addr];
}

static void *vramptr(void *opaque, uint16_t row, uint16_t col)
{
    AtjYUV2RGBState *s = opaque;
    void *vram = memory_region_get_ram_ptr(&s->vram);

    return vram + 2 * ((row * LCD_WIDTH) + col);
}

/* writes to LCM fifo are 32bit actually - 2 pixels at a time */
static void vram_write_windowed(void *opaque, uint32_t value)
{
    AtjYUV2RGBState *s = opaque;
    uint32_t *dst = vramptr(opaque, s->row, s->col);
    *dst = value;

    s->col += 2;
    if (s->col > s->col_end)
    {
        s->col = s->col_start;
        s->row++;
        if (s->row > s->row_end)
        {
            s->row = s->row_start;
        }
    }
}

static void vram_window_reset(void *opaque)
{
    AtjYUV2RGBState *s = opaque;
    s->col = s->col_start;
    s->row = s->row_start;
}

static void YUV2RGB_write(void *opaque, hwaddr addr, uint64_t value, unsigned size)
{
    AtjYUV2RGBState *s = opaque;
    trace_atj_yuv2rgb_write(addr, value);

    addr >>= 2;
    switch (addr)
    {
        case YUV2RGB_CTL:
            s->mode = (value >> 16) & 0x03;
            break;

        case YUV2RGB_DAT:
            switch (s->mode)
            {
                case LCM_CMD:
                    s->cmd = value;

                    if (s->cmd == 0x22)
                    {
                        /* set vram pointer at the beginning of the window */
			vram_window_reset(opaque);
                    }
                    break;

                case LCM_DATA:
                    switch (s->cmd)
                    {
                        case 0x02:
			    /* start column address high byte */
                            s->col_start = (s->col_start & 0x00ff) | ((uint8_t)value) << 8;
                            break;

                        case 0x03:
			    /* start column address low byte */
                            s->col_start = (s->col_start & 0xff00) | (uint8_t)value;
                            break;

                        case 0x04:
			    /* end column address high byte */
                            s->col_end = (s->col_end & 0x00ff) | ((uint8_t)value) << 8;
                            break;

                        case 0x05:
			    /* end column address low byte */
                            s->col_end = (s->col_end & 0xff00) | (uint8_t)value;
                            break;

                        case 0x06:
			    /* start row address high byte */
                            s->row_start = (s->row_start & 0x00ff) | ((uint8_t)value) << 8;
                            break;

                        case 0x07:
			    /* start row address low byte */
                            s->row_start = (s->row_start & 0xff00) | (uint8_t)value;
                            break;

                        case 0x08:
			    /* end row address high byte */
                            s->row_end = (s->row_end & 0x00ff) | ((uint8_t)value) << 8;
                            break;

                        case 0x09:
			    /* end row address low byte */
                            s->row_end = (s->row_end & 0xff00) | (uint8_t)value;
                            break;
                    }
                    break;

		case LCM_RGB:
                    if (s->cmd == 0x22)
                    {
                        vram_write_windowed(opaque, value);
                    }
                    break;

                case LCM_YUV:
                    break;
            }

        default:
            break;
    }

    s->regs[addr] = value;
};

static const MemoryRegionOps YUV2RGB_mmio_ops = {
    .read = YUV2RGB_read,
    .write = YUV2RGB_write,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void YUV2RGB_reset(DeviceState *d)
{
    AtjYUV2RGBState *s = ATJ213X_YUV2RGB(d);

    for (int i=0; i<YUV2RGB_REG_NUM; i++)
    {
        s->regs[i] = 0;
    }

    s->col_start = 0;
    s->col_end = LCD_WIDTH - 1;
    s->row_start = 0;
    s->row_end = LCD_HEIGHT - 1;
}

static void yuv2rgb_update_display(void *opaque)
{
    AtjYUV2RGBState *s = opaque;
    DisplaySurface *surface = qemu_console_surface(s->con);
    void *src = memory_region_get_ram_ptr(&s->vram);
    void *dst = surface_data(surface);

    for (int i=0; i<LCD_WIDTH*LCD_HEIGHT; i++)
    {
        /* scale pixel by backlight value */
        int backlight_val = s->pmu->backlight_get(s->pmu);
        unsigned int r = backlight_val * ((*(uint16_t *)src & 0xf800) >> 8)/31;
        unsigned int g = backlight_val * ((*(uint16_t *)src & 0x07e0) >> 3)/31;
        unsigned int b = backlight_val * ((*(uint16_t *)src & 0x001f) << 3)/31;
        src += 2;

        switch (surface_bits_per_pixel(surface))
        {
            case 8:
                *(uint8_t *)dst = rgb_to_pixel8(r, g, b);
                dst += 1;
                break;

            case 15:
                *(uint16_t *)dst = rgb_to_pixel15(r, g, b);
                dst += 2;
                break;

            case 16:
                *(uint16_t *)dst = rgb_to_pixel16(r, g, b);
                dst += 2;
                break;

            case 24:
                *(uint32_t *)dst = rgb_to_pixel24(r, g, b);
                dst += 3;
                break;

            case 32:
                *(uint32_t *)dst = rgb_to_pixel32(r, g, b);
                dst += 4;
        }
    }

    dpy_gfx_update(s->con, 0, 0, LCD_WIDTH, LCD_HEIGHT);
}

static const GraphicHwOps yuv2rgb_ops = {
    .gfx_update  = yuv2rgb_update_display,
};

static void YUV2RGB_init(Object *obj)
{
    AtjYUV2RGBState *s = ATJ213X_YUV2RGB(obj);
    SysBusDevice *dev = SYS_BUS_DEVICE(obj);

    object_property_add_link(OBJECT(s), "pmu", TYPE_ATJ213X_PMU,
                             (Object **)&s->pmu,
                             object_property_allow_set_link,
                             OBJ_PROP_LINK_UNREF_ON_RELEASE,
                             &error_abort);

    memory_region_init_io(&s->regs_region, obj, &YUV2RGB_mmio_ops, s, TYPE_ATJ213X_YUV2RGB, YUV2RGB_REG_NUM * 4);
    sysbus_init_mmio(dev, &s->regs_region);

    /* Declare video ram - it is inside LCD controller actually
     * but to make things simpler we treat atj213x + lcd controller as single entity
     */
    memory_region_init_ram(&s->vram, NULL, "atj.vram", LCD_WIDTH*LCD_HEIGHT*sizeof(uint16_t), &error_fatal);
    s->vramptr = memory_region_get_ram_ptr(&s->vram);

    s->con = graphic_console_init(NULL, 0, &yuv2rgb_ops, s);
    qemu_console_resize(s->con, LCD_WIDTH, LCD_HEIGHT);

    uint16_t *dst = (uint16_t *)s->vramptr;

    /* random pixels as in uninitialized real hardware */
    srand (time(NULL));
    for (int i=0; i<LCD_WIDTH*LCD_HEIGHT; i++)
    {
        *dst++ = rand();
    }

    dpy_gfx_update(s->con, 0, 0, LCD_WIDTH, LCD_HEIGHT);
}

static void YUV2RGB_realize(DeviceState *dev, Error **errp)
{
}

static const VMStateDescription YUV2RGB_vmstate = {
    .name = TYPE_ATJ213X_YUV2RGB,
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .fields      = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, AtjYUV2RGBState, YUV2RGB_REG_NUM),
        VMSTATE_END_OF_LIST()
    }
};

static void YUV2RGB_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = YUV2RGB_realize;
    dc->reset = YUV2RGB_reset;
    dc->user_creatable = false;
    dc->vmsd = &YUV2RGB_vmstate;
}

static const TypeInfo YUV2RGB_info = {
    .name          = TYPE_ATJ213X_YUV2RGB,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AtjYUV2RGBState),
    .instance_init = YUV2RGB_init,
    .class_init    = YUV2RGB_class_init,
};

static void YUV2RGB_register_types(void)
{
    type_register_static(&YUV2RGB_info);
}
type_init(YUV2RGB_register_types)
