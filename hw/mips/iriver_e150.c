/*
 * QEMU/atj213x emulation
 *
 * Emulates Iriver e150 DAP based on Actions ATJ2137 SOC
 * 
 * Copyright (c) 2017-2019 Marcin Bukat
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu-common.h"
#include "cpu.h"
#include "hw/hw.h"
#include "hw/mips/mips.h"
#include "hw/mips/cpudevs.h"
#include "hw/ptimer.h"
#include "sysemu/sysemu.h"
#include "sysemu/watchdog.h"
#include "hw/boards.h"
#include "hw/mips/bios.h"
#include "hw/loader.h"
#include "hw/misc/unimp.h"
#include "elf.h"
#include "hw/sysbus.h"
#include "exec/address-spaces.h"
#include "qemu/error-report.h"
#include "sysemu/qtest.h"
#include "exec/log.h"
#include "ui/console.h"
#include "ui/pixel_ops.h"
#include "hw/sd/sd.h"
#include "trace.h"
#include "audio/audio.h"

#include "atj213x_soc.h"

static struct _loaderparams {
    int ram_size;
    const char *kernel_filename;
    const char *kernel_cmdline;
    const char *initrd_filename;
} loaderparams;

typedef struct ResetData {
    MIPSCPU *cpu;
    uint64_t vector;
} ResetData;

static int64_t load_kernel(void)
{
    int64_t entry;

    int kernel_size = load_image_targphys(loaderparams.kernel_filename, 0, 8*1024*1024);

    if (kernel_size <= 0)
        exit(1);

    entry = (int32_t)0x80000000;
    return entry;
}

static void main_cpu_reset(void *opaque)
{
    ResetData *s = (ResetData *)opaque;
    CPUMIPSState *env = &s->cpu->env;

    cpu_reset(CPU(s->cpu));
    env->active_tc.PC = s->vector & ~(target_ulong)1;
    if (s->vector & 1) {
        env->hflags |= MIPS_HFLAG_M16;
    }
}

/* Input driver
 * emulate hardware buttons and hw lines on Iriver e150
 * some inputs are connected to gpios, some are to ADC
 *
 *   PC button  | Iriver button | comment
 *   -----------+---------------+---------------------------------
 *     ESC      |  ON           | !GPIOA8
 *     h        |  hold         | !GPIOA10 defaults to not on hold
 *     pageup   |  vol+         | !GPIOA12
 *     pagedown |  vol-         | GPIOB31
 *     left     |  left         | ADC 0x04
 *     right    |  right        | ADC 0x0c
 *     up       |  up           | ADC 0x02
 *     down     |  down         | ADC 0x08
 *     ENTER    |  select       | ADC 0x06
 *     p        |  headphone    | !GPIOA26 defaults to inserted
 *     s        |  SD detect    | !GPIOB22 defaults to inserted
 *     <        |  dec BAT      |
 *     >        |  inc BAT      |
 */
typedef struct {
    qemu_irq irq;
    int keycode;
    int type;
    int adc_val;
    uint8_t pressed;
} atj_button;

typedef struct {
    void *pmu;
    atj_button *buttons;
    int num_buttons;
    int extension;
} atj_button_state;

enum {
    KEY_MONOSTABLE,
    KEY_BISTABLE,
    KEY_ADC_KEY,
    KEY_ADC_BAT_INC,
    KEY_ADC_BAT_DEC
};

static void atj_put_key(void * opaque, int keycode)
{
    atj_button_state *s = (atj_button_state *)opaque;
    AtjPMUState *pmu = s->pmu;
    int down;

    trace_atj_put_key(keycode);

    if (keycode == 0xe0 && !s->extension)
    {
        s->extension = (0xe0 << 8);
        return;
    }

    down = (keycode & 0x80) ? 0 : 1;
    keycode = (keycode & 0x7f) | s->extension;

    for (int i = 0; i < s->num_buttons; i++)
    {
        switch (s->buttons[i].type)
        {
            case KEY_MONOSTABLE:
            {
                if ((s->buttons[i].keycode == keycode) &&
                    s->buttons[i].pressed != down)
                {
                    s->buttons[i].pressed = down;
                    qemu_set_irq(s->buttons[i].irq, down);
                }
                break;
            }

            case KEY_BISTABLE:
            {
                if ((s->buttons[i].keycode == keycode) && down)
                {
                    s->buttons[i].pressed = !s->buttons[i].pressed;
                    qemu_set_irq(s->buttons[i].irq, s->buttons[i].pressed);
                }
                break;
            }

            case KEY_ADC_KEY:
            {
                if ((s->buttons[i].keycode) == keycode &&
                    s->buttons[i].pressed != down)
                {
                    s->buttons[i].pressed = down;
                    pmu->lradc_set(s->pmu, LRADC_CH_KEY, down ? s->buttons[i].adc_val : 0x0f);
                }
                break;
            }

	    case KEY_ADC_BAT_INC:
	    {
                if ((s->buttons[i].keycode) == keycode && down)
		{
		    pmu->lradc_set(s->pmu, LRADC_CH_BAT, pmu->lradc_get(s->pmu, LRADC_CH_BAT) + 1);
		}
		break;
	    }

	    case KEY_ADC_BAT_DEC:
	    {
                if ((s->buttons[i].keycode) == keycode && down)
		{
		    pmu->lradc_set(s->pmu, LRADC_CH_BAT, pmu->lradc_get(s->pmu, LRADC_CH_BAT) - 1);
		}
		break;
	    }
        }
    }

    s->extension = 0;
}

static const VMStateDescription vmstate_atj_button = {
    .name = "atj_button",
    .version_id = 0,
    .minimum_version_id = 0,
    .fields = (VMStateField[]) {
        VMSTATE_UINT8(pressed, atj_button),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_atj_buttons = {
    .name = "atj_buttons",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_INT32(extension, atj_button_state),
        VMSTATE_STRUCT_VARRAY_INT32(buttons, atj_button_state, num_buttons, 0,
                              vmstate_atj_button, atj_button),
        VMSTATE_END_OF_LIST()
    }
};

static void atj_button_init(int n, atj_button *button_desc, void *pmu)
{
    atj_button_state *s;
    int i;

    s = g_new0(atj_button_state, 1);
    s->buttons = g_new0(atj_button, n);
    for (i = 0; i < n; i++) {
        s->buttons[i].irq = button_desc[i].irq;
        s->buttons[i].keycode = button_desc[i].keycode;
        s->buttons[i].type = button_desc[i].type;
        s->buttons[i].adc_val = button_desc[i].adc_val;
        s->buttons[i].pressed = 0;
    }
    s->num_buttons = n;
    s->pmu = pmu;
    qemu_add_kbd_event_handler(atj_put_key, s);
    vmstate_register(NULL, -1, &vmstate_atj_buttons, s);
}

struct IriverE150State {
    MemoryRegion ram;
    MemoryRegion rom;
    Atj213xSocState soc;
};
typedef struct IriverE150State IriverE150State;

static void IriverE150_init(MachineState *machine)
{
    IriverE150State *e150;
    e150 = g_new0(IriverE150State, 1);
    object_initialize(&e150->soc, (sizeof(e150->soc)), TYPE_ATJ213X_SOC);
    object_property_add_child(OBJECT(machine), "soc", OBJECT(&e150->soc),
                              &error_abort);
    object_property_set_bool(OBJECT(&e150->soc), true, "realized",
                             &error_abort);

    /* main system memory */
    memory_region_allocate_system_memory(&e150->ram, OBJECT(machine), "ram",
                                         machine->ram_size);
    memory_region_add_subregion_overlap(get_system_memory(), 0, &e150->ram, 0);

    /* rom area */
    memory_region_init_rom(&e150->rom, NULL, "rom", BIOS_SIZE, &error_abort);


    //const char *cpu_model = machine->cpu_model;
    const char *kernel_filename = machine->kernel_filename;
    const char *kernel_cmdline = machine->kernel_cmdline;
    const char *initrd_filename = machine->initrd_filename;
    char *filename;
    ResetData *reset_info;
    int bios_size;

    MIPSCPU *cpu = &e150->soc.cpu;
    CPUMIPSState *env = &cpu->env;

    reset_info = g_malloc0(sizeof(ResetData));
    reset_info->cpu = cpu;
    reset_info->vector = env->active_tc.PC;
    qemu_register_reset(main_cpu_reset, reset_info);
    if (bios_name == NULL)
        bios_name = BIOS_FILENAME;
    filename = qemu_find_file(QEMU_FILE_TYPE_BIOS, bios_name);
    if (filename) {
        bios_size = load_image_targphys(filename, 0x1fc00000LL, BIOS_SIZE);
        g_free(filename);
    } else {
        bios_size = -1;
    }
    if ((bios_size < 0 || bios_size > BIOS_SIZE) &&
        !kernel_filename && !qtest_enabled()) {
        /* Bail out if we have neither a kernel image nor boot vector code. */
        error_report("Could not load MIPS bios '%s', and no "
                     "-kernel argument was specified", bios_name);
        exit(1);
    } else {
        /* We have a boot vector start address. */
        env->active_tc.PC = (target_long)(int32_t)0xbfc00000;
    }

    if (kernel_filename) {
        loaderparams.ram_size = ram_size;
        loaderparams.kernel_filename = kernel_filename;
        loaderparams.kernel_cmdline = kernel_cmdline;
        loaderparams.initrd_filename = initrd_filename;
        reset_info->vector = load_kernel();
    }

    atj_button button_desc[13];

    Atj213xSocState *soc = &e150->soc;

    /* HOLD defaults to 'not engaged' */
    button_desc[0].irq = qemu_irq_invert(qdev_get_gpio_in(DEVICE(&soc->gpio), 10)); /* GPIOA10 */
    button_desc[0].type = KEY_BISTABLE;
    button_desc[0].keycode = 0x23; /* 'h' */
    button_desc[0].adc_val = 0;

    /* POWER */    
    button_desc[1].irq = qemu_irq_invert(qdev_get_gpio_in(DEVICE(&soc->gpio), 8)); /* GPIOA8 */
    button_desc[1].type = KEY_MONOSTABLE;
    button_desc[1].keycode = 0x01; /* ESC */
    button_desc[1].adc_val = 0;

    /* VOL+ */
    button_desc[2].irq = qemu_irq_invert(qdev_get_gpio_in(DEVICE(&soc->gpio), 12)); /* GPIOA12 */
    button_desc[2].type = KEY_MONOSTABLE;
    button_desc[2].keycode = 0xe049; /* pageup */
    button_desc[2].adc_val = 0;

    /* VOL- */
    button_desc[3].irq = qdev_get_gpio_in(DEVICE(&soc->gpio), 32 + 31); /* GPIOB31 */
    button_desc[3].type = KEY_MONOSTABLE;
    button_desc[3].keycode = 0xe051; /* pagedown */
    button_desc[3].adc_val = 0;

    /* LEFT */
    button_desc[4].irq = NULL; /* adc key */
    button_desc[4].type = KEY_ADC_KEY;
    button_desc[4].keycode = 0xe04b; /* left */
    button_desc[4].adc_val = 0x04;

    /* RIGHT */
    button_desc[5].irq = NULL; /* adc key */
    button_desc[5].type = KEY_ADC_KEY;
    button_desc[5].keycode = 0xe04d; /* right */
    button_desc[5].adc_val = 0x0c;

    /* UP */
    button_desc[6].irq = NULL; /* adc key */
    button_desc[6].type = KEY_ADC_KEY;
    button_desc[6].keycode = 0xe048; /* up */
    button_desc[6].adc_val = 0x02;

    /* DOWN */
    button_desc[7].irq = NULL; /* adc key */
    button_desc[7].type = KEY_ADC_KEY;
    button_desc[7].keycode = 0xe050; /* down */
    button_desc[7].adc_val = 0x08;

    /* SELECT */
    button_desc[8].irq = NULL; /* adc key */
    button_desc[8].type = KEY_ADC_KEY;
    button_desc[8].keycode = 0x1c; /* enter */
    button_desc[8].adc_val = 0x06;

    /* HEADPHONE DETECT defaults to inserted */
    button_desc[9].irq = qdev_get_gpio_in(DEVICE(&soc->gpio), 26); /* GPIOA26 */
    button_desc[9].type = KEY_BISTABLE;
    button_desc[9].keycode = 0x19; /* 'p' */
    button_desc[9].adc_val = 0;
 
    /* SD DETECT defaults to inserted */
    button_desc[10].irq = qdev_get_gpio_in(DEVICE(&soc->gpio), 32 + 22); /* GPIOB22 */
    button_desc[10].type = KEY_BISTABLE;
    button_desc[10].keycode = 0x1f; /* 's' */
    button_desc[10].adc_val = 0;

    /* Battery voltage increase */
    button_desc[11].irq = NULL;
    button_desc[11].type = KEY_ADC_BAT_INC;
    button_desc[11].keycode = 0x34;
    button_desc[11].adc_val = 0;

    /* Battery voltage decrease */
    button_desc[12].irq = NULL;
    button_desc[12].type = KEY_ADC_BAT_DEC;
    button_desc[12].keycode = 0x33;
    button_desc[12].adc_val = 0;

    /* Initialize input layer */
    atj_button_init(13, button_desc, (void *)&soc->pmu);
};

static void IriverE150_machine_init(MachineClass *mc)
{
    mc->desc = "Iriver E150";
    mc->init = IriverE150_init;
    mc->no_parallel = 1;
    mc->no_floppy = 1;
    mc->no_cdrom = 1;
    mc->max_cpus = 1;
    mc->default_ram_size = 8 * 1024 * 1024;
}

DEFINE_MACHINE("IriverE150", IriverE150_machine_init)
