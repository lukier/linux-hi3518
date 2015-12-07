/*
 * (Hisilicon's SoC based) flattened device tree enabled machine
 *
 * Copyright (c) 2015 Robert Lukierski.
 * Copyright (c) 2012-2013 Hisilicon Ltd.
 * Copyright (c) 2012-2013 Linaro Ltd.
 *
 * Author: Haojian Zhuang <haojian.zhuang@linaro.org>
 *         Robert Lukierski <robert@lukierski.eu>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/clocksource.h>
#include <linux/irqchip.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/hw_irq.h>

#include <linux/reboot.h>
#include <asm/system_misc.h>

#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_device.h>

#include <linux/amba/bus.h>
#include <linux/amba/sp810.h>
#include <linux/amba/pl022.h>
#include <linux/amba/pl061.h>
#include <linux/amba/pl080.h>
#include <linux/amba/pl08x.h>

#include <linux/uio_driver.h>

#define HI3620_SYSCTRL_PHYS_BASE		0xfc802000
#define HI3620_SYSCTRL_VIRT_BASE		0xfe802000

/*
 * This table is only for optimization. Since ioremap() could always share
 * the same mapping if it's defined as static IO mapping.
 *
 * Without this table, system could also work. The cost is some virtual address
 * spaces wasted since ioremap() may be called multi times for the same
 * IO space.
 */
static struct map_desc hi3620_io_desc[] __initdata = {
	{
		/* sysctrl */
		.pfn		= __phys_to_pfn(HI3620_SYSCTRL_PHYS_BASE),
		.virtual	= HI3620_SYSCTRL_VIRT_BASE,
		.length		= 0x1000,
		.type		= MT_DEVICE,
	},
};

static void __init hi3620_map_io(void)
{
	debug_ll_io_init();
	iotable_init(hi3620_io_desc, ARRAY_SIZE(hi3620_io_desc));
}

static const char *const hi3xxx_compat[] __initconst = {
	"hisilicon,hi3620-hi4511",
	NULL,
};

DT_MACHINE_START(HI3620, "Hisilicon Hi3620 (Flattened Device Tree)")
	.map_io		= hi3620_map_io,
	.dt_compat	= hi3xxx_compat,
MACHINE_END

#include "hi3518_defs.h"

static void hi3518_restart(enum reboot_mode mode, const char *cmd)
{
    void* regmem = ioremap(SYS_CTRL_BASE, 0x10000);
    /* hardware reset, Use on-chip reset capability */
    sysctl_soft_reset(regmem);
}

static struct pl061_platform_data gpio_plat_data[] = {
    { .gpio_base  = 0, .irq_base   = GPIO0_IRQ, .directions = 0x00, .values = 0x00, },
    { .gpio_base  = 8, .irq_base   = GPIO1_IRQ, .directions = 0x00, .values = 0x00, },
    { .gpio_base  = 16, .irq_base   = GPIO2_IRQ, .directions = 0x00, .values = 0x00, },
    { .gpio_base  = 24, .irq_base   = GPIO3_IRQ, .directions = 0x00, .values = 0x00, },
    { .gpio_base  = 32, .irq_base   = GPIO4_IRQ, .directions = 0x00, .values = 0x00, },
    { .gpio_base  = 40, .irq_base   = GPIO5_IRQ, .directions = 0x00, .values = 0x00, },
    { .gpio_base  = 48, .irq_base   = GPIO6_IRQ, .directions = 0x00, .values = 0x00, },
    { .gpio_base  = 56, .irq_base   = GPIO7_IRQ, .directions = 0x00, .values = 0x00, },
    { .gpio_base  = 64, .irq_base   = GPIO8_IRQ, .directions = 0x00, .values = 0x00, },
    { .gpio_base  = 72, .irq_base   = GPIO9_IRQ, .directions = 0x00, .values = 0x00, },
    { .gpio_base  = 80, .irq_base   = GPIO10_IRQ, .directions = 0x00, .values = 0x00, },
    { .gpio_base  = 88, .irq_base   = GPIO11_IRQ, .directions = 0x00, .values = 0x00, },
};

/* ssp device registration */
struct pl022_ssp_controller pl022_plat_data[] = {
    {
        .bus_id = 1,
        .enable_dma = 0, // Enabling DMA disables SPIDEV apparently 
        .dma_filter = pl08x_filter_id,
        .dma_tx_param = "ssp0_tx",
        .dma_rx_param = "ssp0_rx",
        /*
         * This is number of spi devices that can be connected to spi. There are
         * two type of chipselects on which slave devices can work. One is chip
         * select provided by spi masters other is controlled through external
         * gpio's. We can't use chipselect provided from spi master (because as
         * soon as FIFO becomes empty, CS is disabled and transfer ends). So
         * this number now depends on number of gpios available for spi. each
         * slave on each master requires a separate gpio pin.
         */
        .num_chipselect = 2,
    },
    {
        .bus_id = 2,
        .enable_dma = 0, // Enabling DMA disables SPIDEV apparently 
        .dma_filter = pl08x_filter_id,
        .dma_tx_param = "ssp1_tx",
        .dma_rx_param = "ssp1_rx",
        /*
         * This is number of spi devices that can be connected to spi. There are
         * two type of chipselects on which slave devices can work. One is chip
         * select provided by spi masters other is controlled through external
         * gpio's. We can't use chipselect provided from spi master (because as
         * soon as FIFO becomes empty, CS is disabled and transfer ends). So
         * this number now depends on number of gpios available for spi. each
         * slave on each master requires a separate gpio pin.
         */
        .num_chipselect = 2,
    }
};

/* DMAC helpers */
static int pl08x_get_xfer_signal(const struct pl08x_channel_data *cd)
{
    return cd->min_signal;
}

static void pl08x_put_xfer_signal(const struct pl08x_channel_data *cd, int ch)
{
}

/* DMAC platform data's slave info */
struct pl08x_channel_data hi3518_dma_info[] = {
    {
        .bus_id = "sio_rx",
        .min_signal = 0,
        .max_signal = 0,
        .muxval = 0,
        .periph_buses = PL08X_AHB1,
    }, {
        .bus_id = "sio_tx",
        .min_signal = 1,
        .max_signal = 1,
        .muxval = 0,
        .periph_buses = PL08X_AHB1,
    }, {
        .bus_id = "uart0_rx",
        .min_signal = 6,
        .max_signal = 6,
        .muxval = 0,
        .periph_buses = PL08X_AHB1,
    }, {
        .bus_id = "uart0_tx",
        .min_signal = 7,
        .max_signal = 7,
        .muxval = 0,
        .periph_buses = PL08X_AHB1,
    }, {
        .bus_id = "uart1_rx",
        .min_signal = 8,
        .max_signal = 8,
        .muxval = 0,
        .periph_buses = PL08X_AHB1,
    }, {
        .bus_id = "uart1_tx",
        .min_signal = 9,
        .max_signal = 9,
        .muxval = 0,
        .periph_buses = PL08X_AHB1,
    }, {
        .bus_id = "uart2_rx",
        .min_signal = 10,
        .max_signal = 10,
        .muxval = 0,
        .periph_buses = PL08X_AHB1,
    }, {
        .bus_id = "uart2_tx",
        .min_signal = 11,
        .max_signal = 11,
        .muxval = 0,
        .periph_buses = PL08X_AHB1,
    }, {
        .bus_id = "ssp1_rx",
        .min_signal = 12,
        .max_signal = 12,
        .muxval = 0,
        .periph_buses = PL08X_AHB1,
    }, {
        .bus_id = "ssp1_tx",
        .min_signal = 13,
        .max_signal = 13,
        .muxval = 0,
        .periph_buses = PL08X_AHB1,
    }, {
        .bus_id = "ssp0_rx",
        .min_signal = 14,
        .max_signal = 14,
        .muxval = 0,
        .periph_buses = PL08X_AHB1,
    }, {
        .bus_id = "ssp0_tx",
        .min_signal = 15,
        .max_signal = 15,
        .muxval = 0,
        .periph_buses = PL08X_AHB1,
    },
};

/* dmac device registration */
struct pl08x_platform_data pl080_plat_data = {
    .slave_channels = hi3518_dma_info,
    .num_slave_channels = ARRAY_SIZE(hi3518_dma_info),
    .memcpy_channel = {
        .bus_id = "memcpy",
        .cctl_memcpy =
        (PL080_BSIZE_4 << PL080_CONTROL_SB_SIZE_SHIFT | \
        PL080_BSIZE_4 << PL080_CONTROL_DB_SIZE_SHIFT | \
        PL080_WIDTH_32BIT << PL080_CONTROL_SWIDTH_SHIFT | \
        PL080_WIDTH_32BIT << PL080_CONTROL_DWIDTH_SHIFT | \
        PL080_CONTROL_PROT_BUFF | PL080_CONTROL_PROT_CACHE | \
        PL080_CONTROL_PROT_SYS),
    },
    .lli_buses = PL08X_AHB1,
    .mem_buses = PL08X_AHB1,
    .get_xfer_signal = pl08x_get_xfer_signal,
    .put_xfer_signal = pl08x_put_xfer_signal,
};

/* Add auxdata to pass platform data */
static struct of_dev_auxdata hi3518_auxdata_lookup[] __initdata = {
    
    OF_DEV_AUXDATA("arm,sp805", WDG_BASE,  NULL, NULL),
    
    OF_DEV_AUXDATA("arm,pl081", GPIO0_BASE,  NULL, &gpio_plat_data[0]),
    OF_DEV_AUXDATA("arm,pl081", GPIO1_BASE,  NULL, &gpio_plat_data[1]),
    OF_DEV_AUXDATA("arm,pl081", GPIO2_BASE,  NULL, &gpio_plat_data[2]),
    OF_DEV_AUXDATA("arm,pl081", GPIO3_BASE,  NULL, &gpio_plat_data[3]),
    OF_DEV_AUXDATA("arm,pl081", GPIO4_BASE,  NULL, &gpio_plat_data[4]),
    OF_DEV_AUXDATA("arm,pl081", GPIO5_BASE,  NULL, &gpio_plat_data[5]),
    OF_DEV_AUXDATA("arm,pl081", GPIO6_BASE,  NULL, &gpio_plat_data[6]),
    OF_DEV_AUXDATA("arm,pl081", GPIO7_BASE,  NULL, &gpio_plat_data[7]),
    OF_DEV_AUXDATA("arm,pl081", GPIO8_BASE,  NULL, &gpio_plat_data[8]),
    OF_DEV_AUXDATA("arm,pl081", GPIO9_BASE,  NULL, &gpio_plat_data[9]),
    OF_DEV_AUXDATA("arm,pl081", GPIO10_BASE, NULL, &gpio_plat_data[10]),
    OF_DEV_AUXDATA("arm,pl081", GPIO11_BASE, NULL, &gpio_plat_data[11]),
    
    OF_DEV_AUXDATA("arm,pl022", SSP0_BASE, NULL, &pl022_plat_data[0]),
    OF_DEV_AUXDATA("arm,pl022", SSP1_BASE, NULL, &pl022_plat_data[1]),
    
    OF_DEV_AUXDATA("arm,pl080", DMAC_BASE, NULL, &pl080_plat_data),
    {}
};

static void __init hi3518_dt_init(void)
{    
    of_platform_populate(NULL, of_default_bus_match_table, hi3518_auxdata_lookup, NULL);
}

static const char *const hi3518_compat[] __initconst = {
    "hisilicon,hi3518",
    "hisilicon,hi3516",
    NULL,
};

DT_MACHINE_START(HI3518, "Hisilicon Hi3516/Hi3518 (Flattened Device Tree)")
    .dt_compat    = hi3518_compat,
    .init_machine = hi3518_dt_init,
    .restart      = hi3518_restart,
    .nr_irqs      = HI3518_NR_IRQS,
MACHINE_END

static const char *const hix5hd2_compat[] __initconst = {
	"hisilicon,hix5hd2",
	NULL,
};

DT_MACHINE_START(HIX5HD2_DT, "Hisilicon HIX5HD2 (Flattened Device Tree)")
	.dt_compat	= hix5hd2_compat,
MACHINE_END

static const char *const hip04_compat[] __initconst = {
	"hisilicon,hip04-d01",
	NULL,
};

DT_MACHINE_START(HIP04, "Hisilicon HiP04 (Flattened Device Tree)")
	.dt_compat	= hip04_compat,
MACHINE_END

static const char *const hip01_compat[] __initconst = {
	"hisilicon,hip01",
	"hisilicon,hip01-ca9x2",
	NULL,
};

DT_MACHINE_START(HIP01, "Hisilicon HIP01 (Flattened Device Tree)")
	.dt_compat      = hip01_compat,
MACHINE_END

