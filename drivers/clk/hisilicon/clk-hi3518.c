/*
 * Hisilicon Hi3620 clock driver
 *
 * Copyright (c) 2015 Robert Lukierski.
 * Copyright (c) 2012-2013 Hisilicon Limited.
 * Copyright (c) 2012-2013 Linaro Limited.
 *
 * Author: Haojian Zhuang <haojian.zhuang@linaro.org>
 *           Xin Li <li.xin@linaro.org>
 *         Robert Lukierski <robert@lukierski.eu>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 */

#include <linux/kernel.h>
#include <linux/clk-provider.h>
#include <linux/clkdev.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/slab.h>
#include <linux/clk.h>

#include <dt-bindings/clock/hi3518-clock.h>

static void __init hi3518_clk_init(struct device_node *np)
{
#if 0
    struct clk *clk;
    
    /* APB clock dummy */
    clk = clk_register_fixed_rate(NULL, "apb_pclk", NULL, CLK_IS_ROOT, 0);
    clk_register_clkdev(clk, "apb_pclk", NULL);

    struct hisi_clock_data *clk_data;

    clk_data = hisi_clk_init(np, HI3518_NR_CLKS);
    if (!clk_data)
        return;

    hisi_clk_register_fixed_rate(hi3518_fixed_rate_clks, ARRAY_SIZE(hi3518_fixed_rate_clks), clk_data);
    hisi_clk_register_fixed_factor(hi3518_fixed_factor_clks, ARRAY_SIZE(hi3518_fixed_factor_clks), clk_data);
    hisi_clk_register_mux(hi3518_mux_clks, ARRAY_SIZE(hi3518_mux_clks), clk_data);
    hisi_clk_register_divider(hi3518_div_clks, ARRAY_SIZE(hi3518_div_clks), clk_data);
    hisi_clk_register_gate_sep(hi3518_seperated_gate_clks, ARRAY_SIZE(hi3518_seperated_gate_clks), clk_data);

    of_clk_add_provider(np, of_clk_src_onecell_get, &clocks);
#endif
    printk(KERN_INFO "LUKIER-ClkHi3518 Initializing hi3518-clock node %s %s %s\n", np->type, np->name, np->full_name);
}

CLK_OF_DECLARE(hi3518_clk, "hisilicon,hi3518-clock", hi3518_clk_init);
