/*
 * Copyright (c) 2015 Robert Lukierski.
 * Copyright (c) 2012-2013 Hisilicon Limited.
 * Copyright (c) 2012-2013 Linaro Limited.
 *
 * Author: Haojian Zhuang <haojian.zhuang@linaro.org>
 *         Xin Li <li.xin@linaro.org>
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

#ifndef __DTS_HI3518_CLOCK_H
#define __DTS_HI3518_CLOCK_H

#define HI3518_NONE_CLOCK   0

/* fixed rate & fixed factor clocks */
#define HI3518_OSC32K       1
#define HI3518_OSC24M       2
#define HI3518_APLL         3
#define HI3518_VPLL0        4
#define HI3518_BPLL         5
#define HI3518_EPLL         6

/* divider clocks */

/* mux clocks */

/* gate clocks */

#define HI3518_NR_CLKS (HI3518_EPLL + 1)

#endif	/* __DTS_HI3518_CLOCK_H */
