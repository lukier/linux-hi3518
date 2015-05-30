#ifndef _HI3518_DEFS_H_
#define _HI3518_DEFS_H_

#include <asm/memory.h>

#define DDR_BASE        0x80000000
#define DDRC_BASE       0x20110000
#define IOCONFIG_BASE   0x200F0000
#define UART2_BASE      0x200A0000
#define UART1_BASE      0x20090000
#define UART0_BASE      0x20080000
#define SYS_CTRL_BASE   0x20050000
#define WDG_BASE        0x20040000
#define CRG_REG_BASE    0x20030000
#define TIMER23_BASE    0x20010000
#define TIMER01_BASE    0x20000000
#define REG_BASE_INTC   0x10140000
#define DMAC_BASE       0x100D0000
#define RTC_BASE        0x20060000

#define SSP0_BASE       0x200C0000
#define SSP1_BASE       0x200E0000
#define I2C_BASE        0x200D0000

#define GPIO0_BASE      0x20140000 
#define GPIO1_BASE      0x20150000 
#define GPIO2_BASE      0x20160000 
#define GPIO3_BASE      0x20170000 
#define GPIO4_BASE      0x20180000 
#define GPIO5_BASE      0x20190000 
#define GPIO6_BASE      0x201a0000 
#define GPIO7_BASE      0x201b0000 
#define GPIO8_BASE      0x201c0000
#define GPIO9_BASE      0x201d0000
#define GPIO10_BASE     0x201e0000
#define GPIO11_BASE     0x201f0000

#define HI3518_IRQ_START    (0)

#define TIMER01_IRQ     (HI3518_IRQ_START + 3)
#define TIMER23_IRQ     (HI3518_IRQ_START + 4)

#define UART0_IRQ       (HI3518_IRQ_START + 5)
#define UART1_IRQ       (HI3518_IRQ_START + 5)
#define UART2_IRQ       (HI3518_IRQ_START + 25)

#define GPIO0_IRQ       (HI3518_IRQ_START + 29)
#define GPIO1_IRQ       (HI3518_IRQ_START + 29)
#define GPIO2_IRQ       (HI3518_IRQ_START + 29)
#define GPIO11_IRQ      (HI3518_IRQ_START + 29)

#define GPIO3_IRQ       (HI3518_IRQ_START + 30)
#define GPIO4_IRQ       (HI3518_IRQ_START + 30)
#define GPIO5_IRQ       (HI3518_IRQ_START + 30)
#define GPIO10_IRQ      (HI3518_IRQ_START + 30)

#define GPIO6_IRQ       (HI3518_IRQ_START + 31)
#define GPIO7_IRQ       (HI3518_IRQ_START + 31)
#define GPIO8_IRQ       (HI3518_IRQ_START + 31)
#define GPIO9_IRQ       (HI3518_IRQ_START + 31)

#define WDG_IRQ         (HI3518_IRQ_START + 1)

#define RTC_IRQ         (HI3518_IRQ_START + 2)
#define TEMPER_IRQ      (HI3518_IRQ_START + 10)

#define SSP0_IRQ        (HI3518_IRQ_START + 6)
#define SSP1_IRQ        (HI3518_IRQ_START + 7)

#define I2C_IRQ         (HI3518_IRQ_START + 20)

#define DMAC_IRQ        (HI3518_IRQ_START + 14)

#define HI3518_NR_IRQS  (HI3518_IRQ_START + 32)

#endif // _HI3518_DEFS_H_