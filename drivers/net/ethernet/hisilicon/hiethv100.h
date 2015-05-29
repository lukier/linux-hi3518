#ifndef _HIETHV100_H_
#define _HIETHV100_H_

#define HIETH_MAX_QUEUE_DEPTH_RX   32
#define HIETH_MAX_QUEUE_DEPTH_TX   32
#define HIETH_MAX_FRAME_SIZE       (SZ_1K*2)
#define HIETH_NAPI_WEIGHT          16

#define HIETH_REG_PERI_CRG51            0x200300CC
#define HIETH_REG_UD_MAC_PORTSEL        0x0200
#define HIETH_REG_UD_MAC_RO_STAT        0x0204
#define HIETH_REG_UD_MAC_PORTSET        0x0208
#define HIETH_REG_UD_MAC_STAT_CHANGE    0x020C
#define HIETH_REG_UD_MAC_SET            0x0210
#define HIETH_REG_UD_MAC_RX_IPGCTRL     0x0214 /* UNDOCUMENTED */
#define HIETH_REG_UD_MAC_TX_IPGCTRL     0x0218 /* UNDOCUMENTED */
#define HIETH_REG_UD_MAC_EEE_INT        0x0480
#define HIETH_REG_UD_MAC_EEE_INTEN      0x0484
#define HIETH_REG_UD_MAC_EEE_ENA        0x0488
#define HIETH_REG_UD_MAC_EEE_TIMER      0x048C
#define HIETH_REG_UD_MAC_EEE_LINK_STAT  0x0490
#define HIETH_REG_UD_MAC_EEE_CLK_CNT    0x0494
#define HIETH_REG_GLB_HOSTMAC_L32       0x1300
#define HIETH_REG_GLB_HOSTMAC_H16       0x1304
#define HIETH_REG_GLB_SOFT_RESET        0x1308
#define HIETH_REG_GLB_FWCTRL            0x1310
#define HIETH_REG_GLB_MACTCTRL          0x1314
#define HIETH_REG_GLB_ENDIAN_MOD        0x1318
#define HIETH_REG_GLB_IRQ_STAT          0x1330
#define HIETH_REG_GLB_IRQ_ENA           0x1334
#define HIETH_REG_GLB_IRQ_RAW           0x1338
#define HIETH_REG_GLB_MAC0_L32          0x1400
#define HIETH_REG_GLB_MAC0_H16          0x1404
#define HIETH_REG_GLB_MAC1_L32          0x1408
#define HIETH_REG_GLB_MAC1_H16          0x140C
#define HIETH_REG_GLB_MAC2_L32          0x1410
#define HIETH_REG_GLB_MAC2_H16          0x1414
#define HIETH_REG_GLB_MAC3_L32          0x1418
#define HIETH_REG_GLB_MAC3_H16          0x141C
#define HIETH_REG_GLB_MAC4_L32          0x1420
#define HIETH_REG_GLB_MAC4_H16          0x1424
#define HIETH_REG_GLB_MAC5_L32          0x1428
#define HIETH_REG_GLB_MAC5_H16          0x142C
#define HIETH_REG_GLB_MAC6_L32          0x1430
#define HIETH_REG_GLB_MAC6_H16          0x1434
#define HIETH_REG_GLB_MAC7_L32          0x1438
#define HIETH_REG_GLB_MAC7_H16          0x143C
#define HIETH_REG_UD_GLB_IRQN_SET       0x0340
#define HIETH_REG_UD_GLB_QLEN_SET       0x0344
#define HIETH_REG_UD_GLB_FC_LEVEL       0x0348
#define HIETH_REG_UD_GLB_CAUSE          0x034C
#define HIETH_REG_UD_GLB_RXFRM_SADDR    0x0350
#define HIETH_REG_UD_GLB_IQFRM_DES      0x0354
#define HIETH_REG_UD_GLB_IQ_ADDR        0x0358
#define HIETH_REG_UD_GLB_BFC_STAT       0x035C
#define HIETH_REG_UD_GLB_EQ_ADDR        0x0360
#define HIETH_REG_UD_GLB_EQFRM_LEN      0x0364
#define HIETH_REG_UD_GLB_QSTAT          0x0368
#define HIETH_REG_UD_GLB_ADDRQ_STAT     0x036C
#define HIETH_REG_UD_GLB_FC_TIMECTRL    0x0370
#define HIETH_REG_UD_GLB_FC_RXLIMIT     0x0374
#define HIETH_REG_UD_GLB_FC_DROPCTRL    0x0378
#define HIETH_REG_UD_STS_PORTCNT        0x0584
#define HIETH_REG_UD_PORT2CPU_PKTS      0x05A0
#define HIETH_REG_UD_RX_DVCNT           0x0600
#define HIETH_REG_UD_RX_OCTS            0x0604
#define HIETH_REG_UD_RX_RIGHTOCTS       0x0608
#define HIETH_REG_UD_HOSTMAC_PKTS       0x060C
#define HIETH_REG_UD_RX_RIGHTPKTS       0x0610
#define HIETH_REG_UD_RX_BROADPKTS       0x0614
#define HIETH_REG_UD_RX_MULTPKTS        0x0618
#define HIETH_REG_UD_RX_UNIPKTS         0x061C
#define HIETH_REG_UD_RX_ERRPKTS         0x0620
#define HIETH_REG_UD_RX_CRCERR_PKTS     0x0624
#define HIETH_REG_UD_RX_LENERR_PKTS     0x0628
#define HIETH_REG_UD_RX_OCRCERR_PKTS    0x062C
#define HIETH_REG_UD_RX_PAUSE_PKTS      0x0630
#define HIETH_REG_UD_RF_OVERCNT         0x0634
#define HIETH_REG_UD_FLUX_TOL_IPKTS     0x0638
#define HIETH_REG_UD_FLUX_TOL_DPKTS     0x063C
#define HIETH_REG_UD_MN2CPU_PKTS        0x064C
#define HIETH_REG_UD_TX_PKTS            0x0780
#define HIETH_REG_UD_TX_BROADPKTS       0x0784
#define HIETH_REG_UD_TX_MULTPKTS        0x0788
#define HIETH_REG_UD_TX_UNIPKTS         0x078C
#define HIETH_REG_UD_TX_OCTS            0x0790
#define HIETH_REG_UD_TX_PAUSE_PKTS      0x0794
#define HIETH_REG_UD_TX_RETRYCNT        0x0798
#define HIETH_REG_UD_TX_COLCNT          0x079C
#define HIETH_REG_UD_TX_LC_PKTS         0x07A0
#define HIETH_REG_UD_TX_COLOK_PKTS      0x07A4
#define HIETH_REG_UD_TX_RETRY15_PKTS    0x07A8
#define HIETH_REG_UD_TX_RETRYN_PKTS     0x07AC

#define HIETH_FLAG_INT_RX_UP            (1 << 0)
#define HIETH_FLAG_INT_TX_UP            (1 << 1)
#define HIETH_FLAG_INT_LINK_UP          (1 << 2)
#define HIETH_FLAG_INT_SPEED_UP         (1 << 3)
#define HIETH_FLAG_INT_DUPLEX_UP        (1 << 4)
#define HIETH_FLAG_INT_STAT_UP          (1 << 5)
#define HIETH_FLAG_INT_TXQUEUE_UP       (1 << 6)
#define HIETH_FLAG_INT_RXD_UP           (1 << 7)
#define HIETH_FLAG_INT_VLAN             (1 << 11)
#define HIETH_FLAG_INT_MDIO_FINISH      (1 << 12)

#define HIETH_FLAG_INT_ALL              (1 << 19)
#define HIETH_FLAG_INT_ALL_UP           (1 << 18)

#define HIETH_FLAG_INT_MASK             (HIETH_FLAG_INT_RX_UP | \
                                         HIETH_FLAG_INT_TX_UP | \
                                         HIETH_FLAG_INT_LINK_UP | \
                                         HIETH_FLAG_INT_SPEED_UP | \
                                         HIETH_FLAG_INT_DUPLEX_UP | \
                                         HIETH_FLAG_INT_STAT_UP | \
                                         HIETH_FLAG_INT_TXQUEUE_UP | \
                                         HIETH_FLAG_INT_RXD_UP | \
                                         HIETH_FLAG_INT_VLAN | \
                                         HIETH_FLAG_INT_MDIO_FINISH)

struct hieth_device {
    struct clk              *clk;
    struct device           *dev;
    struct platform_device  *pdev;
    struct net_device       *ndev;
    
    spinlock_t              lock;
    unsigned int            lock_flags;
    
    void __iomem            *membase;
    
    struct phy_device       *phy_dev;
    struct device_node      *phy_node;
    phy_interface_t         phy_interface;
    
    u32                     msg_enable;
    
    unsigned int            link;
    unsigned int            speed;
    unsigned int            duplex;
    
    struct napi_struct      napi;
    
    unsigned char           rxbuffers[HIETH_MAX_QUEUE_DEPTH_RX][(SZ_1K*2)];
    unsigned int            rx_buffer_cur;
    unsigned char           txbuffers[HIETH_MAX_QUEUE_DEPTH_TX][(SZ_1K*2)];
    unsigned int            tx_buffer_cur;
    
    #if 0    
    struct sk_buff          *skb_last;
    u16                     tx_fifo_stat;
    
    int                     macrx_completed_flag;    
    
    
    struct sk_buff_head rx_head;    /*received pkgs*/
    struct sk_buff_head rx_hw;  /*rx pkgs in hw*/
    struct sk_buff_head tx_hw;  /*tx pkgs in hw*/
    int tx_hw_cnt;
    struct {
    int hw_xmitq;
} depth;
#endif
};

static void _hieth_int_clear(struct hieth_device *hdev, unsigned int ints)
{
    // writing one clears the interrupt
    iowrite32(ints & HIETH_FLAG_INT_MASK, hdev->membase + HIETH_REG_GLB_IRQ_RAW);
}

static unsigned int _hieth_int_rawstatus(struct hieth_device *hdev)
{
    return ioread32(hdev->membase + HIETH_REG_GLB_IRQ_RAW) & HIETH_FLAG_INT_MASK;
}

static unsigned int _hieth_int_status(struct hieth_device *hdev)
{
    return ioread32(hdev->membase + HIETH_REG_GLB_IRQ_STAT) & HIETH_FLAG_INT_MASK;
}

static void _hieth_int_enable(struct hieth_device *hdev, unsigned int ints)
{
    unsigned int reg = ioread32(hdev->membase + HIETH_REG_GLB_IRQ_ENA);
    reg |= (ints & HIETH_FLAG_INT_MASK); // enable
    iowrite32(reg, hdev->membase + HIETH_REG_GLB_IRQ_ENA);
}

static void _hieth_int_disable(struct hieth_device *hdev, unsigned int ints)
{
    unsigned int reg = ioread32(hdev->membase + HIETH_REG_GLB_IRQ_ENA);
    reg &= ~(ints & HIETH_FLAG_INT_MASK); // disable
    iowrite32(reg, hdev->membase + HIETH_REG_GLB_IRQ_ENA);
}

static void _hieth_int_enable_all_control(struct hieth_device *hdev, bool enabled)
{
    if(enabled) {
        set_bit(19, hdev->membase + HIETH_REG_GLB_IRQ_ENA);
    } else {
        clear_bit(19, hdev->membase + HIETH_REG_GLB_IRQ_ENA);
    }
}

static void _hieth_int_enable_up_control(struct hieth_device *hdev, bool enabled)
{
    if(enabled) {
        set_bit(18, hdev->membase + HIETH_REG_GLB_IRQ_ENA);
    } else {
        clear_bit(19, hdev->membase + HIETH_REG_GLB_IRQ_ENA);
    }
}

static void _hieth_set_endian(struct hieth_device *hdev, bool rx_little, bool tx_little)
{
    if(tx_little) {
        set_bit(0, hdev->membase + HIETH_REG_GLB_ENDIAN_MOD);
    } else {
        clear_bit(0, hdev->membase + HIETH_REG_GLB_ENDIAN_MOD);
    }
    
    if(rx_little) {
        set_bit(1, hdev->membase + HIETH_REG_GLB_ENDIAN_MOD);
    } else {
        clear_bit(1, hdev->membase + HIETH_REG_GLB_ENDIAN_MOD);
    }
}

static void _hieth_set_cpu_negotiation(struct hieth_device *hdev, bool enabled)
{
    if(enabled) {
        set_bit(0, hdev->membase + HIETH_REG_UD_MAC_PORTSEL);
    } else {
        clear_bit(0, hdev->membase + HIETH_REG_UD_MAC_PORTSEL);
    }
}

static int _hieth_set_mii_mode(struct hieth_device *hdev)
{
    void* regmem = 0;
    unsigned long flags = 0;
    
    regmem = ioremap(HIETH_REG_PERI_CRG51, 0x04);
    
    spin_lock_irqsave(&hdev->lock, flags);
    
    /* Enable RMII */
    if(hdev->phy_interface == PHY_INTERFACE_MODE_RMII) {
        set_bit(3, regmem);
        set_bit(1, hdev->membase + HIETH_REG_UD_MAC_PORTSEL);
    } else { /* MII */
        clear_bit(3, regmem);
        clear_bit(1, hdev->membase + HIETH_REG_UD_MAC_PORTSEL);
    }
    
    spin_unlock_irqrestore(&hdev->lock, flags);
    
    iounmap(regmem);
    
    return 0;
}

static void _hieth_core_reset(struct hieth_device *hdev)
{
    void* regmem = 0;
    
    dev_dbg(hdev->dev, "resetting device\n");
    
    regmem = ioremap(HIETH_REG_PERI_CRG51, 0x04);
    
    /*  reset ON */
    set_bit(0, regmem);
    
    /* sleep a bit */
    msleep(20);
    
    /* reset OFF */
    clear_bit(0, regmem);
    
    iounmap(regmem);
}

static void _hieth_soft_reset(struct hieth_device *hdev)
{
    /* note it needs it twice */
    set_bit(0, hdev->membase + HIETH_REG_GLB_SOFT_RESET);
    msleep(20);
    clear_bit(0, hdev->membase + HIETH_REG_GLB_SOFT_RESET);
    msleep(20);
    set_bit(0, hdev->membase + HIETH_REG_GLB_SOFT_RESET);
    msleep(20);
    clear_bit(0, hdev->membase + HIETH_REG_GLB_SOFT_RESET);
    msleep(20);
}

static void _hieth_core_clock(struct hieth_device *hdev, bool enable)
{
    void* regmem = 0;
    
    regmem = ioremap(HIETH_REG_PERI_CRG51, 0x04);
    
    if(enable)
    {
        set_bit(1, regmem);
    }
    else
    {
        clear_bit(1, regmem);
    }
    
    iounmap(regmem);
}

static void _hieth_queue_depth_set(struct hieth_device *hdev, unsigned int rx_len, unsigned int tx_len)
{
    uint32_t reg = tx_len | (rx_len << 8);
    iowrite32(reg, hdev->membase + HIETH_REG_UD_GLB_QLEN_SET);
}

static bool _hieth_tx_queue_ready(struct hieth_device *hdev)
{
    return test_bit(24, hdev->membase + HIETH_REG_UD_GLB_ADDRQ_STAT);
}

static bool _hieth_rx_queue_ready(struct hieth_device *hdev)
{
    return test_bit(25, hdev->membase + HIETH_REG_UD_GLB_ADDRQ_STAT);
}

static unsigned int _hieth_get_rx_in_use(struct hieth_device *hdev)
{
    unsigned int reg = ioread32(hdev->membase + HIETH_REG_UD_GLB_QSTAT);
    return (reg & 0x3F00) >> 8;
}

static unsigned int _hieth_get_tx_in_use(struct hieth_device *hdev)
{
    unsigned int reg = ioread32(hdev->membase + HIETH_REG_UD_GLB_QSTAT);
    return reg & 0x3F;
}

static void _hieth_transmit_frame(struct hieth_device *hdev, void* buf, unsigned int len)
{
    /* write physical address */
    iowrite32(virt_to_phys(buf), hdev->membase + HIETH_REG_UD_GLB_EQ_ADDR);
    /* write length, triggers transmission */
    iowrite32((len) & 0x7FF, hdev->membase + HIETH_REG_UD_GLB_EQFRM_LEN);
}

static void _hieth_set_receive_buffer(struct hieth_device *hdev, void* buf)
{
    iowrite32(virt_to_phys(buf), hdev->membase + HIETH_REG_UD_GLB_IQ_ADDR);
}

static unsigned long _hieth_get_received_info(struct hieth_device *hdev, unsigned int *rcv_addr)
{
    unsigned int len = 0;
    unsigned int both = ioread32(hdev->membase + HIETH_REG_UD_GLB_IQFRM_DES);
    len = (both & 0xFFF);
    
    if(rcv_addr != 0)
    {
        *rcv_addr = (both & 0x3F000) << 11;
    }
    
    return len;
}

static void* _hieth_get_rx_frame_addr(struct hieth_device *hdev)
{
    uint32_t phys = ioread32(hdev->membase + HIETH_REG_UD_GLB_RXFRM_SADDR);
    return phys_to_virt(phys);
}

static void _hieth_set_link_status(struct hieth_device *hdev, bool enable)
{
    if(enable) {
        set_bit(1, hdev->membase + HIETH_REG_UD_MAC_PORTSET);
    } else {
        clear_bit(1, hdev->membase + HIETH_REG_UD_MAC_PORTSET);
    }
}

static int _hieth_get_mac_address_internal(struct net_device *dev, char *mac)
{
    struct hieth_device *hdev = netdev_priv(dev);
    unsigned long reg;
    
    reg = ioread32(hdev->membase + HIETH_REG_GLB_HOSTMAC_H16);
    mac[0] = (reg>>8) & 0xff;
    mac[1] = reg & 0xff;    
    
    reg = ioread32(hdev->membase + HIETH_REG_GLB_HOSTMAC_L32);
    mac[2] = (reg>>24) & 0xff;
    mac[3] = (reg>>16) & 0xff;
    mac[4] = (reg>>8) & 0xff;
    mac[5] = reg & 0xff;
    
    return 0;
}

static int _hieth_set_mac_address_internal(struct net_device *dev, char *mac)
{
    struct hieth_device *hdev = netdev_priv(dev);
    unsigned long reg;
    
    reg = mac[1] | (mac[0] << 8);
    iowrite32(reg, hdev->membase + HIETH_REG_GLB_HOSTMAC_H16);
    reg = mac[5] | (mac[4]<<8) | (mac[3]<<16) | (mac[2]<<24);
    iowrite32(reg, hdev->membase + HIETH_REG_GLB_HOSTMAC_L32);
    
    return 0;
}

#endif // _HIETHV100_H_