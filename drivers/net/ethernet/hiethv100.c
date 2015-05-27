#include <linux/kernel.h>
#include <linux/module.h>

/* hardware */
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>

/* network driver classes */
#include <linux/platform_device.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/mii.h>
#include <linux/phy.h>

/* device tree */
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_mdio.h>
#include <linux/of_net.h>
#include <linux/of_platform.h>

#define DRV_NAME            "hieth"
#define DRV_VERSION         "v100"

/* Transmit timeout, default 5 seconds. */
static int watchdog = 5000;
module_param(watchdog, int, 0400);
MODULE_PARM_DESC(watchdog, "transmit timeout in milliseconds");

#define HIETH_MAX_QUEUE_DEPTH   64
#define HIETH_MAX_FRAME_SIZE    (SZ_1K*2)
#define HIETH_NAPI_WEIGHT 16

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
    int                     phy_mode;
    phy_interface_t         phy_interface;
    
    u32                     msg_enable;
    
    unsigned int            link;
    unsigned int            speed;
    unsigned int            duplex;
    
    struct napi_struct      napi;
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

static unsigned int _hieth_int_status(struct hieth_device *hdev)
{
    unsigned int ret = 0;
    
    ret = ioread32(hdev->membase + HIETH_REG_GLB_IRQ_STAT) & HIETH_FLAG_INT_MASK;
    
    return ret;
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
    
    dev_info(hdev->dev, "LUKIER: %s\n", __PRETTY_FUNCTION__);
    
    regmem = ioremap(HIETH_REG_PERI_CRG51, 0x04);
    
    spin_lock_irqsave(&hdev->lock, flags);
    
    /* Enable RMII */
    if(hdev->phy_mode == PHY_INTERFACE_MODE_RMII) {
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
    
    dev_info(hdev->dev, "LUKIER: %s\n", __PRETTY_FUNCTION__);
    
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
    dev_info(hdev->dev, "LUKIER: %s\n", __PRETTY_FUNCTION__);
    
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

    dev_info(hdev->dev, "LUKIER: %s\n", __PRETTY_FUNCTION__);
    
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

static void _hieth_set_link_status(struct hieth_device *hdev, bool enable)
{
    if(enable) {
        set_bit(1, hdev->membase + HIETH_REG_UD_MAC_PORTSET);
    } else {
        clear_bit(1, hdev->membase + HIETH_REG_UD_MAC_PORTSET);
    }
}

static void hieth_link_changed(struct hieth_device *hdev)
{
    dev_info(hdev->dev, "LUKIER: %s LINK CHANGED %i %i %i\n", __PRETTY_FUNCTION__, hdev->link, hdev->duplex, hdev->speed);
    
    if (hdev->duplex == DUPLEX_FULL) {
        set_bit(0, hdev->membase + HIETH_REG_UD_MAC_PORTSET);
    } else {
        clear_bit(0, hdev->membase + HIETH_REG_UD_MAC_PORTSET);
    }
    
    if (hdev->speed == SPEED_100) {
        set_bit(2, hdev->membase + HIETH_REG_UD_MAC_PORTSET);
    } else {
        clear_bit(2, hdev->membase + HIETH_REG_UD_MAC_PORTSET);
    }
    
    _hieth_set_link_status(hdev, hdev->link);
}

static void hieth_handle_link_change(struct net_device *dev)
{
    struct hieth_device *hdev = netdev_priv(dev);
    struct phy_device *phydev = hdev->phy_dev;
    unsigned long flags;
    bool status_change = false;
    
    dev_info(hdev->dev, "LUKIER: %s\n", __PRETTY_FUNCTION__);
    
    spin_lock_irqsave(&hdev->lock, flags);
    
    if (phydev->link) {
        if ((hdev->speed != phydev->speed) || 
            (hdev->duplex != phydev->duplex)) {
            hdev->speed = phydev->speed;
            hdev->duplex = phydev->duplex;
            status_change = true;
        }
    }
    
    if (phydev->link != hdev->link) {
        if (!phydev->link) {
            hdev->speed = 0;
            hdev->duplex = DUPLEX_UNKNOWN;
        }
        
        hdev->link = phydev->link;
        
        status_change = true;
    }
    
    spin_unlock_irqrestore(&hdev->lock, flags);
    
    if (status_change)
    {
        hieth_link_changed(hdev);
        phy_print_status(phydev);
    }
}

static int _hieth_get_mac_address_internal(struct net_device *dev, char *mac)
{
    struct hieth_device *hdev = netdev_priv(dev);
    unsigned long reg;
    
    dev_info(hdev->dev, "LUKIER: %s\n", __PRETTY_FUNCTION__);
    
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

static void hieth_power(struct hieth_device *hdev, bool enabled)
{
    unsigned long flags = 0;
    
    spin_lock_irqsave(&hdev->lock, flags);
    
    _hieth_core_clock(hdev, enabled);
    
    spin_unlock_irqrestore(&hdev->lock, flags);
}

static void hieth_reset(struct hieth_device *hdev)
{
    unsigned long flags = 0;
    
    spin_lock_irqsave(&hdev->lock, flags);
    
    _hieth_core_reset(hdev);
    _hieth_soft_reset(hdev);
    
    spin_unlock_irqrestore(&hdev->lock, flags);
}

static int hieth_init(struct hieth_device *hdev)
{
    unsigned long flags = 0;
    
    spin_lock_irqsave(&hdev->lock, flags);
    
    /* Endian mode */
    _hieth_set_endian(hdev, true, true);
    
    /* Disable link for now */
    _hieth_set_link_status(hdev, false);
    
    /* Set CPU negotiation */
    _hieth_set_cpu_negotiation(hdev, true);
    
    /* Setup network interface for RMII or MII mode */
    _hieth_set_mii_mode(hdev);
    
    /* Clear all interrupt status */
    _hieth_int_clear(hdev, HIETH_FLAG_INT_MASK);
    
    /* Disable interrupts */
    _hieth_int_enable_up_control(hdev, false); // ien_up
    _hieth_int_disable(hdev, HIETH_FLAG_INT_MASK);
    
    /* Disable VLAN functionality */
    clear_bit(0, hdev->membase + HIETH_REG_GLB_FWCTRL);
    
    /* Enable UpEther<->CPU */
    set_bit(5, hdev->membase + HIETH_REG_GLB_FWCTRL); // fw2cpu_ena_up
    set_bit(7, hdev->membase + HIETH_REG_GLB_FWCTRL); // fwall2cpu_up
    set_bit(5, hdev->membase + HIETH_REG_GLB_MACTCTRL); // broad2cpu_up
    
    spin_unlock_irqrestore(&hdev->lock, flags);
    
    return 0;
}

static void hieth_get_drvinfo(struct net_device *dev,
                              struct ethtool_drvinfo *info)
{
    strlcpy(info->driver, DRV_NAME, sizeof(DRV_NAME));
    strlcpy(info->version, DRV_VERSION, sizeof(DRV_VERSION));
    strlcpy(info->bus_info, dev_name(&dev->dev), sizeof(info->bus_info));
}

static int hieth_get_settings(struct net_device *dev, struct ethtool_cmd *cmd)
{
    struct hieth_device *hdev = netdev_priv(dev);
    struct phy_device *phydev = hdev->phy_dev;
    
    dev_info(hdev->dev, "LUKIER: %s\n", __PRETTY_FUNCTION__);
    
    if (!phydev)
        return -ENODEV;
    
    return phy_ethtool_gset(phydev, cmd);
}

static int hieth_set_settings(struct net_device *dev, struct ethtool_cmd *cmd)
{
    struct hieth_device *hdev = netdev_priv(dev);
    struct phy_device *phydev = hdev->phy_dev;
    
    dev_info(hdev->dev, "LUKIER: %s\n", __PRETTY_FUNCTION__);
    
    if (!phydev)
        return -ENODEV;
    
    return phy_ethtool_sset(phydev, cmd);
}

static const struct ethtool_ops hieth_ethtool_ops = {
    .get_drvinfo    = hieth_get_drvinfo,
    .get_settings   = hieth_get_settings,
    .set_settings   = hieth_set_settings,
    .get_link   = ethtool_op_get_link,
};

static int hieth_ioctl(struct net_device *ndev, struct ifreq *rq, int cmd)
{
    struct hieth_device *hdev = netdev_priv(ndev);
    struct phy_device *phydev = hdev->phy_dev;
    
    dev_info(hdev->dev, "LUKIER: %s\n", __PRETTY_FUNCTION__);
    
    if (!netif_running(ndev))
        return -EINVAL;
    
    if (!phydev)
        return -ENODEV;
    
    return phy_mii_ioctl(phydev, rq, cmd);
}

static int hieth_set_mac_address(struct net_device *ndev, void *p)
{
    struct sockaddr *addr = p;
    struct hieth_device *hdev = netdev_priv(ndev);
    int ret = 0;
    unsigned long flags;
    
    dev_info(hdev->dev, "LUKIER: %s\n", __PRETTY_FUNCTION__);
    
    if (!is_valid_ether_addr(addr->sa_data))
        return -EADDRNOTAVAIL;
    
    if (netif_running(ndev))
        return -EBUSY;
    
    memcpy(ndev->dev_addr, addr->sa_data, ETH_ALEN);
    
    spin_lock_irqsave(&hdev->lock, flags);
    
    ret = _hieth_set_mac_address_internal(ndev, addr->sa_data);
    
    spin_unlock_irqrestore(&hdev->lock, flags);
    
    return ret;
}

static int hieth_start_xmit(struct sk_buff *skb, struct net_device *ndev)
{
    struct hieth_device *hdev = netdev_priv(ndev);
    
    dev_info(hdev->dev, "LUKIER: %s\n", __PRETTY_FUNCTION__);
    
    return NETDEV_TX_OK;
}

static void hieth_set_rx_mode(struct net_device *ndev)
{
    struct hieth_device *hdev = netdev_priv(ndev);
    
    dev_info(hdev->dev, "LUKIER: %s\n", __PRETTY_FUNCTION__);
}

static irqreturn_t hieth_interrupt(int irq, void *dev_id)
{
    struct net_device *ndev = dev_id;
    struct hieth_device *hdev = netdev_priv(ndev);
    unsigned int ints = 0;
    
    spin_lock(&hdev->lock);
    
    /* Get INT status */
    ints = _hieth_int_status(hdev);
    /* Clear interrupts */
    _hieth_int_clear(hdev, ints);
    
    /* Disable INT */
    _hieth_int_disable(hdev, HIETH_FLAG_INT_RXD_UP | HIETH_FLAG_INT_TXQUEUE_UP);
    _hieth_int_enable_up_control(hdev, false);
    _hieth_int_enable_all_control(hdev, false);
    
    /* Notify NAPI */
    if (likely(napi_schedule_prep(&hdev->napi)))
        __napi_schedule(&hdev->napi);
    
    spin_unlock(&hdev->lock);
    
#if 0
    /* Mask all interrupts */
    _hieth_int_enable_all_control(hdev, false);
    
    /* Read INT status */
    ints = _hieth_int_status(hdev);
    
    dev_info(hdev->dev, "LUKIER: %s 0x%02X\n", __PRETTY_FUNCTION__, ints);
    
    /* RX interrupt */
    if((ints & HIETH_FLAG_INT_RXD_UP)) {
        
    }
    
    /* TX interrupt */
    if((ints & HIETH_FLAG_INT_TXQUEUE_UP)) {
        /* disable */
        _hieth_int_disable(hdev, HIETH_FLAG_INT_TXQUEUE_UP);
        
        /* wake queue */
        // TODO
    }
    
    if (unlikely(ints)) {
        dev_err(hdev->dev, "unknown ints=0x%.8x\n", ints);
        _hieth_int_clear(hdev, ints);
    }
    
    /* Unmask the all interrupt */
    _hieth_int_enable_all_control(hdev, true);
#endif

    return IRQ_HANDLED;
}

static void _hieth_handle_xmit(struct net_device *ndev)
{
    struct hieth_device *hdev = netdev_priv(ndev);
    u32 txcidx, *ptxstat, txstat;
    
#if 0
    txcidx = readl(LPC_ENET_TXCONSUMEINDEX(pldat->net_base));
    while (pldat->last_tx_idx != txcidx) {
        unsigned int skblen = pldat->skblen[pldat->last_tx_idx];
        
        /* A buffer is available, get buffer status */
        ptxstat = &pldat->tx_stat_v[pldat->last_tx_idx];
        txstat = *ptxstat;
        
        /* Next buffer and decrement used buffer counter */
        pldat->num_used_tx_buffs--;
        pldat->last_tx_idx++;
        if (pldat->last_tx_idx >= ENET_TX_DESC)
            pldat->last_tx_idx = 0;
        
        /* Update collision counter */
        ndev->stats.collisions += TXSTATUS_COLLISIONS_GET(txstat);
        
        /* Any errors occurred? */
        if (txstat & TXSTATUS_ERROR) {
            if (txstat & TXSTATUS_UNDERRUN) {
                /* FIFO underrun */
                ndev->stats.tx_fifo_errors++;
            }
            if (txstat & TXSTATUS_LATECOLL) {
                /* Late collision */
                ndev->stats.tx_aborted_errors++;
            }
            if (txstat & TXSTATUS_EXCESSCOLL) {
                /* Excessive collision */
                ndev->stats.tx_aborted_errors++;
            }
            if (txstat & TXSTATUS_EXCESSDEFER) {
                /* Defer limit */
                ndev->stats.tx_aborted_errors++;
            }
            ndev->stats.tx_errors++;
        } else {
            /* Update stats */
            ndev->stats.tx_packets++;
            ndev->stats.tx_bytes += skblen;
        }
        
        txcidx = readl(LPC_ENET_TXCONSUMEINDEX(pldat->net_base));
    }
    
    if (pldat->num_used_tx_buffs <= ENET_TX_DESC/2) {
        if (netif_queue_stopped(ndev))
            netif_wake_queue(ndev);
    }
#endif
}

static int _hieth_handle_recv(struct net_device *ndev, int budget)
{
    struct hieth_device *hdev = netdev_priv(ndev);
    struct sk_buff *skb;
    u32 rxconsidx, len, ethst;
    struct rx_status_t *prxstat;
    u8 *prdbuf;
    int rx_done = 0;
    
#if 0
    /* Get the current RX buffer indexes */
    rxconsidx = readl(LPC_ENET_RXCONSUMEINDEX(pldat->net_base));
    while (rx_done < budget && rxconsidx !=
        readl(LPC_ENET_RXPRODUCEINDEX(pldat->net_base))) {
        /* Get pointer to receive status */
        prxstat = &pldat->rx_stat_v[rxconsidx];
    len = (prxstat->statusinfo & RXSTATUS_SIZE) + 1;
    
    /* Status error? */
    ethst = prxstat->statusinfo;
    if ((ethst & (RXSTATUS_ERROR | RXSTATUS_STATUS_ERROR)) ==
        (RXSTATUS_ERROR | RXSTATUS_RANGE))
        ethst &= ~RXSTATUS_ERROR;
    
    if (ethst & RXSTATUS_ERROR) {
        int si = prxstat->statusinfo;
        /* Check statuses */
        if (si & RXSTATUS_OVERRUN) {
            /* Overrun error */
            ndev->stats.rx_fifo_errors++;
        } else if (si & RXSTATUS_CRC) {
            /* CRC error */
            ndev->stats.rx_crc_errors++;
        } else if (si & RXSTATUS_LENGTH) {
            /* Length error */
            ndev->stats.rx_length_errors++;
        } else if (si & RXSTATUS_ERROR) {
            /* Other error */
            ndev->stats.rx_length_errors++;
        }
        ndev->stats.rx_errors++;
    } else {
        /* Packet is good */
        skb = dev_alloc_skb(len);
        if (!skb) {
            ndev->stats.rx_dropped++;
        } else {
            prdbuf = skb_put(skb, len);
            
            /* Copy packet from buffer */
            memcpy(prdbuf, pldat->rx_buff_v +
            rxconsidx * ENET_MAXF_SIZE, len);
            
            /* Pass to upper layer */
            skb->protocol = eth_type_trans(skb, ndev);
            netif_receive_skb(skb);
            ndev->stats.rx_packets++;
            ndev->stats.rx_bytes += len;
        }
    }
    
    /* Increment consume index */
    rxconsidx = rxconsidx + 1;
    if (rxconsidx >= ENET_RX_DESC)
        rxconsidx = 0;
    writel(rxconsidx,
           LPC_ENET_RXCONSUMEINDEX(pldat->net_base));
    rx_done++;
        }
#endif
        return rx_done;
}

static int hieth_poll(struct napi_struct *napi, int budget)
{
    struct hieth_device *hdev = container_of(napi,struct hieth_device, napi);
    struct net_device *ndev = hdev->ndev;
    int rx_done = 0;
    struct netdev_queue *txq = netdev_get_tx_queue(ndev, 0);
    
    __netif_tx_lock(txq, smp_processor_id());
   _hieth_handle_xmit(ndev);
    __netif_tx_unlock(txq);
    
    rx_done = _hieth_handle_recv(ndev, budget);
    
    if (rx_done < budget) {
        napi_complete(napi);
        _hieth_int_enable(hdev, HIETH_FLAG_INT_RXD_UP | HIETH_FLAG_INT_TXQUEUE_UP);
        _hieth_int_enable_up_control(hdev, true);
        _hieth_int_enable_all_control(hdev, true);
    }
    
    return rx_done;
}

static void hieth_timeout(struct net_device *ndev)
{
    struct hieth_device *hdev = netdev_priv(ndev);
    
    dev_info(hdev->dev, "LUKIER: %s\n", __PRETTY_FUNCTION__);
}

static int hieth_open(struct net_device *ndev)
{
    struct hieth_device *hdev = netdev_priv(ndev);
    unsigned long flags = 0;
    int ret = 0;
    
    dev_info(hdev->dev, "LUKIER: %s\n", __PRETTY_FUNCTION__);
    
    if (netif_msg_ifup(hdev)) {
        dev_dbg(hdev->dev, "enabling %s\n", ndev->name);
    }

    /* Enable clock */
    clk_enable(hdev->clk);
    
    /* Power UP */
    hieth_power(hdev, true);
    
    /* Resume PHY */
    phy_resume(hdev->phy_dev);
    
    /* Reset and initialize */
    hieth_reset(hdev);
    hieth_init(hdev);
    
    //phy_start(hdev->phy_dev);
    phy_start_aneg(hdev->phy_dev);
    
    netif_start_queue(ndev);
    napi_enable(&hdev->napi);
    
    spin_lock_irqsave(&hdev->lock, flags);
    
    // enable interrupts
    _hieth_int_enable(hdev, HIETH_FLAG_INT_RXD_UP | HIETH_FLAG_INT_TXQUEUE_UP);
    _hieth_int_enable_up_control(hdev, true);
    _hieth_int_enable_all_control(hdev, true);
    
    spin_unlock_irqrestore(&hdev->lock, flags);
    
    return ret;
}

static int hieth_stop(struct net_device *ndev)
{
    struct hieth_device *hdev = netdev_priv(ndev);
    unsigned long flags = 0;
    
    if (netif_msg_ifdown(hdev))
    {
        dev_dbg(hdev->dev, "shutting down %s\n", ndev->name);
    }
    
    netif_stop_queue(ndev);
    
    /* Stop PHY */
    if(hdev->phy_dev)
        phy_stop(hdev->phy_dev);
    
    spin_unlock_irqrestore(&hdev->lock, flags);
    
    netif_carrier_off(ndev);
    hieth_power(hdev, false);
    
    spin_unlock_irqrestore(&hdev->lock, flags);
    
    /* Disable clock */
    clk_disable(hdev->clk);
    
    return 0;
}

static const struct net_device_ops hieth_netdev_ops = {
    
    .ndo_open       = hieth_open,
    .ndo_stop       = hieth_stop,
    .ndo_start_xmit     = hieth_start_xmit,
    .ndo_tx_timeout     = hieth_timeout,
    .ndo_set_mac_address    = hieth_set_mac_address,
    .ndo_set_rx_mode    = hieth_set_rx_mode,
    .ndo_do_ioctl       = hieth_ioctl,
    .ndo_change_mtu     = eth_change_mtu,
    .ndo_validate_addr  = eth_validate_addr,
};

static int hieth_netdev_probe(struct platform_device *pdev)
{
    struct device_node *np = pdev->dev.of_node;
    struct hieth_device *hdev;
    struct net_device *ndev;
    int ret = 0;
    const char *mac_addr;
    
    /* Allocate net driver data structure */
    ndev = alloc_etherdev(sizeof(struct hieth_device));
    if (!ndev) {
        dev_err(&pdev->dev, "could not allocate device.\n");
        ret = -ENOMEM;
        goto out;
    }
    
    SET_NETDEV_DEV(ndev, &pdev->dev);
    
    hdev = netdev_priv(ndev);
    memset(hdev, 0, sizeof(*hdev));
    
    hdev->dev = &pdev->dev;
    hdev->ndev = ndev;
    hdev->pdev = pdev;
    
    spin_lock_init(&hdev->lock);
    
    /* Get clock for the device */
    hdev->clk = devm_clk_get(&pdev->dev, NULL);
    if (IS_ERR(hdev->clk)) {
        ret = PTR_ERR(hdev->clk);
        goto out_freenetdev;
    }
    
    /* Enable MAC clock */
    clk_prepare_enable(hdev->clk);
    clk_enable(hdev->clk);
    
    /* Get platform resources (MEM) */
    hdev->membase = of_iomap(np, 0);
    if (!hdev->membase) {
        dev_err(&pdev->dev, "failed to remap registers\n");
        ret = -ENOMEM;
        goto out_disable_clocks;
    }
    
    /* fill in parameters for net-dev structure */
    ndev->base_addr = (unsigned long)hdev->membase;
    
    /* Get platform resources (IRQ) */
    ndev->irq = irq_of_parse_and_map(np, 0);
    if (ndev->irq == -ENXIO) {
        netdev_err(ndev, "No irq resource\n");
        ret = ndev->irq;
        goto out_iounmap;
    }
    
    /* Request IRQ */
    if (request_irq(ndev->irq, &hieth_interrupt, 0, ndev->name, ndev))
    {
        ret = -EAGAIN;
        goto out_iounmap;
    }
    
    /* get PHY mode (MII/RMII) */
    hdev->phy_mode = of_get_phy_mode(np);
    if (hdev->phy_mode < 0) {
        dev_warn(&pdev->dev, "not find phy-mode\n");
        ret = -EINVAL;
        goto out_freeirq;
    }
    
    /* get PHY device */
    hdev->phy_node = of_parse_phandle(np, "phy-handle", 0);
    if (!hdev->phy_node) {
        dev_err(&pdev->dev, "no associated PHY\n");
        ret = -ENODEV;
        goto out_freeirq;
    }
    
    /* Read MAC-address from DT */
    mac_addr = of_get_mac_address(np);
    if (mac_addr)
    {
        memcpy(ndev->dev_addr, mac_addr, ETH_ALEN);
    }
    
    /* Check if the MAC address is valid */
    if (!is_valid_ether_addr(ndev->dev_addr)) {
        /* Get MAC address from current HW setting (POR state is all zeros) */
        _hieth_get_mac_address_internal(ndev, ndev->dev_addr);
        if (!is_valid_ether_addr(ndev->dev_addr)) {
            eth_hw_addr_random(ndev);
            dev_warn(&pdev->dev, "using random MAC address %pM\n", 
                     ndev->dev_addr);
        }
    }
    
    /* Allocate buffers */
    // TODO
    
    /* Power UP */
    hieth_power(hdev, true);
    
    /* Reset the ethernet controller */
    hieth_reset(hdev);
    
    /* Initialize MAC */
    hieth_init(hdev);
    
    /* Setup driver functions */
    ndev->watchdog_timeo = msecs_to_jiffies(watchdog);
    ndev->netdev_ops = &hieth_netdev_ops;
    ndev->ethtool_ops = &hieth_ethtool_ops;
    
    platform_set_drvdata(pdev, ndev);
    
    /* Carrier starts down, phylib will bring it up */
    netif_carrier_off(ndev);
    
    /* Add NAPI */
    netif_napi_add(ndev, &hdev->napi, hieth_poll, HIETH_NAPI_WEIGHT);
    
    /* Register netdev */
    ret = register_netdev(ndev);
    if (ret) {
        dev_err(&pdev->dev, "Registering netdev failed!\n");
        ret = -ENODEV;
        goto out_freeirq;
    }
    
    /* Attach the MAC to the PHY */
    hdev->phy_dev = of_phy_connect(ndev, hdev->phy_node, 
                                   &hieth_handle_link_change, 0, 
                                   hdev->phy_interface);
    if (!hdev->phy_dev) {
        netdev_err(ndev, "could not find the PHY\n");
        ret = -ENODEV;
        goto out_unregister;
    }
    
    /* mask with MAC supported features */
    hdev->phy_dev->supported &= PHY_BASIC_FEATURES;
    hdev->phy_dev->advertising = hdev->phy_dev->supported;
    
    hdev->link = 0; 
    hdev->speed = 0; 
    hdev->duplex = DUPLEX_UNKNOWN;
    
    /* Force default PHY interface setup in chip, this will probably be changed by the PHY driver */
    hieth_link_changed(hdev);
    
    device_init_wakeup(&pdev->dev, 1);
    device_set_wakeup_enable(&pdev->dev, 0);
    
    dev_info(&pdev->dev, "%s: at %p, IRQ %d MAC: %pM\n",
             ndev->name, hdev->membase, ndev->irq, ndev->dev_addr);
    
    return 0;
    
out_unregister:
    unregister_netdev(ndev);
out_freeirq:
    free_irq(ndev->irq, ndev);
out_iounmap:
    iounmap(hdev->membase);
out_disable_clocks:
    clk_disable(hdev->clk);
    clk_put(hdev->clk);
out_freenetdev:
    free_netdev(ndev);
out:
    dev_err(&pdev->dev, "not found (%d).\n", ret);
    
    return ret;
}

static int hieth_netdev_remove(struct platform_device *pdev)
{
    struct net_device *ndev = platform_get_drvdata(pdev);
    struct hieth_device *hdev = netdev_priv(ndev);
    
    printk(KERN_INFO "LUKIER: %s\n", __PRETTY_FUNCTION__);
        
    /* Unregister netdevice */
    unregister_netdev(ndev);
    
    // TODO FIXME remove allocated memory
    
    /* Power down */
    hieth_power(hdev, false);
    
    /* Disable clocks */
    clk_disable(hdev->clk);
    clk_put(hdev->clk);
    
    /* Release netdevice */
    free_netdev(ndev);
    
    dev_info(&pdev->dev, "released and freed device\n");
    
    return 0;
}

#ifdef CONFIG_PM
static int hieth_suspend(struct platform_device *pdev, pm_message_t state)
{
    struct net_device *ndev = platform_get_drvdata(pdev);
    struct hieth_device *hdev = netdev_priv(ndev);
    
    dev_info(hdev->dev, "LUKIER: %s\n", __PRETTY_FUNCTION__);
    
    if (device_may_wakeup(&pdev->dev))
        enable_irq_wake(ndev->irq);
    
    if (ndev) {
        if (netif_running(ndev)) {
            
            netif_carrier_off(ndev);
            netif_device_detach(ndev);
            
            /* Power down */
            hieth_power(hdev, false);
            
            /* Disable clock */
            clk_disable(hdev->clk);
        }
    }
    
    return 0;
}

static int hieth_resume(struct platform_device *pdev)
{    
    struct net_device *ndev = platform_get_drvdata(pdev);
    struct hieth_device *hdev = 0;
    
    dev_info(&pdev->dev, "LUKIER: %s\n", __PRETTY_FUNCTION__);
    
    if (device_may_wakeup(&pdev->dev)) {
        disable_irq_wake(ndev->irq);
    }
    
    if (ndev) {
        if (netif_running(ndev)) {
            hdev = netdev_priv(ndev);
            
            /* Enable interface clock */
            clk_enable(hdev->clk);
            
            /* Power UP */
            hieth_power(hdev, true);
            
            /* Reset and initialize */
            hieth_reset(hdev);
            hieth_init(hdev);
            
            netif_device_attach(ndev);
        }
    }
    
    return 0;
}
#endif

static const struct of_device_id hieth_mac_match[] = {
    { .compatible = "hisilicon,hieth-mac" },
    { }
};

MODULE_DEVICE_TABLE(of, hieth_mac_match);

static struct platform_driver hieth_mac_driver = {
    .probe  = hieth_netdev_probe,
    .remove = hieth_netdev_remove,
#ifdef CONFIG_PM
    .suspend = hieth_suspend,
    .resume = hieth_resume,
#endif 
    .driver = {
        .name       = DRV_NAME,
        .owner      = THIS_MODULE,
        .of_match_table = hieth_mac_match,
    },
};
module_platform_driver(hieth_mac_driver);

MODULE_DESCRIPTION("Hisilicon ETH MAC driver");
MODULE_AUTHOR("Robert Lukierski <robert@lukierski.eu>");
MODULE_LICENSE("GPL");

/* vim: set ts=8 sw=8 tw=78: */