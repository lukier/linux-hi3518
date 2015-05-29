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

#include "hiethv100.h"

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
    
    //dev_info(hdev->dev, "LUKIER: %s\n", __PRETTY_FUNCTION__);
    
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
    
    /* Disable EEE */
    clear_bit(0, hdev->membase + HIETH_REG_UD_MAC_EEE_ENA);
    
    /* Disable MAC filter */
    clear_bit(7, hdev->membase + HIETH_REG_GLB_MACTCTRL);
    
    /* Minimum number of packets = 1 */
    iowrite32(0x100003A, hdev->membase + HIETH_REG_UD_GLB_IRQN_SET);
    
    /* Enable UpEther<->CPU */
    set_bit(5, hdev->membase + HIETH_REG_GLB_FWCTRL); // fw2cpu_ena_up
    set_bit(7, hdev->membase + HIETH_REG_GLB_FWCTRL); // fwall2cpu_up
    set_bit(5, hdev->membase + HIETH_REG_GLB_MACTCTRL); // broad2cpu_up
    
    /* Set RX/TX queues */
    _hieth_queue_depth_set(hdev, 1, 1); // FIXME
    printk(KERN_INFO "LUKIER: QueueRegister: 0x%04X\n", ioread32(hdev->membase + HIETH_REG_UD_GLB_QLEN_SET));
    
    /* Set RX buffer */
    _hieth_set_receive_buffer(hdev, hdev->rxbuffer);
    
    hdev->rxlen = 0;
    hdev->skb_in_flight = 0;
    
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
    
    spin_lock(&hdev->lock);
    
    memcpy(hdev->txbuffer, skb->data, skb->len);
    
    _hieth_transmit_frame(hdev, hdev->txbuffer, skb->len);
    
    hdev->skb_in_flight = skb_get(skb);
    
    netif_stop_queue(ndev);
    
    /* enable */
    _hieth_int_enable(hdev, HIETH_FLAG_INT_TXQUEUE_UP);
    
    spin_unlock(&hdev->lock);
    
    //dev_info(hdev->dev, "LUKIER: %s LEN %i\n", __PRETTY_FUNCTION__, skb->len);
    
    dev_kfree_skb(skb);

    return NETDEV_TX_OK;
}

static void hieth_bfproc_recv(unsigned long data)
{
    int ret = 0;
    struct net_device *ndev = (void *)data;
    struct hieth_device *hdev = netdev_priv(ndev);
    struct sk_buff *skb;
    u8* prdbuf;
    u8* org_data = _hieth_get_rx_frame_addr(hdev);
    
    //printk(KERN_INFO "LUKIER RXBUF 0x%04X vs 0x%04X\n", (uint32_t)hdev->rxbuffer, (uint32_t)org_data);
    
    spin_lock(&hdev->lock);
    
    skb = dev_alloc_skb(hdev->rxlen);
    prdbuf = skb_put(skb, hdev->rxlen);
    
    /* Copy packet from buffer */
    memcpy(prdbuf, hdev->rxbuffer, hdev->rxlen);
    
    /* Pass to upper layer */
    skb->protocol = eth_type_trans(skb, ndev);
    netif_rx(skb);
    
    /* Set RX buffer */
    _hieth_set_receive_buffer(hdev, hdev->rxbuffer);
    
    //dev_info(hdev->dev, "LUKIER: RX TASKLET DONE %i\n", hdev->rxlen);
    
    hdev->rxlen = 0;
    
    /* disable */
    _hieth_int_enable(hdev, HIETH_FLAG_INT_RXD_UP);
    
    spin_unlock(&hdev->lock);
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

#if 0    
    spin_lock(&hdev->lock);

    /* Get INT status */
    ints = _hieth_int_status(hdev);
    
    dev_info(hdev->dev, "LUKIER: %s INTSTAT 0x%04X\n", __PRETTY_FUNCTION__, ints);
    
    if(ints & HIETH_FLAG_INT_RXD_UP != 0) {
        hdev->rx_int = 1;
    } else {
        hdev->rx_int = 0;
    }
    
    if(ints & HIETH_FLAG_INT_TX_UP != 0) {
        hdev->tx_done_int = 1;
    } else {
        hdev->tx_done_int = 0;
    }
    
    /* Clear interrupts */
    _hieth_int_clear(hdev, ints);
    
    /* Disable INT */
    _hieth_int_disable(hdev, HIETH_FLAG_INT_RXD_UP | HIETH_FLAG_INT_TX_UP);
    _hieth_int_enable_up_control(hdev, false);
    _hieth_int_enable_all_control(hdev, false);
    
    /* Notify NAPI */
    if (likely(napi_schedule_prep(&hdev->napi)))
        __napi_schedule(&hdev->napi);
    
    spin_unlock(&hdev->lock);
#endif

    /* Mask all interrupts */
    _hieth_int_enable_all_control(hdev, false);
    
    spin_lock_irq(&hdev->lock);
    
    /* Read INT status */
    ints = _hieth_int_status(hdev);
    
    //dev_info(hdev->dev, "LUKIER: %s 0x%02X\n", __PRETTY_FUNCTION__, ints);
    
    /* RX interrupt */
    if((ints & HIETH_FLAG_INT_RXD_UP)) {
        if((_hieth_int_rawstatus(hdev) & HIETH_FLAG_INT_RX_UP) != 0) {
            hdev->rxlen = _hieth_get_received_info(hdev, 0) - 4;
            
            //dev_info(hdev->dev, "LUKIER: INT RXD Received %i\n", hdev->rxlen);
            
            /* clear to notify */
            _hieth_int_clear(hdev, HIETH_FLAG_INT_RX_UP | HIETH_FLAG_INT_RXD_UP);
            
            /* bottom half */
            tasklet_schedule(&hdev->task_recv);
            
            /* disable */
            _hieth_int_disable(hdev, HIETH_FLAG_INT_RXD_UP);
        }
        
        ints &= ~(HIETH_FLAG_INT_RXD_UP | HIETH_FLAG_INT_RX_UP);
    }
    
    /* TX done interrupt */
    if((ints & HIETH_FLAG_INT_TXQUEUE_UP)) {
        //dev_info(hdev->dev, "LUKIER: INT TX DONE\n");
        
        dev_kfree_skb_irq(hdev->skb_in_flight);
        hdev->skb_in_flight = 0;
        
        /* wake queue */
        netif_wake_queue(ndev);
        
        /* clear */
        _hieth_int_clear(hdev, HIETH_FLAG_INT_TXQUEUE_UP | HIETH_FLAG_INT_TX_UP);

        /* disable */
        _hieth_int_disable(hdev, HIETH_FLAG_INT_TXQUEUE_UP);
        
        ints &= ~(HIETH_FLAG_INT_TXQUEUE_UP | HIETH_FLAG_INT_TX_UP);
    }
    
    if (unlikely(ints)) {
        dev_err(hdev->dev, "unknown ints=0x%.8x\n", ints);
        _hieth_int_clear(hdev, ints);
    }
    
    spin_unlock_irq(&hdev->lock);
    
    /* Unmask the all interrupt */
    _hieth_int_enable_all_control(hdev, true);

    return IRQ_HANDLED;
}

#if 0
static void _hieth_handle_xmit(struct net_device *ndev)
{
    struct hieth_device *hdev = netdev_priv(ndev);
    u32 txcidx, *ptxstat, txstat;
    unsigned long flags = 0;
    
    spin_lock_irqsave(&hdev->lock, flags);
    
    if(hdev->tx_done_int == 1)
    {
        if (netif_queue_stopped(ndev))
            netif_wake_queue(ndev);
        
        dev_info(hdev->dev, "LUKIER: %s\n", __PRETTY_FUNCTION__);
        
        hdev->tx_done_int = 0;
    }
    
    spin_unlock_irqrestore(&hdev->lock, flags);
}

static int _hieth_handle_recv(struct net_device *ndev, int budget)
{
    struct hieth_device *hdev = netdev_priv(ndev);
    struct sk_buff *skb;
    unsigned long flags = 0;
    u8 *prdbuf = 0;
    int rx_done = 0;
    unsigned int idx = 0;
    unsigned int len = 0;
    
    spin_lock_irqsave(&hdev->lock, flags);
    
    if(hdev->rx_int == 1)
    {
        len = _hieth_get_received_info(hdev, &idx);
        
        skb = dev_alloc_skb(len);
        prdbuf = skb_put(skb, len);
        
        /* Copy packet from buffer */
        memcpy(prdbuf, hdev->rxbuffer, len);
        
        /* Pass to upper layer */
        skb->protocol = eth_type_trans(skb, ndev);
        netif_receive_skb(skb);
        
        _hieth_int_clear(hdev, HIETH_FLAG_INT_RXD_UP);
        _hieth_int_clear(hdev, HIETH_FLAG_INT_RX_UP);
        
        /* Set RX buffer */
        _hieth_set_receive_buffer(hdev, hdev->rxbuffer);
        
        rx_done = 1;
        
        dev_info(hdev->dev, "LUKIER: %s %i bytes / %i IDX\n", __PRETTY_FUNCTION__, len, idx);
        
        hdev->rx_int = 0;
    }
    
    spin_unlock_irqrestore(&hdev->lock, flags);

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
        _hieth_int_enable(hdev, HIETH_FLAG_INT_RXD_UP | HIETH_FLAG_INT_TX_UP);
        _hieth_int_enable_up_control(hdev, true);
        _hieth_int_enable_all_control(hdev, true);
    }
    
    return rx_done;
}
#endif

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
    
    /* init tasklet */
    hdev->task_recv.next = NULL;
    hdev->task_recv.state = 0;
    hdev->task_recv.func = hieth_bfproc_recv;
    hdev->task_recv.data = (unsigned long)ndev;
    atomic_set(&hdev->task_recv.count, 0);
    
    dev_info(hdev->dev, "LUKIER: %s\n", __PRETTY_FUNCTION__);
    
    if (netif_msg_ifup(hdev)) {
        dev_dbg(hdev->dev, "enabling %s\n", ndev->name);
    }

    /* Enable clock */
    clk_enable(hdev->clk);
    
    /* Power UP */
    hieth_power(hdev, true);
    
    /* Reset and initialize */
    hieth_reset(hdev);
    hieth_init(hdev);
    
    /* Attach the MAC to the PHY */
    hdev->phy_dev = of_phy_connect(ndev, hdev->phy_node, 
                                   &hieth_handle_link_change, 0, 
                                   hdev->phy_interface);
    if (!hdev->phy_dev) {
        netdev_err(ndev, "could not find the PHY!\n");
        ret = -ENODEV;
        goto _error;
    }
    
    /* Resume PHY */
    phy_resume(hdev->phy_dev);
    
    /* mask with MAC supported features */
    hdev->phy_dev->supported &= PHY_BASIC_FEATURES;
    hdev->phy_dev->advertising = hdev->phy_dev->supported;
    
    hdev->link = 0; 
    hdev->speed = 0; 
    hdev->duplex = DUPLEX_UNKNOWN;
    
    /* Force default PHY interface setup in chip, this will probably be changed by the PHY driver */
    hieth_link_changed(hdev);
    
    phy_start(hdev->phy_dev);
    //phy_start_aneg(hdev->phy_dev);
    
    netif_start_queue(ndev);
    //napi_enable(&hdev->napi);
    
    spin_lock_irqsave(&hdev->lock, flags);
    
    // enable interrupts
    _hieth_int_clear(hdev, HIETH_FLAG_INT_MASK);
    _hieth_int_enable(hdev, HIETH_FLAG_INT_RXD_UP | HIETH_FLAG_INT_TX_UP);
    _hieth_int_enable_up_control(hdev, true);
    _hieth_int_enable_all_control(hdev, true);
    
    spin_unlock_irqrestore(&hdev->lock, flags);
    
_error:
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
    //.ndo_set_rx_mode    = hieth_set_rx_mode,
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
    hdev->phy_interface = of_get_phy_mode(np);
    if (hdev->phy_interface < 0) {
        dev_warn(&pdev->dev, "not find phy-mode\n");
        ret = -EINVAL;
        goto out_freeirq;
    }
    
    printk(KERN_INFO "LUKIER PHY INTERFACE %i\n", hdev->phy_interface);
    
    /* get PHY device */
    hdev->phy_node = of_parse_phandle(np, "phy-handle", 0);
    if (!hdev->phy_node) {
        dev_err(&pdev->dev, "no associated PHY\n");
        ret = -ENODEV;
        goto out_freeirq;
    }
    
    printk(KERN_INFO "LUKIER PHY NODE %s / %s\n", hdev->phy_node->full_name, hdev->phy_node->name);
    
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
    //netif_napi_add(ndev, &hdev->napi, hieth_poll, HIETH_NAPI_WEIGHT);
    
    /* Register netdev */
    ret = register_netdev(ndev);
    if (ret) {
        dev_err(&pdev->dev, "Registering netdev failed!\n");
        ret = -ENODEV;
        goto out_freeirq;
    }
    
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