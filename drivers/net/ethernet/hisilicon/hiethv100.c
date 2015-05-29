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
    bool status_change = false;
    unsigned long flags = 0;
    
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
    
    if (status_change)
    {
        hieth_link_changed(hdev);
        phy_print_status(phydev);
    }
    
    spin_unlock_irqrestore(&hdev->lock, flags);
}

static void hieth_power(struct hieth_device *hdev, bool enabled)
{
    _hieth_core_clock(hdev, enabled);
}

static void hieth_reset(struct hieth_device *hdev)
{
    _hieth_core_reset(hdev);
    _hieth_soft_reset(hdev);
}

static int hieth_init(struct hieth_device *hdev)
{  
    int i = 0;
    
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
    _hieth_int_enable_up_control(hdev, false); 
    _hieth_int_disable(hdev, HIETH_FLAG_INT_MASK);
    
    /* Disable VLAN functionality */
    clear_bit(0, hdev->membase + HIETH_REG_GLB_FWCTRL);
    
    /* Disable EEE */
    clear_bit(0, hdev->membase + HIETH_REG_UD_MAC_EEE_ENA);
    
    /* Disable MAC filter */
    clear_bit(7, hdev->membase + HIETH_REG_GLB_MACTCTRL);
    
    /* Do not discard packets */
    iowrite32(0x00, hdev->membase + HIETH_REG_UD_GLB_FC_DROPCTRL);
    
    /* No traffic limits */
    iowrite32(0x00000000, hdev->membase + HIETH_REG_UD_GLB_FC_RXLIMIT);
    
    /* No traffic control timer */
    iowrite32(0x00000000, hdev->membase + HIETH_REG_UD_GLB_FC_TIMECTRL);
    
    /* Minimum number of packets = 1 */
    //NOTE iowrite32(0x100003A, hdev->membase + HIETH_REG_UD_GLB_IRQN_SET);
    
    /* Enable UpEther<->CPU */
    set_bit(5, hdev->membase + HIETH_REG_GLB_FWCTRL); // fw2cpu_ena_up
    set_bit(7, hdev->membase + HIETH_REG_GLB_FWCTRL); // fwall2cpu_up
    set_bit(5, hdev->membase + HIETH_REG_GLB_MACTCTRL); // broad2cpu_up
    set_bit(3, hdev->membase + HIETH_REG_GLB_MACTCTRL); // multi2cpu_up
    set_bit(1, hdev->membase + HIETH_REG_GLB_MACTCTRL); // uni2cpu_up
    
    /* Reset queues */
    _hieth_soft_reset(hdev);
    
    /* Set RX/TX queues */
    _hieth_queue_depth_set(hdev, HIETH_MAX_QUEUE_DEPTH_RX, HIETH_MAX_QUEUE_DEPTH_TX);
    
    hdev->rx_buffer_cur = 0;
    hdev->tx_buffer_cur = 0;
    
    /* Push initial RX buffers */
    for(i = 0 ; i < HIETH_MAX_QUEUE_DEPTH_RX ; ++i)
    {
        _hieth_set_receive_buffer(hdev, hdev->rxbuffers[i]);
        ++hdev->rx_buffer_cur;
    }
    
    /* Clear interrupts */
    _hieth_int_clear(hdev, HIETH_FLAG_INT_MASK);
    
    /* enable receive */
    _hieth_int_enable(hdev, HIETH_FLAG_INT_RXD_UP);
    _hieth_int_enable_up_control(hdev, true);
    _hieth_int_enable_all_control(hdev, true);
    
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
    
    if (!phydev)
        return -ENODEV;
    
    return phy_ethtool_gset(phydev, cmd);
}

static int hieth_set_settings(struct net_device *dev, struct ethtool_cmd *cmd)
{
    struct hieth_device *hdev = netdev_priv(dev);
    struct phy_device *phydev = hdev->phy_dev;
    
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
    
    if (!netif_running(ndev))
        return -EINVAL;
    
    if (!phydev)
        return -ENODEV;
    
    return phy_mii_ioctl(phydev, rq, cmd);
}

static int hieth_set_mac_address(struct net_device *ndev, void *p)
{
    struct sockaddr *addr = p;
    int ret = 0;
    
    if (!is_valid_ether_addr(addr->sa_data))
        return -EADDRNOTAVAIL;
    
    if (netif_running(ndev))
        return -EBUSY;
    
    memcpy(ndev->dev_addr, addr->sa_data, ETH_ALEN);
    
    ret = _hieth_set_mac_address_internal(ndev, addr->sa_data);
    
    return ret;
}

static int hieth_start_xmit(struct sk_buff *skb, struct net_device *ndev)
{
    struct hieth_device *hdev = netdev_priv(ndev);
    unsigned long flags = 0;
    int ret = 0;
    
    spin_lock_irqsave(&hdev->lock, flags);
    
    /* Notify core */
    _hieth_int_clear(hdev, HIETH_FLAG_INT_TXQUEUE_UP);

    /* Can transmit more */
    if(_hieth_tx_queue_ready(hdev) == true)  {
        
        ret = skb->len;
        
        /* copy */
        memcpy(hdev->txbuffers[hdev->tx_buffer_cur], skb->data, skb->len);
        
        /* transmit, extra 4 bytes for CRC */
        _hieth_transmit_frame(hdev, hdev->txbuffers[hdev->tx_buffer_cur], skb->len + 4);
        
        /* increment / wrap around */
        ++hdev->tx_buffer_cur;
        if(hdev->tx_buffer_cur >= HIETH_MAX_QUEUE_DEPTH_TX) {
            hdev->tx_buffer_cur = 0;
        }
        
        /* consume SKB */
        dev_kfree_skb_irq(skb);    
        
        //dev_info(hdev->dev, "LUKIER: %s TX START %i = %i bytes\n", __PRETTY_FUNCTION__, hdev->tx_buffer_cur, ret);
        
        ret = NETDEV_TX_OK;
    }
    else {
        /* Stop incoming packets */
        netif_stop_queue(ndev);
        
        /* enable TX QUEUE INT */
        _hieth_int_enable(hdev, HIETH_FLAG_INT_TXQUEUE_UP);
        
        //dev_info(hdev->dev, "LUKIER: %s TX FULL WAIT %i\n", __PRETTY_FUNCTION__, hdev->tx_buffer_cur);
        
        ret = -EAGAIN;
    }
    
    spin_unlock_irqrestore(&hdev->lock, flags);
    
    return ret;
}

static irqreturn_t hieth_interrupt(int irq, void *dev_id)
{
    struct net_device *ndev = dev_id;
    struct hieth_device *hdev = netdev_priv(ndev);
    unsigned int ints = 0;

    /* Get INT status */
    ints = _hieth_int_status(hdev);
    
    /* RX */
    if((ints & HIETH_FLAG_INT_RXD_UP) != 0) {
        
        /* Clear this one */
        _hieth_int_clear(hdev, HIETH_FLAG_INT_RXD_UP);
        ints &= ~HIETH_FLAG_INT_RXD_UP;
        
        if (likely(napi_schedule_prep(&hdev->napi))) {
            /* Don't need that now */
            _hieth_int_disable(hdev, HIETH_FLAG_INT_RXD_UP);
            
            //dev_info(hdev->dev, "LUKIER: %s RX NEED WORK %i\n", __PRETTY_FUNCTION__, hdev->rx_buffer_cur);
            __napi_schedule(&hdev->napi);
        }
    }
    
    /* TX */
    if((ints & HIETH_FLAG_INT_TXQUEUE_UP) != 0) {
        
        /* Clear this one */
        _hieth_int_clear(hdev, HIETH_FLAG_INT_TXQUEUE_UP);
        ints &= ~HIETH_FLAG_INT_TXQUEUE_UP;
        
        //dev_info(hdev->dev, "LUKIER: %s TX HANDLE %i\n", __PRETTY_FUNCTION__, hdev->tx_buffer_cur);
        
        /* Got space now? */    
        if (unlikely(netif_queue_stopped(hdev->ndev) && (_hieth_tx_queue_ready(hdev) == true))) {
            
            netif_tx_lock(hdev->ndev);
            
            /* Got space now? - double locking */
            if (netif_queue_stopped(hdev->ndev) && (_hieth_tx_queue_ready(hdev) == true)) {
                
                //dev_info(hdev->dev, "LUKIER: %s TX DONE, OK MORE %i\n", __PRETTY_FUNCTION__, hdev->tx_buffer_cur);
                
                /* Can accept more packets */
                netif_wake_queue(hdev->ndev);
                
                /* Don't need that now */
                _hieth_int_disable(hdev, HIETH_FLAG_INT_TXQUEUE_UP);
            }
            
            netif_tx_unlock(hdev->ndev);
        } 
    }
    
    /* Clear interrupts */
    if(ints != 0) {
        //dev_info(hdev->dev, "LUKIER: %s CLEARING REMAIN 0x%04X\n", __PRETTY_FUNCTION__, ints);
        _hieth_int_clear(hdev, ints);
    }

    return IRQ_HANDLED;
}

static int _hieth_handle_recv(struct net_device *ndev, int budget)
{
    struct hieth_device *hdev = netdev_priv(ndev);
    unsigned int raw_status = 0;
    unsigned int rx_done = 0;
    
    while(1)
    {
        if(rx_done >= HIETH_NAPI_WEIGHT - 1) { break; }
        
        raw_status = _hieth_int_rawstatus(hdev);
        
        //dev_info(hdev->dev, "LUKIER: %s RX TRIES WORK %i 0x%04X\n", __PRETTY_FUNCTION__, hdev->rx_buffer_cur, raw_status);
        
        if( ((raw_status & HIETH_FLAG_INT_RX_UP) != 0) || ((raw_status & HIETH_FLAG_INT_RXD_UP) != 0) )
        {
            /* Query incoming packet length & address */
            unsigned int len = _hieth_get_received_info(hdev, NULL) - 4;
            void* data = _hieth_get_rx_frame_addr(hdev);
            
            /* Allocate new SKB, +2 to make IP header L1 cache aligned */
            struct sk_buff *skb = netdev_alloc_skb(ndev, len + 2);
            
            /* align IP header */
            skb_reserve(skb, 2);
            
            /* Proper data size - make room */
            skb_put(skb, len);
            
            /* Copy packet from buffer */
            skb_copy_to_linear_data(skb, data, len);
            
            /* Pass to upper layer */
            skb->protocol = eth_type_trans(skb, ndev);
            netif_receive_skb(skb);
            
            /* Packets processed counter */
            ++rx_done;
            
            //dev_info(hdev->dev, "LUKIER: %s RX DONE %i / %i = %i bytes\n", __PRETTY_FUNCTION__, hdev->rx_buffer_cur, rx_done, len);
            
            /* Notify core that we are done with this packet */
            _hieth_int_clear(hdev, HIETH_FLAG_INT_RX_UP | HIETH_FLAG_INT_RXD_UP);
            
            /* Push more rx-buffers if possible */
            while(_hieth_rx_queue_ready(hdev) == true) {
                ++hdev->rx_buffer_cur;
                if(hdev->rx_buffer_cur >= HIETH_MAX_QUEUE_DEPTH_RX) {
                    hdev->rx_buffer_cur = 0;
                }
               _hieth_set_receive_buffer(hdev, hdev->rxbuffers[hdev->rx_buffer_cur]);
            }
        } else {
            break;
        }
    }
    
    /* Enable RX interrupts */
    _hieth_int_enable(hdev, HIETH_FLAG_INT_RXD_UP);

    return rx_done;
}

static int hieth_poll(struct napi_struct *napi, int budget)
{
    struct hieth_device *hdev = container_of(napi,struct hieth_device, napi);
    struct net_device *ndev = hdev->ndev;
    int rx_done = 0;
    
    rx_done = _hieth_handle_recv(ndev, budget);
    
    if (rx_done < budget) {
        napi_complete(napi);
        
        /* enable RXD INT */
        _hieth_int_enable(hdev, HIETH_FLAG_INT_RXD_UP);
    }
    
    return rx_done;
}

static void hieth_timeout(struct net_device *ndev)
{
    struct hieth_device *hdev = netdev_priv(ndev);
    unsigned long flags = 0;
    int i = 0;
    
    spin_lock_irqsave(&hdev->lock, flags);
    
    /* Disable interrupts */
    _hieth_int_enable_up_control(hdev, false); 
    _hieth_int_disable(hdev, HIETH_FLAG_INT_MASK);
    
    /* Reset queues */
    _hieth_soft_reset(hdev);
    
    /* Set RX/TX queues */
    _hieth_queue_depth_set(hdev, HIETH_MAX_QUEUE_DEPTH_RX, HIETH_MAX_QUEUE_DEPTH_TX);
    
    hdev->rx_buffer_cur = 0;
    hdev->tx_buffer_cur = 0;
    
    /* Push initial RX buffers */
    for(i = 0 ; i < HIETH_MAX_QUEUE_DEPTH_RX ; ++i)
    {
        _hieth_set_receive_buffer(hdev, hdev->rxbuffers[i]);
        ++hdev->rx_buffer_cur;
    }
    
    /* Clear interrupts */
    _hieth_int_clear(hdev, HIETH_FLAG_INT_MASK);
    
    /* enable receive */
    _hieth_int_enable(hdev, HIETH_FLAG_INT_RXD_UP);
    _hieth_int_enable_up_control(hdev, true);
    _hieth_int_enable_all_control(hdev, true);
    
    spin_unlock_irqrestore(&hdev->lock, flags);
}

static int hieth_open(struct net_device *ndev)
{
    struct hieth_device *hdev = netdev_priv(ndev);
    int ret = 0;
    unsigned long flags = 0;
    
    spin_lock_irqsave(&hdev->lock, flags);
    
    //dev_info(hdev->dev, "LUKIER: %s\n", __PRETTY_FUNCTION__);
    
    if (netif_msg_ifup(hdev)) {
        dev_dbg(hdev->dev, "enabling %s\n", ndev->name);
    }

    /* Enable clock */
    clk_prepare_enable(hdev->clk);
    clk_enable(hdev->clk);
    
    /* Power UP */
    hieth_power(hdev, true);
    
    /* Reset and initialize HW */
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
    
    /* Request IRQ */
    if (request_irq(ndev->irq, &hieth_interrupt, 0, ndev->name, ndev))
    {
        ret = -EAGAIN;
        goto _error;
    }
    
    /* mask with MAC supported features */
    hdev->phy_dev->supported &= PHY_BASIC_FEATURES;
    hdev->phy_dev->advertising = hdev->phy_dev->supported;
    
    /* Unknown settings for start */
    hdev->link = 0; 
    hdev->speed = 0; 
    hdev->duplex = DUPLEX_UNKNOWN;
    
    /* Force default PHY interface setup in chip, this will probably be changed by the PHY driver */
    hieth_link_changed(hdev);
    
    /* Start PHY */
    phy_start(hdev->phy_dev); // NOTE phy_start_aneg(hdev->phy_dev);
    
    /* Start packet queue */
    netif_start_queue(ndev);
    
    /* NAPI */
    napi_enable(&hdev->napi);

_error:
    spin_unlock_irqrestore(&hdev->lock, flags);

    return ret;
}

static int hieth_stop(struct net_device *ndev)
{
    struct hieth_device *hdev = netdev_priv(ndev);
    unsigned long flags = 0;
    
    spin_lock_irqsave(&hdev->lock, flags);
    
    /* stop packet queue */
    netif_stop_queue(ndev);
    
    /* Stop PHY */
    phy_stop(hdev->phy_dev);
    
    /* Release interrupt handler */
    free_irq(ndev->irq, ndev);
    
    /* Detach PHY */
    phy_detach(hdev->phy_dev);
    
    /* Disable carrier */
    netif_carrier_off(ndev);
    
    /* Power off */
    hieth_power(hdev, false);
    
    /* Disable clock */
    clk_disable(hdev->clk);
    
    spin_unlock_irqrestore(&hdev->lock, flags);
    
    return 0;
}

static const struct net_device_ops hieth_netdev_ops = {
    
    .ndo_open       = hieth_open,
    .ndo_stop       = hieth_stop,
    .ndo_start_xmit     = hieth_start_xmit,
    .ndo_tx_timeout     = hieth_timeout,
    .ndo_set_mac_address    = hieth_set_mac_address,
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

    /* Get clock for the device */
    hdev->clk = devm_clk_get(&pdev->dev, NULL);
    if (IS_ERR(hdev->clk)) {
        ret = PTR_ERR(hdev->clk);
        goto out_freenetdev;
    }
    
    /* Get platform resources (MEM) */
    hdev->membase = of_iomap(np, 0);
    if (!hdev->membase) {
        dev_err(&pdev->dev, "failed to remap registers\n");
        ret = -ENOMEM;
        goto out_freenetdev;
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
    
    /* get PHY mode (MII/RMII) */
    hdev->phy_interface = of_get_phy_mode(np);
    if (hdev->phy_interface < 0) {
        dev_warn(&pdev->dev, "not find phy-mode\n");
        ret = -EINVAL;
        goto out_iounmap;
    }
   
    /* get PHY device */
    hdev->phy_node = of_parse_phandle(np, "phy-handle", 0);
    if (!hdev->phy_node) {
        dev_err(&pdev->dev, "no associated PHY\n");
        ret = -ENODEV;
        goto out_iounmap;
    }
    
    /* Read MAC-address from DT */
    mac_addr = of_get_mac_address(np);
    if (mac_addr)
    {
        memcpy(ndev->dev_addr, mac_addr, ETH_ALEN);
    }
    
    /* Check if the MAC address is valid */
    if (!is_valid_ether_addr(ndev->dev_addr)) {
        /* Get MAC address from current HW setting (POR state is all zeros, but u-boot maybe) */
        _hieth_get_mac_address_internal(ndev, ndev->dev_addr);
        if (!is_valid_ether_addr(ndev->dev_addr)) { /* no luck, go random */
            eth_hw_addr_random(ndev);
            dev_warn(&pdev->dev, "using random MAC address %pM\n",  ndev->dev_addr);
        }
    }
    
    /* Setup driver functions */
    ndev->watchdog_timeo = msecs_to_jiffies(watchdog);
    ndev->netdev_ops = &hieth_netdev_ops;
    ndev->ethtool_ops = &hieth_ethtool_ops;
    
    /* Done setting ndev, attach to pdev */
    platform_set_drvdata(pdev, ndev);
    
    /* Carrier starts down, phylib will bring it up */
    netif_carrier_off(ndev);
    
    /* Add NAPI */
    netif_napi_add(ndev, &hdev->napi, hieth_poll, HIETH_NAPI_WEIGHT);
    
    /* Make lock */
    spin_lock_init(&hdev->lock);
    
    /* Register netdev */
    ret = register_netdev(ndev);
    if (ret) {
        dev_err(&pdev->dev, "Registering netdev failed!\n");
        ret = -ENODEV;
        goto out_iounmap;
    }
    
    /* Enable wakeup */
    device_init_wakeup(&pdev->dev, 1);
    device_set_wakeup_enable(&pdev->dev, 0);
    
    dev_info(&pdev->dev, "%s: at %p, IRQ %d MAC: %pM\n",
             ndev->name, hdev->membase, ndev->irq, ndev->dev_addr);
    
    return 0;
    
out_iounmap:
    iounmap(hdev->membase);
    
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
    
    /* Unregister netdevice */
    unregister_netdev(ndev);
    
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
    
    if (device_may_wakeup(&pdev->dev))
        enable_irq_wake(ndev->irq);
    
    if (ndev) {
        if (netif_running(ndev)) {
            
            /* Disable carrier */
            netif_carrier_off(ndev);
            
            /* PHY suspend */
            phy_suspend(hdev->phy_dev);
            
            /* Detach device */
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
            
            /* Attach device */
            netif_device_attach(ndev);
            
            /* PHY resume */
            phy_resume(hdev->phy_dev);
            
            /* Reset and initialize */
            hieth_reset(hdev);
            hieth_init(hdev);
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