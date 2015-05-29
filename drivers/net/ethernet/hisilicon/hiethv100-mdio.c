/*
 * HiSilicon Hi3518 ETH MDIO interface driver
 *
 * Copyright 2015 Robert Lukierski <robert@lukierski.eu>
 *
 * Copyright (C) 2015 Robert Lukierski
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>

#include <linux/phy.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>

#include <linux/of_address.h>
#include <linux/of_mdio.h>
#include <linux/of_gpio.h>

#define MDIO_RWCTRL         (0x00)
#define MDIO_RO_DATA        (0x04)
#define UD_MDIO_PHYADDR     (0x08)
#define UD_MDIO_RO_STAT     (0x0C)
#define UD_MDIO_ANEG_CTRL   (0x10)
#define UD_MDIO_IRQENA      (0x04)

#define MDIO_TIMEOUT		(msecs_to_jiffies(100))

struct hieth_mdio_data {
	void __iomem		*membase;
	struct regulator	*regulator;
    int                 reset_gpios;
    enum of_gpio_flags  reset_flags;
};

#define HIETH_MDIO_FRQDIV 2

static int hieth_mdio_read(struct mii_bus *bus, int mii_id, int regnum)
{
	struct hieth_mdio_data *data = bus->priv;
	unsigned long timeout_jiffies;
	uint32_t value = 0;

    /* issue the phy address and reg */
    value = ((mii_id & 0x1F) << 8) | ((HIETH_MDIO_FRQDIV & 0x7) << 5) | (regnum & 0x1F);
    iowrite32(value, data->membase + MDIO_RWCTRL);

	/* Wait read complete */
    timeout_jiffies = jiffies + MDIO_TIMEOUT;
    while ((ioread32(data->membase + MDIO_RWCTRL) & 0x8000) == 0) 
    {
        if (time_is_before_jiffies(timeout_jiffies))
            return -ETIMEDOUT;
        msleep(1);
    }

    /* and read data */
    value = ioread32(data->membase + MDIO_RO_DATA);

	return value;
}

static int hieth_mdio_write(struct mii_bus *bus, int mii_id, int regnum,
			    u16 value)
{
	struct hieth_mdio_data *data = bus->priv;
	unsigned long timeout_jiffies;
    uint32_t out_value = 0;
    
	/* Wait MDIO ready */
    timeout_jiffies = jiffies + MDIO_TIMEOUT;
    while ((ioread32(data->membase + MDIO_RWCTRL) & 0x8000) == 0) 
    {
        if (time_is_before_jiffies(timeout_jiffies))
            return -ETIMEDOUT;
        msleep(1);
    }

	/* and write data */
    out_value = (value << 16) | (1 << 13) | ((mii_id & 0x1F) << 8) | ((HIETH_MDIO_FRQDIV & 0x7) << 5) | (regnum & 0x1F);
    iowrite32(out_value, data->membase + MDIO_RWCTRL);
    
    /* Wait write complete */
    timeout_jiffies = jiffies + MDIO_TIMEOUT;
    while ((ioread32(data->membase + MDIO_RWCTRL) & 0x8000) == 0) 
    {
        if (time_is_before_jiffies(timeout_jiffies))
            return -ETIMEDOUT;
        msleep(1);
    }

	return 0;
}

static int hieth_mdio_reset(struct mii_bus *bus)
{
    struct hieth_mdio_data *data = bus->priv;
    
    iowrite32(0x00008000, data->membase + MDIO_RWCTRL); 
    iowrite32(0x00000001, data->membase + UD_MDIO_PHYADDR); 
    iowrite32(0x04631EA9, data->membase + UD_MDIO_ANEG_CTRL); 
    iowrite32(0x00000000, data->membase + UD_MDIO_IRQENA); 
    
    if (gpio_is_valid(data->reset_gpios)) 
    {
        if(data->reset_flags == OF_GPIO_ACTIVE_LOW)
        {
            gpio_set_value(data->reset_gpios, 0);
        }
        else
        {
            gpio_set_value(data->reset_gpios, 1);
        }
        
        msleep(4);
        
        if(data->reset_flags == OF_GPIO_ACTIVE_LOW)
        {
            gpio_set_value(data->reset_gpios, 1);
        }
        else
        {
            gpio_set_value(data->reset_gpios, 0);
        }
    }
    
    return 0;
}

static int hieth_mdio_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct mii_bus *bus;
	struct hieth_mdio_data *data;
	struct resource *res;
	int ret, i;

	bus = mdiobus_alloc_size(sizeof(*data));
	if (!bus)
		return -ENOMEM;

	bus->name = "hieth_mii_bus";
	bus->read = &hieth_mdio_read;
	bus->write = &hieth_mdio_write;
    bus->reset = &hieth_mdio_reset;
	snprintf(bus->id, MII_BUS_ID_SIZE, "%s-mii", dev_name(&pdev->dev));
	bus->parent = &pdev->dev;

	bus->irq = devm_kzalloc(&pdev->dev, sizeof(int) * PHY_MAX_ADDR,
			GFP_KERNEL);
	if (!bus->irq) {
		ret = -ENOMEM;
		goto err_out_free_mdiobus;
	}

	for (i = 0; i < PHY_MAX_ADDR; i++)
		bus->irq[i] = PHY_POLL;

	data = bus->priv;
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	data->membase = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(data->membase)) {
		ret = PTR_ERR(data->membase);
		goto err_out_free_mdiobus;
	}

	data->regulator = devm_regulator_get(&pdev->dev, "phy");
	if (IS_ERR(data->regulator)) {
		if (PTR_ERR(data->regulator) == -EPROBE_DEFER)
			return -EPROBE_DEFER;

		dev_info(&pdev->dev, "no regulator found\n");
	} else {
		ret = regulator_enable(data->regulator);
		if (ret)
			goto err_out_free_mdiobus;
	}
	
	data->reset_gpios = of_get_named_gpio_flags(np, "reset-gpios", 0, 
                                                &data->reset_flags);
    if (gpio_is_valid(data->reset_gpios)) {
        ret = devm_gpio_request_one(&pdev->dev, data->reset_gpios, 
                                    data->reset_flags, "phy_reset");
        if (ret) {
            dev_err(&pdev->dev, "failed to request reset gpio %d: %d\n",
                    data->reset_gpios, ret);
            return -ENODEV;
        }
    }
    else
    {
        data->reset_gpios = -1;
    }

	ret = of_mdiobus_register(bus, np);
	if (ret < 0)
		goto err_out_disable_regulator;

	platform_set_drvdata(pdev, bus);
    
    printk(KERN_INFO "LUKIER HIETH MDIO PROBE OK\n");

	return 0;

err_out_disable_regulator:
	regulator_disable(data->regulator);
err_out_free_mdiobus:
	mdiobus_free(bus);
	return ret;
}

static int hieth_mdio_remove(struct platform_device *pdev)
{
	struct mii_bus *bus = platform_get_drvdata(pdev);
    
	mdiobus_unregister(bus);
	mdiobus_free(bus);

	return 0;
}

static const struct of_device_id hieth_mdio_dt_ids[] = {
    { .compatible = "hisilicon,hieth-mdio" },
	{ }
};
MODULE_DEVICE_TABLE(of, hieth_mdio_dt_ids);

static struct platform_driver hieth_mdio_driver = {
	.probe = hieth_mdio_probe,
	.remove = hieth_mdio_remove,
	.driver = {
		.name = "hieth-mdio",
		.of_match_table = hieth_mdio_dt_ids,
	},
};

module_platform_driver(hieth_mdio_driver);

MODULE_DESCRIPTION("Hisilicon ETH MDIO interface driver");
MODULE_AUTHOR("Robert Lukierski <robert@lukierski.eu>");
MODULE_LICENSE("GPL");
