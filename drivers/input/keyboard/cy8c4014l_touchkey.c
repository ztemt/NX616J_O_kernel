/*
 * Touchkey driver for CYPRESS4000 controller
 *
 * Copyright (C) 2018 nubia
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/i2c/mcs.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/irq.h>
#include <linux/slab.h>
#include <linux/pm.h>
#include <linux/gpio.h>
#include <linux/device.h>
#include <linux/of_gpio.h>
#include <linux/reboot.h>
#include "cy8c4014l_touchkey.h"
#include "cy8c4014l_touchkey_StringImage.h"
#include "cy8c4014l_touchkey_firmware_update.h"


static int cypress_power_on(bool val)
{
	int ret = 0;
	if (val)
	{
	// add power on func
	}
	else
	{
	//add power off func
	}
	return ret;
}

static irqreturn_t cypress_touchkey_interrupt(int irq, void *dev_id)
{
//   struct cypress_touchkey_data *data = dev_id;
//	struct cypress_touchkey_chip *chip = &data->chip;


    struct cypress_info * info = dev_id;
	struct i2c_client *client = info->i2c;
	struct input_dev *input = info->platform_data->input_dev;
	u8 val;
    int cur_value;
    int status;
	static int last_value = 0;

	//dev_info(&client->dev, "cypress key interrupt\n");
	//read key value single keys value are 0x01, 0x02, both pressed keys value is 0x03
	val = i2c_smbus_read_byte_data(client, CYPRESS4000_TOUCHKEY_KEYVAL);
	if (val < 0) {
		dev_err(&client->dev, "cypress key read error [%d]\n", val);
		goto out;
	}
    dev_info(&client->dev, "val = 0x%x\n", val);

    cur_value = (val == 0xff ) ? 0x0 : val;
    status = last_value ^ cur_value;
    if(status & CYPRESS_LEFT_BIT)
    {
        input_report_key(input, CYPRESS_KEY_LEFT, (cur_value & CYPRESS_LEFT_BIT)?1:0);
        input_sync(input);
    }
    if(status & CYPRESS_RIGHT_BIT)
    {
        input_report_key(input, CYPRESS_KEY_RIGHT, (cur_value & CYPRESS_RIGHT_BIT)?1:0);
        input_sync(input);
    }
    last_value = cur_value;

 out:
	return IRQ_HANDLED;
}
static int parse_dt(struct device *dev, struct cypress_platform_data *pdata)
{
    int retval;
	u32 value;
	struct device_node *np = dev->of_node;

	pdata->irq_gpio = of_get_named_gpio_flags(np,
			"touchkey,irq-gpio", 0, NULL);

    retval = of_property_read_u32(np, "touchkey,irq-flags", &value);
	if (retval < 0)
		return retval;
	else
		pdata->irq_flag = value;

    return 0;
}
/*
static void send_key_work(struct work_struct *work)
{
    struct cypress_info * info = container_of(work, struct cypress_info, cypress_work.work);
    struct input_dev * dev = info->platform_data->input_dev;
    dev_info(&info->i2c->dev, "send key left\n");
    input_report_key(dev, CYPRESS_KEY_LEFT, 1);
    input_sync(dev);
    input_report_key(dev, CYPRESS_KEY_LEFT, 0);
    input_sync(dev);
}
*/
static int cypress_touchkey_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct cypress_info *info = NULL;
	struct cypress_platform_data *pdata = client->dev.platform_data;
    struct input_dev *input_dev = NULL;
	int ret = 0;
	int fw_ver;

    dev_info(&client->dev, "Cypress probe start!\n");
	cypress_power_on(true);
	info = kzalloc(sizeof(struct cypress_info), GFP_KERNEL);
	if (!info)
    {
        dev_err(&client->dev, "kzalloc failed!\n");
		ret= -ENOMEM;
		goto info_err;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA)) 
    {
        dev_err(&client->dev, "i2c_check_functionality error!\n");
		ret = -ENODEV;
		goto info_err;
	}

	if (client->dev.of_node)
    {
		pdata = devm_kzalloc(&client->dev,
				sizeof(struct cypress_platform_data),
				GFP_KERNEL);
		if (!pdata)
        {
            dev_err(&client->dev, "Failed to allocate memory\n");
			ret = -ENOMEM;
			goto info_err;
		}

		client->dev.platform_data = pdata;
		ret = parse_dt(&client->dev,pdata);
		if (ret)
        {
            dev_err(&client->dev, "Cypress parse device tree error\n");
			goto data_err;
		}
        else
        {
            dev_info(&client->dev, "Cypress irq gpio:%d,irg flag:0x%x\n",pdata->irq_gpio,pdata->irq_flag);
        }
	}
    else
    {
		pdata = client->dev.platform_data;
		if (!pdata)
        {
            dev_err(&client->dev, "No platform data\n");
			ret = -ENODEV;
			goto info_err;
		}
	}

    input_dev = input_allocate_device();
    if(input_dev == NULL)
    {
       dev_info(&client->dev, "Failed to allocate input device !\n");
       ret= -ENOMEM;
       goto info_err;
    }
    input_dev->name = "cypress_touchkey";
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &client->dev;
    __set_bit(EV_KEY, input_dev->evbit);
    __set_bit(CYPRESS_KEY_LEFT, input_dev->keybit);
    __set_bit(CYPRESS_KEY_RIGHT, input_dev->keybit);

    ret = input_register_device(input_dev);
    if (ret)
    {
        dev_info(&client->dev, "Failed to register input device !\n");
        goto input_err;
    }

    pdata->input_dev = input_dev;
	info->i2c = client;
	info->platform_data = pdata;

	info->irq = gpio_to_irq(pdata->irq_gpio);
    ret = gpio_direction_input(pdata->irq_gpio);
    if(ret)
    {
        dev_err(&client->dev, "Failed to set gpio\n");
		goto data_err;
    }
    i2c_set_clientdata(client, info);

    //read fw version and update
	fw_ver = i2c_smbus_read_byte_data(client, CYPRESS4000_TOUCHKEY_FW);
	dev_err(&client->dev, "current fw version:[0x%x] , new fw version [0x%x]\n", 
	fw_ver,CURRENT_NEW_FIRMWARE_VER);
	if (fw_ver != CURRENT_NEW_FIRMWARE_VER)
	{
		int ret; //0:success
		dev_info(&client->dev, "Ready to update firmware\n");
		ret = cypress_firmware_update(client,stringImage_0, LINE_CNT_0);
		if (ret < 0)
		  dev_err(&client->dev, "cypress Firmware update fail,cur ver is :%x,ret=%x\n", fw_ver,ret);
		else
		  dev_err(&client->dev, "cypress Firmware update success\n");
	}

    ret = request_threaded_irq(info->irq, NULL, cypress_touchkey_interrupt,
				     pdata->irq_flag,
				     client->dev.driver->name, info);
	if (ret)
    {
		dev_err(&client->dev, "Failed to register interrupt\n");
		goto irq_err;
	}

/*******************************send key******************************************/
    //INIT_DELAYED_WORK(&info->cypress_work, send_key_work);
    //schedule_delayed_work(&info->cypress_work, 60 * HZ);
/*************************************************************************/

    return 0;

irq_err:
    free_irq(info->irq, info);
data_err:
    //devm_kfree(pdata);
input_err:
    input_free_device(input_dev);
info_err:
    kfree(info);
    return ret;
}


static int cypress_touchkey_remove(struct i2c_client *client)
{
#if 0
	struct cypress_touchkey_data *data = i2c_get_clientdata(client);

	free_irq(client->irq, data);
	cypress_power_on(false);
	input_unregister_device(data->input_dev);
	kfree(data);
#endif
	return 0;
}

static void cypress_touchkey_shutdown(struct i2c_client *client)
{

	cypress_power_on(false);
}

#ifdef CONFIG_PM_SLEEP
static int cypress_touchkey_suspend(struct device *dev)
{
#if 0
	struct cypress_touchkey_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;

	//Disable the work
	disable_irq(client->irq);

	//Finally turn off the power
	cypress_power_on(false);
#endif

	return 0;
}

static int cypress_touchkey_resume(struct device *dev)
{
#if 0
	struct cypress_touchkey_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;

	//Enable the device first
	cypress_power_on(true);
	// Enable irq again
	enable_irq(client->irq);
#endif

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(cypress_touchkey_pm_ops,
			 cypress_touchkey_suspend, cypress_touchkey_resume);

static const struct i2c_device_id cypress_touchkey_id[] = {
	{ "cypress,touchkey", 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, cypress_touchkey_id);


static struct of_device_id cypress_touchkey_match_table[] = {
        { .compatible = "cypress,touchkey-i2c",},
        {},
};

static struct i2c_driver cypress_touchkey_driver = {
	.driver = {
		.name	= "cypress_touchkey",
        .owner  = THIS_MODULE,
        .of_match_table = cypress_touchkey_match_table,
		.pm	= &cypress_touchkey_pm_ops,
	},
	.probe		= cypress_touchkey_probe,
	.remove		= cypress_touchkey_remove,
	.shutdown       = cypress_touchkey_shutdown,
	.id_table	= cypress_touchkey_id,
};

module_i2c_driver(cypress_touchkey_driver);

/* Module information */
MODULE_AUTHOR("nubia, Inc.");
MODULE_DESCRIPTION("Touchkey driver for cy8c4014lqi-421");
MODULE_LICENSE("GPL");
