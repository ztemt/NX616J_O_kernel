/*
 * This file is part of the virtual_proximity sensor driver.
 * virtual_proximity is combined proximity, and VCSEL.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 *Reversion
 *
 *
 *
 
when         	who         		Remark : what, where, why          		version
-----------   	------------     	-----------------------------------   	------------------
2015/11/17	Simon Hsueh	& Allen Hsiao		For virtual_proximity interrupt mode and oil alg   	v1.1.1
==========================================================================================
*/
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/string.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/uaccess.h>
#include <asm/atomic.h>
#include "virtual_proximity.h"

#define PROX_DRV_NAME "virtual_proximity"
#define INPUT_NAME_PS       "proximity"
#define DRIVER_VERSION "1.3.0"
#define LOG_TAG "VIRTUAL_PROXIMITY"
#define SENSOR_LOG_ERROR(fmt, args...) printk(KERN_ERR "[%s] [%s:%d] " fmt,\
					LOG_TAG, __FUNCTION__, __LINE__, ##args)
#define SENSOR_LOG_INFO(fmt, args...)  printk(KERN_INFO "[%s] [%s:%d] "  fmt,\
					LOG_TAG, __FUNCTION__, __LINE__, ##args)

static dev_t const virtual_proximity_dev_t = MKDEV(MISC_MAJOR, 101);
static struct class         *proximity_class;
static struct virtual_proximity_data *g_proximity;
/*----------------------------------------------------------------------------*/
void virtual_proximity_send_data(int value)
{
	struct virtual_proximity_data *data = g_proximity;

	if(value != 3 && value != 10)
		SENSOR_LOG_ERROR("send failed for prox value invalid\n");
	else
	{
		data->ps_status = value;
		virtual_proximity_report_event(data);
	}

	return;
}
EXPORT_SYMBOL(virtual_proximity_send_data);

static void virtual_proximity_report_event(struct virtual_proximity_data *data)
{
	//SENSOR_LOG_INFO("ps_status = %d pre_ps_status = %d \n", data->ps_status, data->pre_ps_status);
	if (data->ps_status != data->pre_ps_status) {
		SENSOR_LOG_INFO("ps_status = %d pre_ps_status = %d \n", data->ps_status, data->pre_ps_status);
		input_report_rel(data->ps_input_dev, REL_RZ, data->ps_status);
		input_sync(data->ps_input_dev);

		data->pre_ps_status = data->ps_status;
	}

	return;
}

static ssize_t tmd2725_prox_flush_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct virtual_proximity_data *data = dev_get_drvdata(dev);
	input_report_rel(data->ps_input_dev, REL_RZ, 10);
	input_sync(data->ps_input_dev);

	SENSOR_LOG_INFO("tmd2725_prox_flush_show\n");
	
	return sprintf(buf, "%s\n", "virtual proximity");
}

/* Device init */
static ssize_t virtual_proximity_store_dev_init(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	return count;
}

static int near_count;
static int far_count;
static ssize_t virtual_proximity_store_value(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
    struct virtual_proximity_data *data = dev_get_drvdata(dev);
    u8 value;
    value = simple_strtoul(buf, NULL, 10 );
    
    //printk("ultrasonic proximity value = %d\n", value);
    if(value == 3 )
    {
        near_count ++;
        far_count = 0;
    }
    else if(value == 10 )
    {
        far_count ++;
        near_count = 0;
    }
    //SENSOR_LOG_INFO("ultrasonic value = %d, near_count=%d, far_count=%d\n", value, near_count, far_count);
    if(near_count == 3)
    {
        near_count = 0;
        data->ps_status = 3;
        virtual_proximity_report_event(data);
    }
    if(far_count == 4)
    {
        far_count = 0;
        data->ps_status = 10;
        virtual_proximity_report_event(data);
    }
    /*
	if(value != 3 && value != 10)
		SENSOR_LOG_ERROR("write failed for prox value invalid\n");
	else
	{
		data->ps_status = value;
		virtual_proximity_report_event(data);
	}
	*/
	return count;
}

static ssize_t virtual_proximity_store_reset(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	struct virtual_proximity_data *data = dev_get_drvdata(dev);
	u8 value;
	value = simple_strtoul(buf, NULL, 10 );
	SENSOR_LOG_INFO("value = %d, near_count=%d, far_count=%d\n", value, near_count, far_count);
	data->pre_ps_status = value;
    near_count = 0;
    far_count = 0;

	input_report_rel(data->ps_input_dev, REL_RZ, 10);
	input_sync(data->ps_input_dev);

	return count;
}

static ssize_t virtual_proximity_show_reset(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	struct virtual_proximity_data *data = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", data->pre_ps_status);
}


static struct device_attribute attrs_prox_device[] = {
	__ATTR(prox_init, 0200, NULL, virtual_proximity_store_dev_init),
	__ATTR(prox_value_a, 0200, NULL, virtual_proximity_store_value),
	__ATTR(prox_value, 0440, tmd2725_prox_flush_show, NULL),
	__ATTR(reset, 0200, virtual_proximity_show_reset, virtual_proximity_store_reset),
};

static int create_sysfs_interfaces(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attrs_prox_device); i++)
	{
		if (device_create_file(dev, attrs_prox_device + i))
			return -ENODEV;
	}
	return 0;
}

static void remove_sysfs_interfaces(struct device *dev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(attrs_prox_device); i++)
		device_remove_file(dev, attrs_prox_device + i);
	return;
}

static const struct i2c_device_id virtual_proximity_id[] = {
	{ PROX_DRV_NAME, 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, virtual_proximity_id);

static struct of_device_id proximity_match_table[] = {
	{ .compatible = "nubia,virtual_proximity",},
	{},
};

static struct i2c_driver virtual_proximity = {
	.driver = {
		.name	= PROX_DRV_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = proximity_match_table,
	},
	.probe	= virtual_proximity_probe,
	.remove	= virtual_proximity_remove,
	.id_table = virtual_proximity_id,
};
/*
 * I2C init/probing/exit functions
 */
static int virtual_proximity_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct virtual_proximity_data *data = NULL;
	int err = 0;

	SENSOR_LOG_INFO("probe start\n");

	data = kzalloc(sizeof(struct virtual_proximity_data), GFP_KERNEL);
	if (!data) {
		SENSOR_LOG_ERROR("kzalloc virtual_proximity_data failed\n");
		err = -ENOMEM;
		goto exit;
	}
	//mutex_init(&data->i2c_lock);
	mutex_init(&data->dev_lock);
	data->client = client;
	i2c_set_clientdata(client, data);
	data->flag_prox_debug = false;
	data->pre_ps_status = NONE;

	proximity_class = class_create(THIS_MODULE, "proximity");
	data->proximity_dev = device_create(proximity_class, NULL, virtual_proximity_dev_t,
		&virtual_proximity ,"proximity");
	if (IS_ERR(data->proximity_dev)) {
		SENSOR_LOG_ERROR("device_create proximity failed\n");
		goto create_proximity_dev_failed;
	}
	dev_set_drvdata(data->proximity_dev, data);
	err = create_sysfs_interfaces(data->proximity_dev);
	if (err < 0) {
		SENSOR_LOG_ERROR("create sysfs interfaces failed\n");
		goto create_sysfs_interface_failed;
	}
	/* allocate proximity input_device */
	data->ps_input_dev = input_allocate_device();
	if (IS_ERR_OR_NULL(data->ps_input_dev)) {
		err = -ENOMEM;
		SENSOR_LOG_ERROR("could not allocate input device\n");
		goto allocate_input_device_failed;
	}
	input_set_drvdata(data->ps_input_dev, data);
	data->ps_input_dev->name = INPUT_NAME_PS;
	data->ps_input_dev->id.bustype = BUS_I2C;
	set_bit(EV_REL, data->ps_input_dev->evbit);
	set_bit(REL_RZ,  data->ps_input_dev->relbit);
	err = input_register_device(data->ps_input_dev);
	if (err < 0) {
		SENSOR_LOG_ERROR("could not register input device\n");
		err = -ENOMEM;
		goto register_input_device_failed;
	}

	g_proximity = data;

	SENSOR_LOG_INFO("probe ok.\n");

	return 0;
register_input_device_failed:
	input_unregister_device(data->ps_input_dev);
allocate_input_device_failed:
	input_free_device(data->ps_input_dev);
create_sysfs_interface_failed:
	remove_sysfs_interfaces(data->proximity_dev);
create_proximity_dev_failed:
	data->proximity_dev = NULL;
	device_destroy(proximity_class, virtual_proximity_dev_t);
	class_destroy(proximity_class);
exit:
	return err;
}

static int virtual_proximity_remove(struct i2c_client *client)
{
	struct virtual_proximity_data *data = i2c_get_clientdata(client);
	struct device *dev = data->proximity_dev;

	input_unregister_device(data->ps_input_dev);
	input_free_device(data->ps_input_dev);
	remove_sysfs_interfaces(dev);
	//mutex_destroy(&data->i2c_lock);
	mutex_destroy(&data->dev_lock);
	kfree(data);

	return 0;
}

static int __init virtual_proximity_init(void)
{
	SENSOR_LOG_INFO("virtual proximity init\n");
	return i2c_add_driver(&virtual_proximity);
}

static void __exit virtual_proximity_exit(void)
{

	i2c_del_driver(&virtual_proximity);
}

MODULE_AUTHOR("Peripherial team, NUBIA");
MODULE_DESCRIPTION("Vitual proxomity sensor driver.");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(DRIVER_VERSION);

module_init(virtual_proximity_init);
module_exit(virtual_proximity_exit);


