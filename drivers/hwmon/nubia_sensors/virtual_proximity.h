/*
 *  virtual_proximity.h - Linux kernel modules for proximity sensor
 *
 *  Copyright (c) 2013, All rights reserved.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 and
 *  only version 2 as published by the Free Software Foundation.

 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifndef __VIRTUAL_PROXIMITY_H__
#define __VIRTUAL_PROXIMITY_H__

#include <linux/types.h>
#include <linux/gpio.h>
#include <linux/workqueue.h>

enum {
	NONE = 0,
	FAR = 10,
	NEAR = 3,
};


struct virtual_proximity_data {
	struct i2c_client *client;
	struct device *proximity_dev;
	struct input_dev *ps_input_dev;
	struct workqueue_struct *irq_work_queue;
	struct work_struct irq_dwork;
	//struct mutex i2c_lock;
	struct mutex dev_lock;

	u8 pre_ps_status;
	u8 ps_status;
	bool flag_prox_debug;
};

static int virtual_proximity_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int virtual_proximity_remove(struct i2c_client *client);
static void virtual_proximity_report_event(struct virtual_proximity_data *data);
#endif
