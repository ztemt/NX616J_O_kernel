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

#define LOG_TAG "DOUBLE_PROXIMITY"
#define SENSOR_LOG_LEVEL 1

//#define DEBUG_ON //DEBUG SWITCH
//nubia add debug log mesage
//#define TMD3702_DEBUG_REGISTER_RW

#define SENSOR_LOG_ERROR(fmt, args...) printk(KERN_ERR "[%s] [%s:%d] " fmt,\
					LOG_TAG, __FUNCTION__, __LINE__, ##args)
#define SENSOR_LOG_INFO(fmt, args...)  printk(KERN_INFO "[%s] [%s:%d] "  fmt,\
					LOG_TAG, __FUNCTION__, __LINE__, ##args)
#define SENSOR_LOG_DEBUG_IF(en, fmt, args...) \
do { \
    if (en>=SENSOR_LOG_LEVEL) { \
        printk(KERN_ERR "[%s] [%s:%d] " fmt,\
					LOG_TAG, __FUNCTION__, __LINE__, ##args); \
    }; \
} while (0)

#define SENSOR_LOG_IF(en, fmt, args...) \
do { \
    if (en>=SENSOR_LOG_LEVEL) { \
        printk(KERN_ERR "[%s] [%s:%d] " fmt,\
					LOG_TAG, __FUNCTION__, __LINE__, ##args); \
    }; \
} while (0)

#ifdef  DEBUG_ON
#define SENSOR_LOG_DEBUG(fmt, args...) printk(KERN_DEBUG "[%s] [%s:%d] "  fmt,\
					LOG_TAG, __FUNCTION__, __LINE__, ##args)
#else
#define SENSOR_LOG_DEBUG(fmt, args...)
#endif
//#define ABI_SET_GET_REGISTERS

#ifdef AMS_MUTEX_DEBUG
#define AMS_MUTEX_LOCK(m) { \
        printk(KERN_INFO "%s: Mutex Lock\n", __func__); \
        mutex_lock(m); \
    }
#define AMS_MUTEX_UNLOCK(m) { \
        printk(KERN_INFO "%s: Mutex Unlock\n", __func__); \
        mutex_unlock(m); \
    }
#else
#define AMS_MUTEX_LOCK(m) { \
        mutex_lock(m); \
    }
#define AMS_MUTEX_UNLOCK(m) { \
        mutex_unlock(m); \
    }
#endif

#define SENSOR_LOG_FILE__ strrchr(__FILE__, '/') ? (strrchr(__FILE__, '/') + 1) : __FILE__



struct virtual_proximity_data {
	struct device *proximity_dev;
	struct input_dev *ps_input_dev;
	struct mutex rw_lock;

	volatile u8 fb_status;
    u8 report_fb_status;
    int report_value;
    volatile bool enable_status;
	bool flag_prox_debug;

#ifdef CONFIG_NUBIA_SWITCH_LCD
    struct notifier_block ps_fb_notify;
#endif
    u8 debug_level;
};

enum{
	ERR_NAKED_CAL = 1,
	ERR_THRES_CAL,
	ERR_FILE_OPS,
	ERR_DEV_OPS,
	ERR_OTHER,
};

//extern struct virtual_proximity_data *vp_data;
extern struct input_dev *virtual_ps_input_dev;
extern void virtual_proximity_report_event(int value, unsigned int code);


#endif
