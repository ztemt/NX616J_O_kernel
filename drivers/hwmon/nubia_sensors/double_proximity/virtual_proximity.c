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

when         	who         		Remark : what, where, why          							version
-----------   	------------     	-----------------------------------   					    ------------------
2018/7/11       GaoKuan                 For set_proximity front or back mode                        v1.0
2018/8/1        Xuxiaohua               Add calibration node                                        v1.1
2018/8/8        GaoKuan                 And volatile type for fb_status and enable_status           v1.2
2018/8/13       GaoKuan                 Change prox front or back status,When LCD switch            V1.3


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
#include "ams_tmd3702.h"
#include "txc_pa224.h"

#ifdef CONFIG_NUBIA_SWITCH_LCD
#include <linux/notifier.h>
#include <linux/fb.h>
#include <linux/msm_drm_notify.h>
#endif


#define VIR_PROX_DRV_NAME "proximity"
#define VIR_PROXNAME_NAME "proximity"
#define INPUT_NAME_VIR_PS       "proximity"
#define DRIVER_VERSION "1.0"

#define PA224_SYS_PATH "/sys/class/proximity_back/proximity_back/"
#define PA224_SYS_ENABLE_PATH "/sys/class/proximity_back/proximity_back/enable"

#define PA224_SYS_prox_offset "/sys/class/proximity_back/proximity_back/prox_offset"
#define PA224_SYS_debug "/sys/class/proximity_back/proximity_back/debug"
#define PA224_SYS_FLUSH_PATH "/sys/class/proximity_back/proximity_back/prox_value"
#define PA224_SYS_prox_offset_cal  "/sys/class/proximity_back/proximity_back/prox_offset_cal"
#define PA224_SYS_prox_thres_min "/sys/class/proximity_back/proximity_back/prox_thres_min"
#define PA224_SYS_prox_thres_max "/sys/class/proximity_back/proximity_back/prox_thres_max"
#define PA224_SYS_prox_thres "/sys/class/proximity_back/proximity_back/prox_thres"
#define PA224_SYS_prox_debug "/sys/class/proximity_back/proximity_back/prox_debug"
#define PA224_SYS_prox_uncover_min  "/sys/class/proximity_back/proximity_back/prox_uncover_min"
#define PA224_SYS_prox_uncover_max  "/sys/class/proximity_back/proximity_back/prox_uncover_max"
#define PA224_SYS_prox_first_event  "/sys/class/proximity_back/proximity_back/prox_first_event"

#define TMD3702_SYS_PATH "/sys/class/proximity_front/proximity_front/"
#define TMD3702_SYS_ENABLE_PATH "/sys/class/proximity_front/proximity_front/enable"
#define TMD3702_SYS_INIT_PATH "/sys/class/proximity_front/proximity_front/prox_init"
#define TMD3702_SYS_FLUSH_PATH "/sys/class/proximity_front/proximity_front/prox_value"

#define TMD3702_SYS_prox_offset  "/sys/class/proximity_front/proximity_front/prox_offset"
#define TMD3702_SYS_debug  "/sys/class/proximity_front/proximity_front/debug"
#define TMD3702_SYS_prox_offset_cal  "/sys/class/proximity_front/proximity_front/prox_offset_cal"
#define TMD3702_SYS_prox_thres_min  "/sys/class/proximity_front/proximity_front/prox_thres_min"
#define TMD3702_SYS_prox_thres_max  "/sys/class/proximity_front/proximity_front/prox_thres_max"
#define TMD3702_SYS_prox_thres  "/sys/class/proximity_front/proximity_front/prox_thres"
#define TMD3702_SYS_prox_debug  "/sys/class/proximity_front/proximity_front/prox_debug"
#define TMD3702_SYS_prox_uncover_min  "/sys/class/proximity_front/proximity_front/prox_uncover_min"
#define TMD3702_SYS_prox_uncover_max  "/sys/class/proximity_front/proximity_front/prox_uncover_max"
#define TMD3702_SYS_prox_first_event  "/sys/class/proximity_front/proximity_front/prox_first_event"

#define SET_PROXIMITY_FB_PATH  "/sys/class/proximity/proximity/proximity_fb"

#define VIR_PROX_CHIP_NAME "virtual proximity"



#define PROX_FRONT 1
#define PROX_BACK  2
#define BUF_MAX_SIZE  20

static dev_t  virtual_proximity_dev_t;
static struct class  *virtual_proximity_class;

static struct virtual_proximity_data *vp_data;
struct input_dev *virtual_ps_input_dev;


/*----------------------------------------------------------------------------*/

void virtual_proximity_report_event(int value,unsigned int code)
{

    struct virtual_proximity_data *data = vp_data;

    AMS_MUTEX_LOCK(&data->rw_lock);
	data->report_value = value;
	//SENSOR_LOG_INFO("input report value = %d code = %d \n", data->report_value, code);
	input_report_rel(data->ps_input_dev, code, data->report_value);
	input_sync(data->ps_input_dev);
    AMS_MUTEX_UNLOCK(&data->rw_lock);
}

static void virtual_proximity_enable(int fb_type)
{
    char enable_buf[2] = {'1', '\0'};
    char disable_buf[2] = {'0', '\0'};

    SENSOR_LOG_INFO("fb_type is %d \n",fb_type);
    switch (fb_type)
    {
        case PROX_FRONT:
            sensor_write_file(PA224_SYS_ENABLE_PATH, disable_buf, sizeof(disable_buf));
            sensor_write_file(TMD3702_SYS_ENABLE_PATH, enable_buf, sizeof(enable_buf));
            break;

        case PROX_BACK:
            sensor_write_file(TMD3702_SYS_ENABLE_PATH, disable_buf, sizeof(disable_buf));
            sensor_write_file(PA224_SYS_ENABLE_PATH, enable_buf, sizeof(enable_buf));
            break;

        default:
			SENSOR_LOG_ERROR("enable fail Prox type error");
			break;
    }
}

static void virtual_proximity_disable(void)
{
    char disable_buf[2] = {'0','\0'};
    sensor_write_file(TMD3702_SYS_ENABLE_PATH, disable_buf, sizeof(disable_buf));
    sensor_write_file(PA224_SYS_ENABLE_PATH, disable_buf, sizeof(disable_buf));
}

static int  set_proximity_fb(struct virtual_proximity_data *data, u8 value)
{
	char init_buf[2] = {'1', '\0'};

	SENSOR_LOG_INFO("proximity_fb value = %d\n", value);
	if(data->fb_status != value && (value == PROX_FRONT || value == PROX_BACK))
	{
		AMS_MUTEX_LOCK(&data->rw_lock);
		data->fb_status = value;
		if (data->fb_status == PROX_FRONT)
			sensor_write_file(TMD3702_SYS_INIT_PATH, init_buf, sizeof(init_buf));
		if(data->enable_status == true)
			virtual_proximity_enable(data->fb_status);
		AMS_MUTEX_UNLOCK(&data->rw_lock);
	}
	else if (value != PROX_FRONT && value != PROX_BACK)
	{
		SENSOR_LOG_ERROR("value is not legal \n");
		return -EINVAL;
	}
	return 0;
}
static ssize_t proximity_fb_store(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	struct virtual_proximity_data *data = dev_get_drvdata(dev);
	u8 value;
	value = simple_strtoul(buf, NULL, 10 );

	if (set_proximity_fb(data, value) < 0)
		return -EINVAL;
	return count;
}

static ssize_t proximity_fb_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	struct virtual_proximity_data *data = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", data->fb_status);
}
 static ssize_t virtual_proximity_enable_store(struct device *dev,
                struct device_attribute *attr,
                const char *buf, size_t size)
{
    int ret;
    int enable;
    struct virtual_proximity_data *data = dev_get_drvdata(dev);

    ret = kstrtoint(buf, 0, &enable);
    if (ret)
        return -EINVAL;
    SENSOR_LOG_INFO(" virtual_proximity enable is %d \n", enable);
    switch(enable)
    {
        case 0:
            AMS_MUTEX_LOCK(&data->rw_lock);
            virtual_proximity_disable();
            data->enable_status = false;
            AMS_MUTEX_UNLOCK(&data->rw_lock);
            break;

        case 1:
            AMS_MUTEX_LOCK(&data->rw_lock);
            virtual_proximity_enable(data->fb_status);
            data->enable_status = true;
            AMS_MUTEX_UNLOCK(&data->rw_lock);
            break;
        default:
            return -EINVAL;
    }
	/*
	If HAL need proximity front or back status, open it.
	And HAL need add process REL_RX event code
	*/
#ifdef REPORT_FB_STATUS
	virtual_proximity_report_event(data->fb_status, REL_RX);
#endif
    return size;
}
static ssize_t virtual_proximity_enable_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
    int ret;
    u8 enable;
	struct virtual_proximity_data *data = dev_get_drvdata(dev);

    switch(data->fb_status)
    {
        case PROX_FRONT:
            // read  PROX_FRONT enable status
            ret = sensor_read_file(TMD3702_SYS_ENABLE_PATH, &enable,1);
            SENSOR_LOG_INFO("PROX_FRONT read enable is %c\n",enable);
            break;

        case PROX_BACK:
            // read  PROX_BACK enable status
            ret = sensor_read_file(PA224_SYS_ENABLE_PATH, &enable, 1);
            SENSOR_LOG_INFO("PROX_BACK read enable is %c\n",enable);
            break;
        default:
            return -EINVAL;

    }
    if (ret < 0) {
		SENSOR_LOG_ERROR("read file fail\n");
		return ret;
	}
    return sprintf(buf, "%c\n", enable);

}

static ssize_t virtual_prox_min_thres_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
    int ret;
    int thre_min;
    char rbuf[BUF_MAX_SIZE];
	struct virtual_proximity_data *data = dev_get_drvdata(dev);

    switch(data->fb_status)
    {
        case PROX_FRONT:
            // read  PROX_FRONT enable status
            ret = sensor_read_file(TMD3702_SYS_prox_thres_min, rbuf,sizeof(rbuf));
            sscanf(rbuf, "%d", &thre_min);
            SENSOR_LOG_IF(data->debug_level,"PROX_FRONT read enable is %d\n",thre_min);
            break;

        case PROX_BACK:
            // read  PROX_BACK enable status
            ret = sensor_read_file(PA224_SYS_prox_thres_min, rbuf,sizeof(rbuf));
            sscanf(rbuf, "%d", &thre_min);
            SENSOR_LOG_IF(data->debug_level,"PROX_BACK read enable is %d\n",thre_min);
            break;
        default:
            return -EINVAL;

    }
    if (ret < 0) {
		SENSOR_LOG_ERROR("read file fail\n");
		return ret;
	}
    return sprintf(buf, "%d\n", thre_min);

}
static ssize_t virtual_prox_max_thres_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
    int ret;
    int thre_max;
    char rbuf[BUF_MAX_SIZE];
	struct virtual_proximity_data *data = dev_get_drvdata(dev);

    switch(data->fb_status)
    {
        case PROX_FRONT:
            // read  PROX_FRONT enable status
            ret = sensor_read_file(TMD3702_SYS_prox_thres_max, rbuf,sizeof(rbuf));
            sscanf(rbuf, "%d", &thre_max);
            SENSOR_LOG_IF(data->debug_level,"PROX_FRONT read enable is %d\n",thre_max);
            break;

        case PROX_BACK:
            // read  PROX_BACK enable status
            ret = sensor_read_file(PA224_SYS_prox_thres_max, rbuf,sizeof(rbuf));
            sscanf(rbuf, "%d", &thre_max);
            SENSOR_LOG_IF(data->debug_level,"PROX_BACK read enable is %d\n",thre_max);
            break;
        default:
            return -EINVAL;

    }
    if (ret < 0) {
		SENSOR_LOG_ERROR("read file fail\n");
		return ret;
	}
    return sprintf(buf, "%d\n", thre_max);

}
static ssize_t virtual_prox_thres_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
    int ret;
    int thre_max;
    char rbuf[BUF_MAX_SIZE];
	struct virtual_proximity_data *data = dev_get_drvdata(dev);

    switch(data->fb_status)
    {
        case PROX_FRONT:
            // read  PROX_FRONT enable status
            ret = sensor_read_file(TMD3702_SYS_prox_thres, rbuf,sizeof(rbuf));
            sscanf(rbuf, "%d", &thre_max);
            SENSOR_LOG_IF(data->debug_level,"PROX_FRONT read enable is %d\n",thre_max);
            break;

        case PROX_BACK:
            // read  PROX_BACK enable status
            ret = sensor_read_file(PA224_SYS_prox_thres, rbuf,sizeof(rbuf));
            sscanf(rbuf, "%d", &thre_max);
            SENSOR_LOG_IF(data->debug_level,"PROX_BACK read enable is %d\n",thre_max);
            break;
        default:
            return -EINVAL;

    }
    if (ret < 0) {
		SENSOR_LOG_ERROR("read file fail\n");
		return ret;
	}
    return sprintf(buf, "%d\n", thre_max);

}
static ssize_t virtual_prox_thres_store(struct device *dev,
                struct device_attribute *attr,
                const char *buf, size_t size)
{
    int ret;
    int val = 0;
    char init_buf[BUF_MAX_SIZE] = {'1', '\0'};
    struct virtual_proximity_data *data = dev_get_drvdata(dev);

    ret = kstrtoint(buf, 0, &val);
    sprintf(init_buf, "%d\n", val);
    switch(data->fb_status)
    {
        case PROX_FRONT:
            // read  PROX_FRONT enable status
            //ret = sensor_read_file(TMD3702_SYS_prox_thres, rbuf,sizeof(rbuf));
            ret=sensor_write_file(TMD3702_SYS_prox_thres, init_buf, sizeof(init_buf));
            SENSOR_LOG_IF(data->debug_level,"PROX_FRONT read enable is %d\n",val);
            break;

        case PROX_BACK:
            // read  PROX_BACK enable status
           // ret = sensor_read_file(PA224_SYS_prox_thres, rbuf,sizeof(rbuf));
            ret=sensor_write_file(PA224_SYS_prox_thres, init_buf, sizeof(init_buf));
            SENSOR_LOG_IF(data->debug_level,"PROX_BACK read enable is %d\n",val);
            break;
        default:
            return -EINVAL;

    }
    if (ret < 0) {
		SENSOR_LOG_ERROR("read file fail\n");
		return ret;
	}
    return size;
}

static ssize_t virtual_prox_raw_debug_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
    int ret;
    int thre_max;
    char rbuf[BUF_MAX_SIZE];
	struct virtual_proximity_data *data = dev_get_drvdata(dev);

    switch(data->fb_status)
    {
        case PROX_FRONT:
            // read  PROX_FRONT enable status
            ret = sensor_read_file(TMD3702_SYS_prox_debug, rbuf,sizeof(rbuf));
            sscanf(rbuf, "%d", &thre_max);
            SENSOR_LOG_IF(data->debug_level,"PROX_FRONT read enable is %d\n",thre_max);
            break;

        case PROX_BACK:
            // read  PROX_BACK enable status
            ret = sensor_read_file(PA224_SYS_prox_debug, rbuf,sizeof(rbuf));
            sscanf(rbuf, "%d", &thre_max);
            SENSOR_LOG_IF(data->debug_level,"PROX_BACK read enable is %d\n",thre_max);
            break;
        default:
            return -EINVAL;

    }
    if (ret < 0) {
		SENSOR_LOG_ERROR("read file fail\n");
		return ret;
	}
    return sprintf(buf, "%d\n", thre_max);

}
static ssize_t virtual_prox_raw_debug_store(struct device *dev,
                struct device_attribute *attr,
                const char *buf, size_t size)
{
    int ret;
    int val = 0;
    char init_buf[BUF_MAX_SIZE] = {'1', '\0'};
    struct virtual_proximity_data *data = dev_get_drvdata(dev);

    ret = kstrtoint(buf, 0, &val);
    sprintf(init_buf, "%d\n", val);

    switch(data->fb_status)
    {
        case PROX_FRONT:
            // read  PROX_FRONT enable status
            //ret = sensor_read_file(TMD3702_SYS_prox_thres, rbuf,sizeof(rbuf));
            ret=sensor_write_file(TMD3702_SYS_prox_debug, init_buf, sizeof(init_buf));
            SENSOR_LOG_IF(data->debug_level,"PROX_FRONT read enable is %d\n",val);
            break;

        case PROX_BACK:
            // read  PROX_BACK enable status
           // ret = sensor_read_file(PA224_SYS_prox_thres, rbuf,sizeof(rbuf));
            ret=sensor_write_file(PA224_SYS_prox_debug, init_buf, sizeof(init_buf));
            SENSOR_LOG_IF(data->debug_level,"PROX_BACK read enable is %d\n",val);
            break;
        default:
            return -EINVAL;

    }
    if (ret < 0) {
		SENSOR_LOG_ERROR("read file fail\n");
		return ret;
	}
    return size;
}
static ssize_t virtual_prox_uncover_data_min_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
    int ret;
    int thre_max;
    char rbuf[BUF_MAX_SIZE];
	struct virtual_proximity_data *data = dev_get_drvdata(dev);

    switch(data->fb_status)
    {
        case PROX_FRONT:
            // read  PROX_FRONT enable status
            ret = sensor_read_file(TMD3702_SYS_prox_uncover_min, rbuf,sizeof(rbuf));
            sscanf(rbuf, "%d", &thre_max);
            SENSOR_LOG_IF(data->debug_level,"PROX_FRONT read enable is %d\n",thre_max);
            break;

        case PROX_BACK:
            // read  PROX_BACK enable status
            ret = sensor_read_file(PA224_SYS_prox_uncover_min, rbuf,sizeof(rbuf));
            sscanf(rbuf, "%d", &thre_max);
            SENSOR_LOG_IF(data->debug_level,"PROX_BACK read enable is %d\n",thre_max);
            break;
        default:
            return -EINVAL;

    }
    if (ret < 0) {
		SENSOR_LOG_ERROR("read file fail\n");
		return ret;
	}
    return sprintf(buf, "%d\n", thre_max);

}
static ssize_t virtual_prox_uncover_data_max_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
    int ret;
    int thre_max;
    char rbuf[BUF_MAX_SIZE];
	struct virtual_proximity_data *data = dev_get_drvdata(dev);

    switch(data->fb_status)
    {
        case PROX_FRONT:
            // read  PROX_FRONT enable status
            ret = sensor_read_file(TMD3702_SYS_prox_uncover_max, rbuf,sizeof(rbuf));
            sscanf(rbuf, "%d", &thre_max);
            SENSOR_LOG_IF(data->debug_level,"PROX_FRONT read enable is %d\n",thre_max);
            break;

        case PROX_BACK:
            // read  PROX_BACK enable status
            ret = sensor_read_file(PA224_SYS_prox_uncover_max, rbuf,sizeof(rbuf));
            sscanf(rbuf, "%d", &thre_max);
            SENSOR_LOG_IF(data->debug_level,"PROX_BACK read enable is %d\n",thre_max);
            break;
        default:
            return -EINVAL;

    }
    if (ret < 0) {
		SENSOR_LOG_ERROR("read file fail\n");
		return ret;
	}
    return sprintf(buf, "%d\n", thre_max);

}


static ssize_t virtual_offset_first_event_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
    int ret;
    int thre_max;
    char rbuf[BUF_MAX_SIZE];
	struct virtual_proximity_data *data = dev_get_drvdata(dev);

    switch(data->fb_status)
    {
        case PROX_FRONT:
            // read  PROX_FRONT enable status
            ret = sensor_read_file(TMD3702_SYS_prox_first_event, rbuf,sizeof(rbuf));
            sscanf(rbuf, "%d", &thre_max);
            SENSOR_LOG_IF(data->debug_level,"PROX_FRONT read enable is %d\n",thre_max);
            break;

        case PROX_BACK:
            // read  PROX_BACK enable status
            ret = sensor_read_file(PA224_SYS_prox_first_event, rbuf,sizeof(rbuf));
            sscanf(rbuf, "%d", &thre_max);
            SENSOR_LOG_IF(data->debug_level,"PROX_BACK read enable is %d\n",thre_max);
            break;
        default:
            return -EINVAL;

    }
    if (ret < 0) {
		SENSOR_LOG_ERROR("read file fail\n");
		return ret;
	}
    return sprintf(buf, "%d\n", thre_max);

}
static ssize_t virtual_offset_first_event_store(struct device *dev,
                struct device_attribute *attr,
                const char *buf, size_t size)
{
    int ret;
    int val = 0;
    char init_buf[BUF_MAX_SIZE] = {'1', '\0'};
    struct virtual_proximity_data *data = dev_get_drvdata(dev);

    ret = kstrtoint(buf, 0, &val);
    sprintf(init_buf, "%d\n", val);
    switch(data->fb_status)
    {
        case PROX_FRONT:
            // read  PROX_FRONT enable status
            //ret = sensor_read_file(TMD3702_SYS_prox_thres, rbuf,sizeof(rbuf));
            ret=sensor_write_file(TMD3702_SYS_prox_first_event, init_buf, sizeof(init_buf));
            SENSOR_LOG_IF(data->debug_level,"PROX_FRONT read enable is %d\n",val);
            break;

        case PROX_BACK:
            // read  PROX_BACK enable status
           // ret = sensor_read_file(PA224_SYS_prox_thres, rbuf,sizeof(rbuf));
            ret=sensor_write_file(PA224_SYS_prox_first_event, init_buf, sizeof(init_buf));
            SENSOR_LOG_IF(data->debug_level,"PROX_BACK read enable is %d\n",val);
            break;
        default:
            return -EINVAL;

    }
    if (ret < 0) {
		SENSOR_LOG_ERROR("read file fail\n");
		return ret;
	}
    return size;
}


static ssize_t virtual_prox_uncover_cal_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
    int ret;
    int thre_max;
    char rbuf[BUF_MAX_SIZE];
	struct virtual_proximity_data *data = dev_get_drvdata(dev);

    switch(data->fb_status)
    {
        case PROX_FRONT:
            // read  PROX_FRONT enable status
            ret = sensor_read_file(TMD3702_SYS_prox_offset_cal, rbuf,sizeof(rbuf));
            sscanf(rbuf, "%d", &thre_max);
            SENSOR_LOG_IF(data->debug_level,"PROX_FRONT read enable is %d\n",thre_max);
            break;

        case PROX_BACK:
            // read  PROX_BACK enable status
            ret = sensor_read_file(PA224_SYS_prox_offset_cal, rbuf,sizeof(rbuf));
            sscanf(rbuf, "%d", &thre_max);
            SENSOR_LOG_IF(data->debug_level,"PROX_BACK read enable is %d\n",thre_max);
            break;
        default:
            return -EINVAL;

    }
    if (ret < 0) {
		SENSOR_LOG_ERROR("read file fail\n");
		return ret;
	}
    return sprintf(buf, "%d\n", thre_max);

}


static ssize_t virtual_prox_uncover_cal_store(struct device *dev,
                struct device_attribute *attr,
                const char *buf, size_t size)
{
    int ret;
    int val = 0;
    char init_buf[BUF_MAX_SIZE] = {'1', '\0'};
    struct virtual_proximity_data *data = dev_get_drvdata(dev);

    ret = kstrtoint(buf, 0, &val);
    sprintf(init_buf, "%d\n", val);
    switch(data->fb_status)
    {
        case PROX_FRONT:
            // read  PROX_FRONT enable status
            //ret = sensor_read_file(TMD3702_SYS_prox_thres, rbuf,sizeof(rbuf));
            ret=sensor_write_file(TMD3702_SYS_prox_offset_cal, init_buf, sizeof(init_buf));
            SENSOR_LOG_IF(data->debug_level,"PROX_FRONT read enable is %d\n",val);
            break;

        case PROX_BACK:
            // read  PROX_BACK enable status
           // ret = sensor_read_file(PA224_SYS_prox_thres, rbuf,sizeof(rbuf));
            ret=sensor_write_file(PA224_SYS_prox_offset_cal, init_buf, sizeof(init_buf));
            SENSOR_LOG_IF(data->debug_level,"PROX_BACK read enable is %d\n",val);
            break;
        default:
            return -EINVAL;

    }
    if (ret < 0) {
		SENSOR_LOG_ERROR("read file fail\n");
		return ret;
	}
    return size;
}

static ssize_t virtual_prox_debug_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
    int ret;
    int thre_max;
    char rbuf[BUF_MAX_SIZE];
	struct virtual_proximity_data *data = dev_get_drvdata(dev);

    switch(data->fb_status)
    {
        case PROX_FRONT:
            // read  PROX_FRONT enable status
            ret = sensor_read_file(TMD3702_SYS_debug, rbuf,sizeof(rbuf));
            sscanf(rbuf, "%d", &thre_max);
            SENSOR_LOG_IF(data->debug_level,"PROX_FRONT read enable is %d\n",thre_max);
            break;

        case PROX_BACK:
            // read  PROX_BACK enable status
            ret = sensor_read_file(PA224_SYS_debug, rbuf,sizeof(rbuf));
            sscanf(rbuf, "%d", &thre_max);
            SENSOR_LOG_IF(data->debug_level,"PROX_BACK read enable is %d\n",thre_max);
            break;
        default:
            return -EINVAL;

    }
    if (ret < 0) {
		SENSOR_LOG_ERROR("read file fail\n");
		return ret;
	}
    return sprintf(buf, "%d\n", thre_max);

}


static ssize_t virtual_prox_debug_store(struct device *dev,
                struct device_attribute *attr,
                const char *buf, size_t size)
{
    int ret;
    int val = 0;
    char init_buf[BUF_MAX_SIZE] = {'1', '\0'};
    struct virtual_proximity_data *data = dev_get_drvdata(dev);

    ret = kstrtoint(buf, 0, &val);
    sprintf(init_buf, "%d\n", val);

/* Promblem Number: PR000     Author:xuxiaohua,   Date:2018/9/3
   Description    : add virtual_proximity_data  debug_level */
    data->debug_level =val;
    //return size;
    sensor_set_debug_level(val); //call ams_tmd3702.c
/* END:   Added by xuxiaohua, 2018/9/3 */

    switch(data->fb_status)
    {
        case PROX_FRONT:
            // read  PROX_FRONT enable status
            //ret = sensor_read_file(TMD3702_SYS_prox_thres, rbuf,sizeof(rbuf));
            ret=sensor_write_file(TMD3702_SYS_debug, init_buf, sizeof(init_buf));
            SENSOR_LOG_IF(data->debug_level,"PROX_FRONT read enable is %d\n",val);
            break;

        case PROX_BACK:
            // read  PROX_BACK enable status
           // ret = sensor_read_file(PA224_SYS_prox_thres, rbuf,sizeof(rbuf));
            ret=sensor_write_file(PA224_SYS_prox_debug, init_buf, sizeof(init_buf));
            SENSOR_LOG_IF(data->debug_level,"PROX_BACK read enable is %d\n",val);
            break;
        default:
            return -EINVAL;

    }
    if (ret < 0) {
		SENSOR_LOG_ERROR("read file fail\n");
		return ret;
	}
    return size;
}


static ssize_t virtual_prox_offset_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
    int ret;
    int thre_max;
    char rbuf[BUF_MAX_SIZE];
	struct virtual_proximity_data *data = dev_get_drvdata(dev);

    switch(data->fb_status)
    {
        case PROX_FRONT:
            // read  PROX_FRONT enable status
            ret = sensor_read_file(TMD3702_SYS_prox_offset, rbuf,sizeof(rbuf));
            sscanf(rbuf, "%d", &thre_max);
            SENSOR_LOG_IF(data->debug_level,"PROX_FRONT read enable is %d\n",thre_max);
            break;

        case PROX_BACK:
            // read  PROX_BACK enable status
            ret = sensor_read_file(PA224_SYS_prox_offset, rbuf,sizeof(rbuf));
            sscanf(rbuf, "%d", &thre_max);
            SENSOR_LOG_IF(data->debug_level,"PROX_BACK read enable is %d\n",thre_max);
            break;
        default:
            return -EINVAL;

    }
    if (ret < 0) {
		SENSOR_LOG_ERROR("read file fail\n");
		return ret;
	}
    return sprintf(buf, "%d\n", thre_max);

}


static ssize_t virtual_prox_dev_init_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return 1;
}

static ssize_t virtual_prox_dev_init_store(struct device *dev,
                struct device_attribute *attr,
                const char *buf, size_t size)
{
    int ret;
    int val = 0;
    char init_buf[2] = {'1', '\0'};
    struct virtual_proximity_data *data = dev_get_drvdata(dev);

    ret = kstrtoint(buf, 0, &val);
    if (data->fb_status == PROX_BACK)
        return size;
    if (val != 0)
        sensor_write_file(TMD3702_SYS_INIT_PATH, init_buf, sizeof(init_buf));
    return size;
}

static ssize_t virtual_prox_value_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
    int ret;
    u8 show_buf[8];
	struct virtual_proximity_data *data = dev_get_drvdata(dev);

    switch(data->fb_status)
    {
        case PROX_FRONT:
            ret = sensor_read_file(TMD3702_SYS_FLUSH_PATH, show_buf, sizeof(show_buf));
            break;

        case PROX_BACK:
            ret = sensor_read_file(PA224_SYS_FLUSH_PATH, show_buf, sizeof(show_buf));;
            break;
        default:
            return -EINVAL;
    }

    if (ret < 0) {
		SENSOR_LOG_ERROR("read file fail\n");
		return ret;
	}
    return sprintf(buf, "%s\n", "virtual prox flush ok");

}

static ssize_t virtual_prox_chip_id_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", VIR_PROX_CHIP_NAME);
}

static struct device_attribute attrs_prox_device[] = {
	__ATTR(enable, 0644, virtual_proximity_enable_show, virtual_proximity_enable_store),
    __ATTR(proximity_fb,0640,proximity_fb_show,proximity_fb_store),
    __ATTR(prox_init, 0644, virtual_prox_dev_init_show, virtual_prox_dev_init_store),
    __ATTR(prox_value, 0440, virtual_prox_value_show, NULL),
    __ATTR(chip_name, 0440, virtual_prox_chip_id_show, NULL),
    __ATTR(prox_thres_min, 0440, virtual_prox_min_thres_show, NULL),
	__ATTR(prox_thres_max,0440, virtual_prox_max_thres_show, NULL),
	__ATTR(prox_thres,0644, virtual_prox_thres_show, virtual_prox_thres_store),
 	__ATTR(prox_debug, 0644, virtual_prox_raw_debug_show,  virtual_prox_raw_debug_store),
	__ATTR(prox_uncover_min, 0440, virtual_prox_uncover_data_min_show, NULL),
    __ATTR(prox_uncover_max, 0440, virtual_prox_uncover_data_max_show, NULL),
    __ATTR(prox_first_event, 0644, virtual_offset_first_event_show,  virtual_offset_first_event_store),
    __ATTR(prox_offset_cal, 0644, virtual_prox_uncover_cal_show, virtual_prox_uncover_cal_store),
    __ATTR(debug, 0644, virtual_prox_debug_show,  virtual_prox_debug_store),
    __ATTR(prox_offset, 0440, virtual_prox_offset_show,NULL),
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

int prox_fb_notifier_callback(struct notifier_block *self,
                         unsigned long event, void *data)
{
	struct virtual_proximity_data *prox_data =
		container_of(self, struct virtual_proximity_data, ps_fb_notify);
	struct fb_event *fb_event = data;
    int *blank;
	char front_buf[2] = {'1', '\0'};
	char back_buf[2] = {'2', '\0'};
	SENSOR_LOG_IF(prox_data->debug_level, "prox_fb_notifier_callback enter.\n");
    if (fb_event && fb_event->data && prox_data)
    {
        if(event == MSM_DRM_SWITCH_EVENT_BLANK){
            blank = fb_event->data;
            switch (*blank)
            {
                case MSM_DRM_MAJOR_BLANK_UNBLANK:
					/*
					*don`t modify to set_proximity_fb() set proximity fornt or back
					*because android.hardware.sensors@1.0-impl.so need notify proximity_fb node
					*/
					SENSOR_LOG_INFO("set proximity is front(value is 1)");
					sensor_write_file(SET_PROXIMITY_FB_PATH, front_buf, sizeof(front_buf));
                    break;

                case MSM_DRM_SLAVE_BLANK_UNBLANK:
					/*
					*don`t modify to set_proximity_fb() set proximity fornt or back
					*because android.hardware.sensors@1.0-impl.so need notify proximity_fb node
					*/
					SENSOR_LOG_INFO("set proximity is back(value is 2)");
					sensor_write_file(SET_PROXIMITY_FB_PATH, back_buf, sizeof(back_buf));
                    break;
				default:
					SENSOR_LOG_IF(prox_data->debug_level,"No have event to process\n");
            }
        }
    }
	SENSOR_LOG_IF(prox_data->debug_level,"prox_fb_notifier_callback exit\n");
    return NOTIFY_DONE;
}

static void prox_fb_notifier_register(void){
    int ret = 0;
	vp_data->ps_fb_notify.notifier_call = prox_fb_notifier_callback;
    ret = msm_drm_switch_register_client(&vp_data->ps_fb_notify);
    if (ret < 0){
        SENSOR_LOG_ERROR("Failed to register proximity notify cilent\n");
    }
}

static int virtual_proximity_register(void)
{
    int err = 0;

	SENSOR_LOG_INFO("virtual_proximity_register_sys start\n");

    vp_data = kzalloc(sizeof(struct virtual_proximity_data), GFP_KERNEL);
	if (!vp_data) {
		SENSOR_LOG_ERROR("kzalloc vp_data failed\n");
		err = -ENOMEM;
		goto exit;
	}
	mutex_init(&vp_data->rw_lock);
	vp_data->flag_prox_debug = false;
	vp_data->fb_status = PROX_FRONT;
    vp_data->report_fb_status = 0;
    vp_data->report_value = 0;
    vp_data->enable_status = false;

    virtual_proximity_class = class_create(THIS_MODULE, VIR_PROXNAME_NAME);
    alloc_chrdev_region(&virtual_proximity_dev_t, 0, 1, VIR_PROX_DRV_NAME);
    vp_data->proximity_dev = device_create(virtual_proximity_class, 0,
        virtual_proximity_dev_t, 0, VIR_PROX_DRV_NAME);
	if (IS_ERR(vp_data->proximity_dev)) {
		SENSOR_LOG_ERROR("device_create proximity failed\n");
		goto create_proximity_dev_failed;
	}

	dev_set_drvdata(vp_data->proximity_dev, vp_data);

	err = create_sysfs_interfaces(vp_data->proximity_dev);
	if (err < 0) {
		SENSOR_LOG_ERROR("create sysfs interfaces failed\n");
		goto create_sysfs_interface_failed;
	}

	/* allocate proximity input_device */

    virtual_ps_input_dev = input_allocate_device();
    if (IS_ERR_OR_NULL(virtual_ps_input_dev)) {
		err = -ENOMEM;
		SENSOR_LOG_ERROR("could not allocate virtual input device\n");
		goto allocate_input_device_failed;
	}
	input_set_drvdata(virtual_ps_input_dev, vp_data);
	virtual_ps_input_dev->name = INPUT_NAME_VIR_PS;
	virtual_ps_input_dev->id.bustype = BUS_I2C;
	set_bit(EV_REL, virtual_ps_input_dev->evbit);
    set_bit(REL_RZ,  virtual_ps_input_dev->relbit);
	set_bit(REL_RX,  virtual_ps_input_dev->relbit);
	set_bit(REL_MISC, virtual_ps_input_dev->relbit);
    set_bit(REL_DIAL, virtual_ps_input_dev->relbit);
	err = input_register_device(virtual_ps_input_dev);
	if (err < 0) {
		SENSOR_LOG_ERROR("could not register virtual_ps input device\n");
		err = -ENOMEM;
		goto register_input_device_failed;
	}

    vp_data->ps_input_dev = virtual_ps_input_dev;

#ifdef CONFIG_NUBIA_SWITCH_LCD
	prox_fb_notifier_register();
#endif

	SENSOR_LOG_INFO("virtual_proximity_register ok.\n");

	return 0;
register_input_device_failed:
	input_unregister_device(virtual_ps_input_dev);
allocate_input_device_failed:
	input_free_device(virtual_ps_input_dev);
create_sysfs_interface_failed:
	remove_sysfs_interfaces(vp_data->proximity_dev);
create_proximity_dev_failed:
	vp_data->proximity_dev = NULL;
	device_destroy(virtual_proximity_class, virtual_proximity_dev_t);
	class_destroy(virtual_proximity_class);
exit:
	return err;
}

static int __init virtual_proximity_init(void)
{
	SENSOR_LOG_INFO("virtual_proximity_init init 2018-09-07\n");
    virtual_proximity_register();

    tmd3702_init();
    pa224_init();

    return 0;
}

static void __exit virtual_proximity_exit(void)
{
    struct device *dev = vp_data->proximity_dev;

	input_unregister_device(vp_data->ps_input_dev);
	input_free_device(vp_data->ps_input_dev);
	remove_sysfs_interfaces(dev);
	mutex_destroy(&vp_data->rw_lock);
	kfree(vp_data);

    pa224_exit();
    tmd3702_exit();
}

MODULE_AUTHOR("Peripherial team, NUBIA");
MODULE_DESCRIPTION("Set proxomity front or back sensor driver.");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(DRIVER_VERSION);

module_init(virtual_proximity_init);
module_exit(virtual_proximity_exit);

