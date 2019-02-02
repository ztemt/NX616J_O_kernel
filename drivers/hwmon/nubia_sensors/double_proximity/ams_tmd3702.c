/*
*****************************************************************************
* Copyright by ams AG                                                       *
* All rights are reserved.                                                  *
*                                                                           *
* IMPORTANT - PLEASE READ CAREFULLY BEFORE COPYING, INSTALLING OR USING     *
* THE SOFTWARE.                                                             *
*                                                                           *
* THIS SOFTWARE IS PROVIDED FOR USE ONLY IN CONJUNCTION WITH AMS PRODUCTS.  *
* USE OF THE SOFTWARE IN CONJUNCTION WITH NON-AMS-PRODUCTS IS EXPLICITLY    *
* EXCLUDED.                                                                 *
*                                                                           *
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       *
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT         *
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS         *
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT  *
* OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,     *
* SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT          *
* LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,     *
* DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY     *
* THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT       *
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE     *
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.      *
*****************************************************************************
*/

/*! \file
* \brief Device driver for monitoring ambient light intensity in (lux)
* proximity detection (prox) functionality within the
* AMS-TAOS TMD3702 family of devices.
*/

#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/irq.h>
#include <linux/mutex.h>
#include <linux/unistd.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/pm.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/kthread.h>
#include <linux/freezer.h>
#include <linux/gpio/consumer.h>
#ifdef CONFIG_OF
#include <linux/of_device.h>
#endif

//#include <linux/i2c/ams/ams_tmd3702.h>
#include "ams_tmd3702.h"
#include "ams_tmd3702_i2c.h"
#include "ams_tmd3702_prox.h"
#include "ams_tmd3702_als.h"
#include "virtual_proximity.h"

/* TMD3702 Identifiers */
static u8 const tmd3702_ids[] = {
//    ID,    REVID,    REVID2
    0x10,    0x01,    0x05
};

/* TMD3702 Device Names */
static char const *tmd3702_names[] = {
    "tmd3702"
};

/* Registers to restore */
static u8 const restorable_regs[] = {
    TMD3702_REG_PILTL,
    TMD3702_REG_PILTH,
    TMD3702_REG_PIHTL,
    TMD3702_REG_PIHTH,
    TMD3702_REG_PERS,
    TMD3702_REG_PGCFG0,
    TMD3702_REG_PGCFG1,
    TMD3702_REG_CFG0,
    TMD3702_REG_CFG1,
    TMD3702_REG_CFG4,
    TMD3702_REG_CFG6,
    /* TODO remove me after new datasheet rev */
    TMD3702_REG_TEST3,
    TMD3702_REG_PRATE,
    TMD3702_REG_ATIME,
};
#define INPUT_ALS_NAME "light"
#define INPUT_PS_NAME "proximity_front"
#define TMD3702_DRIVER_VERSION		"1.0.0"

//static struct i2c_driver tmd3702_driver;
static struct i2c_driver tmd3702_driver;
static void tmd3702_set_defaults(struct tmd3702_chip *chip);
static int tmd3702_flush_regs(struct tmd3702_chip *chip);
static int tmp3702_debug_level=0;
/*****************************************************************/
/* start of ams_common.h file function list          */
//1.int sensor_read_file(char *file_path, char *read_buf, int count)
//2.int sensor_write_file(char *file_path, const char *write_buf, int count)
//3.int sensor_create_sysfs_interfaces(struct device *dev,
//4.void sensor_remove_sysfs_interfaces(struct device *dev,
//5.int sensor_regulator_configure(struct tmd3702_chip *chip, bool on)
//6.int sensor_regulator_power_on(struct tmd3702_chip *chip, bool on)
//7.int sensor_hw_pinctrl_init(struct tmd3702_chip *chip, struct device *dev)
//8.static int tmd3702_platform_hw_power_on(struct tmd3702_chip *chip, bool on)
//9.static int tmd3702_common_hw_init(struct tmd3702_chip *chip)
//10.void sensor_irq_enable(struct tmd3702_chip *data, bool enable, bool flag_sync)
//11.void sensor_quick_sort(int *pdata, int len, int left, int right)
//12.static int tmd3702_add_sysfs_interfaces(struct device *dev,
//13.static void tmd3702_remove_sysfs_interfaces(struct device *dev,
//14.static int tmd3702_parse_dt(struct device *dev,
//15.int tmd3702_device_warm_up(struct tmd3702_chip *chip)
/****************************************************************/

/* Promblem Number: PR000     Author:xuxiaohua,   Date:2018/9/3
   Description    : add virtual_proximity_data  debug_level */
int sensor_set_debug_level(u8 data)
{
    tmp3702_debug_level=data;
    SENSOR_LOG_INFO("run to tmp3702_debug_level:%d ",tmp3702_debug_level);
    return 1;
}
/* END:   Added by xuxiaohua, 2018/9/3 */

int sensor_read_file(char *file_path, char *read_buf, int count)
{
	struct file *file_p;
	mm_segment_t old_fs;
	int vfs_retval = -EINVAL;
	bool file_exist = true;
	char *buf = NULL;

	SENSOR_LOG_IF(tmp3702_debug_level,"read infomation :file_path:%s, size =%d\n",
       file_path,count);
  //  SENSOR_LOG_INFO("run to tmp3702_debug_level:%d \n",tmp3702_debug_level);

	if (NULL == file_path) {
		SENSOR_LOG_ERROR("file_path is NULL\n");
		return -EINVAL;
	}

	file_p = filp_open(file_path, O_RDONLY , 0444);
	if (IS_ERR(file_p)) {
		file_exist = false;
		SENSOR_LOG_INFO("file does not exist\n");
		buf = kzalloc(count * sizeof(char), GFP_KERNEL);
		if (IS_ERR_OR_NULL(buf)) {
			SENSOR_LOG_ERROR("alloc mem failed\n");
			goto error;
		}
	} else {
		filp_close(file_p, NULL);
	}

	file_p = filp_open(file_path, O_RDWR , 0666);
	if (IS_ERR(file_p)) {
		SENSOR_LOG_ERROR("[open file <%s>failed]\n",file_path);
		goto error;
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	if (!file_exist) {
		SENSOR_LOG_DEBUG("init file memory\n");
		if (!IS_ERR_OR_NULL(buf)) {
			vfs_retval = vfs_write(file_p, (char *)buf, sizeof(buf), &file_p->f_pos);
			if (vfs_retval < 0) {
				SENSOR_LOG_ERROR("[write file <%s>failed]\n",file_path);
				goto file_close;
			}
		}

	}

	file_p->f_pos = 0;
	vfs_retval = vfs_read(file_p, (char*)read_buf, count, &file_p->f_pos);
	if (vfs_retval < 0) {
		SENSOR_LOG_ERROR("[write file <%s>failed]\n",file_path);
		goto file_close;
	}

	SENSOR_LOG_IF(tmp3702_debug_level,"read ok\n");

file_close:
	set_fs(old_fs);
	filp_close(file_p, NULL);
error:
	if (!IS_ERR_OR_NULL(buf))
		kfree(buf);
	return vfs_retval;
}

int sensor_write_file(char *file_path, const char *write_buf, int count)
{
	struct file *file_p;
	mm_segment_t old_fs;
	int vfs_retval = -EINVAL;

   // SENSOR_LOG_INFO("run to tmp3702_debug_level:%d \n",tmp3702_debug_level);
    SENSOR_LOG_IF(tmp3702_debug_level,"write infomation : file_path:%s size =%d\n",
        file_path,count);

	if (NULL == file_path) {
		SENSOR_LOG_ERROR("file_path is NULL\n");
		return -EINVAL;
	}

	file_p = filp_open(file_path, O_CREAT|O_RDWR|O_TRUNC , 0666);
	if (IS_ERR(file_p)) {
		SENSOR_LOG_ERROR("[open file <%s>failed]\n",file_path);
		goto error;
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	vfs_retval = vfs_write(file_p, (char*)write_buf, count, &file_p->f_pos);
	if (vfs_retval < 0) {
		SENSOR_LOG_ERROR("[write file <%s>failed]\n",file_path);
		goto file_close;
	}

	SENSOR_LOG_IF(tmp3702_debug_level,"write ok\n");

file_close:
	set_fs(old_fs);
	filp_close(file_p, NULL);
error:
	return vfs_retval;
}

int sensor_create_sysfs_interfaces(struct device *dev,
	struct device_attribute *attr, int size)
{
	int i;
	for (i = 0; i < size; i++)
		if (device_create_file(dev, attr + i))
			goto exit;
	return 0;
exit:
	for (; i >= 0 ; i--)
		device_remove_file(dev, attr + i);
	SENSOR_LOG_ERROR("failed to create sysfs interface\n");
	return -ENODEV;
}

void sensor_remove_sysfs_interfaces(struct device *dev,
	struct device_attribute *attr, int size)
{
	int i;
	for (i = 0; i < size; i++)
		device_remove_file(dev, attr + i);
}
int sensor_regulator_configure(struct tmd3702_chip *chip, bool on)
{
	int rc;
	if (unlikely(IS_ERR_OR_NULL(chip->pdata))) {
		SENSOR_LOG_ERROR("null pointer.\n");
		rc = -PTR_ERR(chip->pdata);
		return rc;
	}

	if (!on) {
		if(!IS_ERR_OR_NULL(chip->pdata->vdd)) {
			if (regulator_count_voltages(chip->pdata->vdd) > 0)
				regulator_set_voltage(chip->pdata->vdd, 0, POWER_VDD_MAX_UV);
			regulator_put(chip->pdata->vdd);
			regulator_disable(chip->pdata->vdd);
		}
		if(!IS_ERR_OR_NULL(chip->pdata->vio)) {
			if (regulator_count_voltages(chip->pdata->vio) > 0)
				regulator_set_voltage(chip->pdata->vio, 0, POWER_VIO_MAX_UV);
			regulator_put(chip->pdata->vio);
			regulator_disable(chip->pdata->vio);
		}
	} else {
		chip->pdata->vdd = regulator_get(&chip->client->dev, "vdd");
		if (IS_ERR_OR_NULL(chip->pdata->vdd)) {
			rc = -PTR_ERR(chip->pdata->vdd);
			SENSOR_LOG_ERROR("Regulator get failed vdd rc=%d\n", rc);
			return rc;
		}
		if (!IS_ERR_OR_NULL(chip->pdata->vdd)) {
			if (regulator_count_voltages(chip->pdata->vdd) > 0) {
				rc = regulator_set_voltage(chip->pdata->vdd,
					POWER_VDD_MIN_UV, POWER_VDD_MAX_UV);
				if (rc) {
					SENSOR_LOG_ERROR("Regulator set failed vdd rc=%d\n",	rc);
					rc = - EINVAL;
					goto reg_vdd_put;
				}
			}

			rc = regulator_enable(chip->pdata->vdd);
			if (rc) {
				SENSOR_LOG_ERROR("Regulator enable vdd failed. rc=%d\n", rc);
				rc = - EINVAL;
				goto reg_vdd_put;
			}
			SENSOR_LOG_INFO("vdd regulator ok\n");
		}

		chip->pdata->vio = regulator_get(&chip->client->dev, "vio");
		if (IS_ERR_OR_NULL(chip->pdata->vio)) {
			rc = -PTR_ERR(chip->pdata->vio);
			SENSOR_LOG_ERROR("Regulator get failed vdd rc=%d\n", rc);
			return rc;
		}
		if (!IS_ERR_OR_NULL(chip->pdata->vio)) {
			if (regulator_count_voltages(chip->pdata->vio) > 0) {
				rc = regulator_set_voltage(chip->pdata->vio,
					POWER_VIO_MIN_UV, POWER_VIO_MAX_UV);
				if (rc) {
					SENSOR_LOG_ERROR("Regulator set failed vdd rc=%d\n",	rc);
					rc = - EINVAL;
					goto reg_vio_put;
				}
			}

			rc = regulator_enable(chip->pdata->vio);
			if (rc) {
				SENSOR_LOG_ERROR("Regulator enable vdd failed. rc=%d\n", rc);
				rc = - EINVAL;
				goto reg_vio_put;
			}
			SENSOR_LOG_INFO("vio regulator ok\n");
		}
	}
	return 0;
reg_vio_put:
	regulator_put(chip->pdata->vio);
reg_vdd_put:
	regulator_put(chip->pdata->vdd);
	return rc;
}


int sensor_regulator_power_on(struct tmd3702_chip *chip, bool on)
{
	int rc = 0;

	if (IS_ERR_OR_NULL(chip->pdata))
		return -EINVAL;

	if (!on) {
		if (!IS_ERR_OR_NULL(chip->pdata->vdd)) {
			rc = regulator_disable(chip->pdata->vdd);
			if (rc) {
				SENSOR_LOG_ERROR(	"Regulator vdd disable failed rc=%d\n", rc);
				return rc;
			}
		}
		if (!IS_ERR_OR_NULL(chip->pdata->vio)) {
			rc = regulator_disable(chip->pdata->vio);
			if (rc) {
				SENSOR_LOG_ERROR(	"Regulator vdd disable failed rc=%d\n", rc);
				return rc;
			}
		}
	} else {
		if (!IS_ERR_OR_NULL(chip->pdata->vdd)) {
			rc = regulator_enable(chip->pdata->vdd);
			if (rc) {
				SENSOR_LOG_ERROR(	"Regulator vdd enable failed rc=%d\n", rc);
				return rc;
			}
		}
		if (!IS_ERR_OR_NULL(chip->pdata->vio)) {
			rc = regulator_enable(chip->pdata->vio);
			if (rc) {
				SENSOR_LOG_ERROR(	"Regulator vdd enable failed rc=%d\n", rc);
				return rc;
			}
		}
	}

	SENSOR_LOG_INFO("power state : (%s)\n", on ? "on":"off");
	mdelay(5);
	return rc;
}

int sensor_hw_pinctrl_init(struct tmd3702_chip *chip, struct device *dev)
{
	int rc;
	if (unlikely(IS_ERR_OR_NULL(chip) || IS_ERR_OR_NULL(dev)))
		return -EINVAL;

	chip->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR_OR_NULL(chip->pinctrl)) {
		SENSOR_LOG_ERROR("data->pinctrl is NULL\n");
		return PTR_ERR(chip->pinctrl);
	}

	chip->pin_default = pinctrl_lookup_state(chip->pinctrl, "tmd3702_default");
	if (IS_ERR_OR_NULL(chip->pin_default)) {
		SENSOR_LOG_ERROR("lookup default state failed\n");
		return PTR_ERR(chip->pin_default);
	}

	chip->pin_sleep = pinctrl_lookup_state(chip->pinctrl, "tmd3702_sleep");
	if (IS_ERR_OR_NULL(chip->pin_sleep)) {
		SENSOR_LOG_ERROR("lookup sleep state failed\n");
		return PTR_ERR(chip->pin_sleep);
	}

	if (!IS_ERR_OR_NULL(chip->pinctrl)) {
		rc = pinctrl_select_state(chip->pinctrl, chip->pin_default);
		if (rc) {
			SENSOR_LOG_ERROR("select default state failed\n");
			return rc;
		}
	}
	SENSOR_LOG_INFO("pinctrl init success\n");
	return 0;
}
/*******************************************************************************************
* platform init  *
// 1.   int (*platform_power)(struct device *dev, enum tmd3702_pwr_state state);
   //   int (*platform_power)(struct device *dev, enum tmd3702_pwr_state state);
// 2.   int (*platform_init)(void);
// 3.   void (*platform_teardown)(struct device *dev);
*******************************************************************************************/
static int tmd3702_platform_hw_power_on(struct tmd3702_chip *chip, bool on)
{
	int err=0x0;
	if (unlikely(IS_ERR_OR_NULL(chip->pdata)))
		return -ENODEV;
	if (!chip->pdata->power_always_on) {
		if (chip->pdata->power_state == POWER_OFF) {
			err = sensor_regulator_power_on(chip, on);
		}
	} else {
		err = sensor_regulator_power_on(chip, on);
	}
	return err;
}
//  void (*platform_teardown)(struct device *dev);
static int tmd3702_common_hw_init(struct tmd3702_chip *chip)
{
	int ret;
	tmd3702_set_defaults(chip);

	ret = tmd3702_flush_regs(chip);
	if (ret < 0) {
		SENSOR_LOG_ERROR("flush reg error\n");
		return -ENODEV;
	}
	SENSOR_LOG_INFO("tmd3702 common hw init finished\n");
	return 0;
}
//  void (*platform_teardown)(struct device *dev);
static int tmd3702_platform_hw_exit(struct tmd3702_chip *chip)
{
	return 0;
}

/*******************************************************************************************
* end of add  platform init  *
*******************************************************************************************/
void sensor_irq_enable(struct tmd3702_chip *data, bool enable, bool flag_sync)
{
	SENSOR_LOG_DEBUG_IF(data->pdata->debug_level, "irq %s\n",enable ? "enable" : "disable");
	if (enable == data->irq_enabled) {
		SENSOR_LOG_DEBUG("doubule %s irq %d\n",enable? "enable" : "disable",data->irq);
		return;
	} else {
			data->irq_enabled = enable;
	}
	if (enable) {
		enable_irq(data->irq);
	} else {
		if (flag_sync) {
			disable_irq(data->irq);
		} else {
			disable_irq_nosync(data->irq);
		}
	}
}
void sensor_quick_sort(int *pdata, int len, int left, int right)
{
	int i, j, tmp, t;

	if(left > right || IS_ERR_OR_NULL(pdata) || len == 0)
		return;
	tmp = pdata[left];
	i = left;
	j = right;
	while(i != j){
		while(pdata[j] >= tmp && i < j)
			j--;
		while(pdata[i] <= tmp && i < j)
			i++;

		if (i < j) {
			t = pdata[i];
			pdata[i] = pdata[j];
			pdata[j] = t;
		}
	}
	pdata[left] = pdata[i];
	pdata[i] = tmp;
	sensor_quick_sort(pdata, len, left, i-1);
	sensor_quick_sort(pdata, len, i + 1, right);
}




static int tmd3702_flush_regs(struct tmd3702_chip *chip)
{
    unsigned i;
    int rc;
    u8 reg;

	for (i = 0; i < ARRAY_SIZE(restorable_regs); i++)
	{
		reg = restorable_regs[i];
		rc = ams_i2c_write(chip->client, chip->shadow, reg, chip->shadow[reg]);
		if (rc) {
			//dev_err(&chip->client->dev, "%s: err on reg 0x%02x\n", __func__, reg);
			SENSOR_LOG_ERROR(" err on reg 0x%02x\n",  reg);
			break;
		}
		SENSOR_LOG_IF(chip->pdata->debug_level,"reg:0x%2x,val:0x%2x\n",reg,chip->shadow[reg]);
	}

    return rc;
}

 int tmd3702_read_regs(struct tmd3702_chip *chip)
{

	int rc=0x0;
	unsigned i;
	u8 reg;
	u8 val;
	for (i = 0; i < ARRAY_SIZE(restorable_regs); i++)
	{
		reg = restorable_regs[i];
		ams_i2c_blk_read(chip->client, reg, &val, 1);
		SENSOR_LOG_IF(chip->pdata->debug_level,"reg: 0x%2x,val:0x%2x\n",reg,val);
	}

	return rc;
}



static void tmd3702_set_defaults(struct tmd3702_chip *chip)
{
    u8 *sh = chip->shadow;
 //   struct device *dev = &chip->client->dev;

    // Clear the register shadow area
    memset(chip->shadow, 0x00, sizeof(chip->shadow));

    // If there is platform data use it
    if (chip->pdata) {

        chip->params.prox_th_max = chip->pdata->parameters.prox_th_max;
        chip->params.prox_th_min = chip->pdata->parameters.prox_th_min;
        chip->params.persist = chip->pdata->parameters.persist;
        chip->params.prox_pulse_cnt = chip->pdata->parameters.prox_pulse_cnt;
        chip->params.prox_apc = chip->pdata->parameters.prox_apc;
        chip->params.prox_gain = chip->pdata->parameters.prox_gain;
        chip->params.prox_drive = chip->pdata->parameters.prox_drive;
        chip->params.poffset = chip->pdata->parameters.poffset;
        chip->params.prox_pulse_len = chip->pdata->parameters.prox_pulse_len;
        chip->params.prox_pulse_16x = chip->pdata->parameters.prox_pulse_16x;
        chip->params.als_gain = chip->pdata->parameters.als_gain;
        chip->params.als_deltaP = chip->pdata->parameters.als_deltaP;
        chip->params.als_time = chip->pdata->parameters.als_time;
        chip->params.ct_coef   = chip->pdata->parameters.ct_coef;
        chip->params.ct_offset = chip->pdata->parameters.ct_offset;
        chip->params.c_coef = chip->pdata->parameters.c_coef;
        chip->params.r_coef = chip->pdata->parameters.r_coef;
        chip->params.g_coef = chip->pdata->parameters.g_coef;
        chip->params.b_coef = chip->pdata->parameters.b_coef;
        chip->params.dgf    = chip->pdata->parameters.dgf;
        chip->params.coef_scale = chip->pdata->parameters.coef_scale;

    }
    else {


        chip->params.prox_th_min = 40;					//64
        chip->params.prox_th_max = 60;					//128
        chip->params.persist = PRX_PERSIST(1) | ALS_PERSIST(2);
        chip->params.prox_pulse_cnt = 32;					//4/
        chip->params.prox_apc = 1; /* 0 -> APC enabled */		// 0
        chip->params.prox_gain = PGAIN_1;					//PGAIN_4
        chip->params.prox_drive = PDRIVE_MA(14);			//PDRIVE_MA(75)
        chip->params.poffset = 0;
        chip->params.prox_pulse_len = 2; //2-->PG_PULSE_16US;		//3 -->PG_PULSE_32US
        chip->params.prox_pulse_16x = 0;
        chip->params.als_gain = AGAIN_64;					//AGAIN_16
        chip->params.als_deltaP = 10;
        chip->params.als_time = AW_TIME_MS(50);			//AW_TIME_MS(200)
        chip->params.ct_coef   = CT_COEF_DEFAULT;
        chip->params.ct_offset = CT_OFFSET_DEFAULT;
        chip->params.c_coef = CLR_COEF_DEFAULT;
        chip->params.r_coef = RED_COEF_DEFAULT;
        chip->params.g_coef = GRN_COEF_DEFAULT;
        chip->params.b_coef = BLU_COEF_DEFAULT;
        chip->params.dgf    = DGF_DEFAULT;
        chip->params.coef_scale = COEF_SCALE_DEFAULT;
    }

	chip->params.prox_pulse_cnt = 32;					//4/
	chip->params.prox_gain = PGAIN_1;					//PGAIN_4
	chip->params.prox_drive = PDRIVE_MA(14);			//PDRIVE_MA(75)
	chip->params.prox_pulse_len =  2;  // 2->PG_PULSE_16US;		//3->PG_PULSE_32US
	//ams change code 20180523 true ->flash close als gain auto
	chip->als_gain_auto = true;

    // Copy the default values into the register shadow area
    sh[TMD3702_REG_PILTL]    = chip->params.prox_th_min;
   // sh[TMD3702_REG_PILTH]    = ((chip->params.prox_th_min >> 8) & 0xff);
    sh[TMD3702_REG_PIHTL]    = chip->params.prox_th_max;
   // sh[TMD3702_REG_PIHTH]    = ((chip->params.prox_th_max >> 8) & 0xff);
    sh[TMD3702_REG_PERS]     = chip->params.persist;
    sh[TMD3702_REG_CFG6]    = (chip->params.prox_apc << TMD3702_SHIFT_APC)|0x3f;
    sh[TMD3702_REG_PGCFG0]   = chip->params.prox_pulse_cnt |
                               (chip->params.prox_pulse_len << TMD3702_SHIFT_PPULSE_LEN);

    sh[TMD3702_REG_ATIME]    = chip->params.als_time;
    sh[TMD3702_REG_CFG1]     = chip->params.als_gain;
    sh[TMD3702_REG_CFG0]    = chip->params.prox_pulse_16x|0x40;
    sh[TMD3702_REG_PRATE]    = P_TIME_US(TMD3702_PROXMITY_TIME);	//P_TIME_US(INTEGRATION_CYCLE)
    sh[TMD3702_REG_PGCFG1]   = (chip->params.prox_gain << TMD3702_SHIFT_PGAIN) |
                               chip->params.prox_drive;
    /*TODO remove after new datasheet revision?*/
    sh[TMD3702_REG_CFG4]     = 0x3D;
    //set PTAT to improve acc over ambient temp
    sh[TMD3702_REG_TEST3]    = 0xC4;
    //set to 1 to reduce vcsel current / 2
    sh[TMD3702_REG_CFG1]     |= (1 << 5);

	SENSOR_LOG_INFO("Loading pltform data %d,%d\n",chip->params.prox_th_max,chip->params.prox_th_min);
//	tmd3702_flush_regs(chip);
    SENSOR_LOG_INFO("prox_pulse_len:%x PGCFG0 :0x%x\n",chip->params.prox_pulse_len,sh[TMD3702_REG_PGCFG0] );


}

#ifdef ABI_SET_GET_REGISTERS
/* bitmap of registers that are in use */
static u8 reginuse[MAX_REGS / 8] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,    /* 0x00 - 0x3f */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,    /* 0x40 - 0x7f */
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,    /* 0x80 - 0xbf */
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,    /* 0xc0 - 0xff */
};

static ssize_t tmd3702_regs_get(struct tmd3702_chip *chip,
                                char *buf, int bufsiz)
{
    u8 regval[16];
    int i, j, cnt;

    // find first
    for (i = 0; i < sizeof(reginuse) / sizeof(reginuse[0]); i++) {
        if (reginuse[i] != 0)
            break;
    }

    i &= ~1;  // round down to the start of a group of 16
    i *= 8;  // set to actual register id

    cnt = 0;
    for (; i < MAX_REGS; i += 16) {
        cnt += snprintf(buf + cnt, bufsiz - cnt, "%02x  ", i);

        ams_i2c_blk_read(chip->client, i, &regval[0], 16);

        for (j = 0; j < 16; j++) {

            if (reginuse[(i >> 3) + (j >> 3)] & (1 << (j & 7))) {
                cnt += snprintf(buf + cnt, bufsiz - cnt, " %02x", regval[j]);
            } else {
                cnt += snprintf(buf + cnt, bufsiz - cnt, " --");
            }

            if (j == 7)
                cnt += snprintf(buf + cnt, bufsiz - cnt, "  ");
        }

        cnt += snprintf(buf + cnt, bufsiz - cnt, "\n");
    }

    cnt += snprintf(buf + cnt, bufsiz - cnt, "\n");
    return cnt;

}

void tmd3702_reg_log(struct tmd3702_chip *chip)
{
    char *buf;

    buf = kmalloc(PAGE_SIZE, GFP_KERNEL);
    if (buf) {
        tmd3702_regs_get(chip, &buf[0], PAGE_SIZE);
        printk(KERN_ERR "%s", buf);
        kfree(buf);
    }
    else {
        dev_err(&chip->client->dev, "%s: out of memory!\n", __func__);
    }
}

static ssize_t tmd3702_regs_dump(
    struct device *dev, struct device_attribute *attr, char *buf)
{
    return tmd3702_regs_get(dev_get_drvdata(dev), buf, PAGE_SIZE);
}

static ssize_t tmd3702_reg_set(struct device *dev,
                struct device_attribute *attr,
                const char *buf, size_t size)
{
    int preg;
    int pval;
    int pmask = -1;
    int numparams;
    int rc;
    struct tmd3702_chip *chip = dev_get_drvdata(dev);

    numparams = sscanf(buf, "0x%x:0x%x:0x%x", &preg, &pval, &pmask);
    if (numparams == 0) {
        // try decimal
        numparams = sscanf(buf, "%d:%d:%d", &preg, &pval, &pmask);
    }

    if ( (numparams < 2) || (numparams > 3) )
        return -EINVAL;
    if ( (numparams >= 1) && ((preg < 0) ||
         ((reginuse[(preg >> 3)] & (1 << (preg & 7))) == 0)) )
        return -EINVAL;
    if ( (numparams >= 2) && (preg < 0 || preg > 0xff) )
        return -EINVAL;
    if ( (numparams >= 3) && (pmask < 0 || pmask > 0xff) )
        return -EINVAL;

    AMS_MUTEX_LOCK(&chip->lock);

    if (pmask == -1) {
        rc = ams_i2c_write(chip->client, chip->shadow, preg, pval);
    } else {
        rc = ams_i2c_modify(chip, chip->shadow,
            preg, pmask, pval);
    }

    AMS_MUTEX_UNLOCK(&chip->lock);

    return rc ? rc : size;
}

struct device_attribute tmd3702_attrs[] = {
    __ATTR(regs, 0644, tmd3702_regs_dump, tmd3702_reg_set),
};

int tmd3702_attrs_size = ARRAY_SIZE(tmd3702_attrs);
#endif // #ifdef ABI_SET_GET_REGISTERS

/*
static int tmd3702_add_sysfs_interfaces(struct device *dev,
                                        struct device_attribute *a,
                                        int size)
{
    int i;
    for (i = 0; i < size; i++)
        if (device_create_file(dev, a + i))
            goto undo;
    return 0;
undo:
    for (; i >= 0 ; i--)
        device_remove_file(dev, a + i);
    dev_err(dev, "%s: failed to create sysfs interface\n", __func__);
    return -ENODEV;
}

static void tmd3702_remove_sysfs_interfaces(struct device *dev,
    struct device_attribute *a, int size)
{
    int i;
    for (i = 0; i < size; i++)
        device_remove_file(dev, a + i);
}
*/
static int tmd3702_get_id(struct tmd3702_chip *chip, u8 *id, u8 *rev, u8 *rev2)
{
    ams_i2c_read(chip->client, TMD3702_REG_REVID2, rev2);
    ams_i2c_read(chip->client, TMD3702_REG_REVID, rev);
    ams_i2c_read(chip->client, TMD3702_REG_ID, id);

    return 0;
}


/********************************************************************
*Validate the appropriate ams device is available for this driver
********************************************************************/
static int tmd3702_check_device_id(struct tmd3702_chip *chip)
{
	int ret;
	u8 id, rev, rev2, i;

	ret = tmd3702_get_id(chip, &id, &rev, &rev2);
	if (ret < 0) {
		SENSOR_LOG_ERROR("i2c read fail\n");
		return -ENODEV;
	}

    SENSOR_LOG_INFO("device id:%02x device revid:%02x device revid_2:%02x\n",id, rev, rev2);


	id &= 0xfc;
	rev &= 0x07;

	for (i = 0; i < ARRAY_SIZE(tmd3702_ids)/3; i++) {
		if (id == (tmd3702_ids[i*3+0]))
			if (rev == (tmd3702_ids[i*3+1]))
				if (rev2 == (tmd3702_ids[i*3+2]))
		break;
	}
	if (i < ARRAY_SIZE(tmd3702_names)) {
		//dev_info(dev, "%s: '%s revid_2: 0x%x' detected\n", __func__,   tmd3702_names[i], rev2);
		SENSOR_LOG_INFO(" '%s revid_2: 0x%x' detected\n",    tmd3702_names[i], rev2);
		chip->device_index = i;
	}
	else {
		///dev_err(dev, "%s: not supported chip id\n", __func__);
		SENSOR_LOG_ERROR("not supported chip id\n");
		ret = -EOPNOTSUPP;
		return -ENODEV;
	}

	return 0;
}
/************************************************/
/* Specific Device Setup.  Configured in Probe. */
/************************************************/

#ifdef CONFIG_OF
int tmd3702_init_dt(struct tmd3702_i2c_platform_data *pdata)
{
    struct device_node *np = pdata->of_node;
    const char *str;
    u32 val;

    if (!pdata->of_node)
           return 0;
/* irq gpio */
    if (!of_property_read_string(np, "prox_name", &str))
        pdata->prox_name = str;

    if (!of_property_read_string(np, "als_name", &str))
        pdata->als_name = str;

    if (!of_property_read_u32(np, "persist", &val))
        pdata->parameters.persist = val;

    if (!of_property_read_u32(np, "prox_th_min", &val))
        pdata->parameters.prox_th_min = val;

    if (!of_property_read_u32(np, "prox_th_max", &val))
        pdata->parameters.prox_th_max = val;

    if (!of_property_read_u32(np, "prox_pulse_cnt", &val))
        pdata->parameters.prox_pulse_cnt = val;

    if (!of_property_read_u32(np, "prox_apc", &val))
        pdata->parameters.prox_apc = val;

    if (!of_property_read_u32(np, "prox_pulse_len", &val))
        pdata->parameters.prox_pulse_len = val;

    if (!of_property_read_u32(np, "prox_pulse_16x", &val))
        pdata->parameters.prox_pulse_16x = val;

    if (!of_property_read_u32(np, "prox_gain", &val))
        pdata->parameters.prox_gain = val;

    if (!of_property_read_u32(np, "poffset", &val))
        pdata->parameters.poffset = val;

    if (!of_property_read_u32(np, "prox_drive", &val))
        pdata->parameters.prox_drive = val;

    if (!of_property_read_u32(np, "als_gain", &val))
        pdata->parameters.als_gain = val;

    if (!of_property_read_u32(np, "als_deltap", &val))
        pdata->parameters.als_deltaP = val;

    if (!of_property_read_u32(np, "als_time", &val))
        pdata->parameters.als_time = val;

    if (!of_property_read_u32(np, "dgf", &val))
        pdata->parameters.dgf = val;

    if (!of_property_read_u32(np, "ct_coef", &val))
        pdata->parameters.ct_coef = val;

    if (!of_property_read_u32(np, "ct_offset", &val))
        pdata->parameters.ct_offset = val;

    if (!of_property_read_u32(np, "c_coef", &val))
        pdata->parameters.c_coef = val;

    if (!of_property_read_u32(np, "r_coef", &val))
        pdata->parameters.r_coef = val;

    if (!of_property_read_u32(np, "g_coef", &val))
        pdata->parameters.g_coef = val;

    if (!of_property_read_u32(np, "b_coef", &val))
        pdata->parameters.b_coef = val;

    if (!of_property_read_u32(np, "coef_scale", &val))
		pdata->parameters.coef_scale = val;

    return 0;
}

static const struct of_device_id tmd3702_of_match[] = {
  { .compatible = "ams,tmd3702" },
  { }
};
MODULE_DEVICE_TABLE(of, tmd3702_of_match);
#endif
/********************************************************************
*prepare ams device state
********************************************************************/
static int tmd3702_parse_dt(struct device *dev,
               struct tmd3702_i2c_platform_data *pdata,struct tmd3702_chip *data) {

	struct device_node *np = dev->of_node;
	const char *str;
	u32 val;
	unsigned int tmp = 0;
	int rc = 0;
	bool supported;

	/* irq gpio */
	rc = of_get_named_gpio(dev->of_node,"tmd,irq-gpio", 0);
	if (rc < 0) {
		SENSOR_LOG_ERROR("Unable to read irq gpio\n");
		return rc;
	}
	data->irq_gpio = rc;
	SENSOR_LOG_INFO("irq gpio is %d\n", data->irq_gpio);

	/* ps tuning data*/
	supported = of_property_read_u32(np, "tmd,power_always_on", &tmp);
	pdata->power_always_on = tmp;
	SENSOR_LOG_INFO("power_always_on is %d\n", pdata->power_always_on);

	supported = of_property_read_u32(np, "tmd,has_als", &tmp);
	//pdata->has_als = tmp;
	SENSOR_LOG_INFO("has_als is %s\n", pdata->has_als?"true":"false");

	supported = of_property_read_u32(np, "tmd,has_ps", &tmp);
	pdata->has_ps = tmp;
	SENSOR_LOG_INFO("has_ps is %s\n", pdata->has_ps?"true":"false");

	if (!of_property_read_string(np, "prox_name", &str))
		pdata->prox_name = str;

	SENSOR_LOG_INFO("prox_name %s\n", str);

    if (!of_property_read_string(np, "als_name", &str))
        pdata->als_name = str;

    if (!of_property_read_u32(np, "persist", &val))
        pdata->parameters.persist = val;

    if (!of_property_read_u32(np, "prox_th_min", &val))
        pdata->parameters.prox_th_min = val;

      SENSOR_LOG_INFO("prox_th_min %d\n", val);

    if (!of_property_read_u32(np, "prox_th_max", &val))
        pdata->parameters.prox_th_max = val;

    SENSOR_LOG_INFO("prox_th_max %d\n", val);

    if (!of_property_read_u32(np, "prox_pulse_cnt", &val))
        pdata->parameters.prox_pulse_cnt = val;

    if (!of_property_read_u32(np, "prox_apc", &val))
        pdata->parameters.prox_apc = val;

    if (!of_property_read_u32(np, "prox_pulse_len", &val))
        pdata->parameters.prox_pulse_len = val;

    if (!of_property_read_u32(np, "prox_pulse_16x", &val))
        pdata->parameters.prox_pulse_16x = val;

    if (!of_property_read_u32(np, "prox_gain", &val))
        pdata->parameters.prox_gain = val;

    if (!of_property_read_u32(np, "poffset", &val))
        pdata->parameters.poffset = val;

    if (!of_property_read_u32(np, "prox_drive", &val))
        pdata->parameters.prox_drive = val;

    if (!of_property_read_u32(np, "als_gain", &val))
        pdata->parameters.als_gain = val;

    if (!of_property_read_u32(np, "als_deltap", &val))
        pdata->parameters.als_deltaP = val;

    if (!of_property_read_u32(np, "als_time", &val))
        pdata->parameters.als_time = val;

    if (!of_property_read_u32(np, "dgf", &val))
        pdata->parameters.dgf = val;

    if (!of_property_read_u32(np, "ct_coef", &val))
        pdata->parameters.ct_coef = val;

    if (!of_property_read_u32(np, "ct_offset", &val))
        pdata->parameters.ct_offset = val;

    if (!of_property_read_u32(np, "c_coef", &val))
        pdata->parameters.c_coef = val;

    if (!of_property_read_u32(np, "r_coef", &val))
        pdata->parameters.r_coef = val;

    if (!of_property_read_u32(np, "g_coef", &val))
        pdata->parameters.g_coef = val;

    if (!of_property_read_u32(np, "b_coef", &val))
        pdata->parameters.b_coef = val;

    if (!of_property_read_u32(np, "coef_scale", &val))
        pdata->parameters.coef_scale = val;

	SENSOR_LOG_INFO("pdata->parameters.b_coef %d\n", pdata->parameters.b_coef);
    SENSOR_LOG_INFO("pdata->parameters.dgf %d\n", pdata->parameters.dgf);
    SENSOR_LOG_INFO("pdata->parameters.ct_coef %d\n", pdata->parameters.ct_coef);
    SENSOR_LOG_INFO("pdata->parameters.ct_offset %d\n", pdata->parameters.ct_offset);
    SENSOR_LOG_INFO("pdata->parameters.c_coef %d\n", pdata->parameters.c_coef);
    SENSOR_LOG_INFO("pdata->parameters.r_coef %d\n", pdata->parameters.r_coef);
    SENSOR_LOG_INFO("pdata->parameters.b_coef %d\n", pdata->parameters.b_coef);
    SENSOR_LOG_INFO("pdata->parameters.g_coef %d\n", pdata->parameters.g_coef);
    SENSOR_LOG_INFO("pdata->parameters.coef_scale %d\n", pdata->parameters.coef_scale);
	return 0;
}
static int tmd3702_update_enable_reg(struct tmd3702_chip *chip)
{
	return ams_i2c_write(chip->client, chip->shadow, TMD3702_REG_ENABLE,
			chip->shadow[TMD3702_REG_ENABLE]);
}
int tmd3702_device_warm_up(struct tmd3702_chip *chip)
{
	int ret;
	ams_i2c_write(chip->client, chip->shadow, TMD3702_REG_ENABLE, 0x01);
	ret = tmd3702_update_enable_reg(chip);
	if (ret < 0) {
		SENSOR_LOG_ERROR("update enable reg fail\n");
		return ret;
	}
	return 0;
}
/********************************************************************
* struct tmd2725_platfotm_ops
********************************************************************/
static struct tmd3702_platfotm_ops p_ops = {
	.platform_init = tmd3702_common_hw_init,
	.platform_power = tmd3702_platform_hw_power_on,
	.platform_exit = tmd3702_platform_hw_exit,
};
static int tmd3702_probe(struct i2c_client *client,
                         const struct i2c_device_id *idp)
{
    int  ret;

    struct device *dev = &client->dev;
    static struct tmd3702_chip *chip;
    struct tmd3702_i2c_platform_data *pdata = dev->platform_data;


    SENSOR_LOG_INFO("probe start :\n");

    /****************************************/
    /* Validate bus and device registration */
    /****************************************/


    SENSOR_LOG_INFO("client->irq = %d\n",  client->irq);

	chip = kzalloc( sizeof(*chip), GFP_KERNEL);
	//chip = devm_kzalloc(dev, sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		ret = -ENOMEM;
		goto exit_alloc_failed;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(dev, "%s: i2c smbus byte data unsupported\n", __func__);
		ret = -EOPNOTSUPP;
		goto exit_alloc_failed;
	}


	if (client->dev.of_node)
	{
		pdata = kzalloc( sizeof(struct tmd3702_i2c_platform_data),GFP_KERNEL);
		if (!pdata) {
			SENSOR_LOG_ERROR("Failed to allocate memory\n");
			ret = -ENOMEM;
			goto exit_alloc_failed;
		}
		client->dev.platform_data = pdata;
	}

	chip->client = client;
	chip->pdata = pdata;
	chip->pdata->ops = &p_ops;

	//wakeup_source_init(&chip->ps_wlock, "tmd3702");
	mutex_init(&chip->lock);
	mutex_init(&chip->pdata->lock);
	mutex_init(&chip->pdata->i2c_lock);

	i2c_set_clientdata(client, chip);

	//pdata->platform_power=&tmd3702_platform_hw_power_on;
    /********************************************************************/
    /* Validate the appropriate ams device is available for this driver */
    /********************************************************************/

	ret = sensor_regulator_configure(chip, true);
	if (ret < 0) {
		SENSOR_LOG_ERROR("unable to configure regulator\n");
		goto exit_alloc_failed;
	}

	ret = sensor_hw_pinctrl_init(chip, &chip->client->dev);
	if (ret < 0) {
		SENSOR_LOG_ERROR("pinctrl init error\n");
		goto exit_alloc_failed;
	}


	/* validate if hw exits */
	ret = tmd3702_check_device_id(chip);
	if (ret < 0){
		SENSOR_LOG_ERROR("check device info error\n");
		goto exit_regulator_put;
	}

	/* chip hw init */
	if (pdata->ops->platform_init) {
		/* parse device tree */
		ret = tmd3702_parse_dt(&client->dev, pdata, chip);
		ret = pdata->ops->platform_init(chip);		//
		if (ret < 0)
			goto exit_regulator_put;
	}

	tmd3702_common_hw_init(chip);

	SENSOR_LOG_INFO("irq gpio is %d\n", chip->irq_gpio);

/* Promblem Number: PR000     Author:xuxiaohua,   Date:2018/8/6
   Description    : add has_als variable value  */
    chip->has_als =pdata->has_als;
    chip->has_ps =pdata->has_ps;
/* END:   Added by xuxiaohua, 2018/8/6 */

	if (pdata->has_als) {
		pdata->als_name = INPUT_ALS_NAME;
		ret = tmd3702_als_device_register(chip, &tmd3702_driver);
		if (ret < 0) {
			SENSOR_LOG_ERROR("als device register error\n");
			goto exit_platform_init_failed;
		}
	}

	if (pdata->has_ps) {
        chip->p_idev = virtual_ps_input_dev;
		pdata->prox_name = INPUT_PS_NAME;
		ret = tmd3702_ps_device_register(chip, &tmd3702_driver);
		if (ret < 0) {
			SENSOR_LOG_ERROR("ps device register error\n");
			goto exit_als_unregister;
		}

	}

	ret = tmd3702_device_warm_up(chip);
	if (ret < 0) {
		SENSOR_LOG_ERROR("enable als/ps fail.\n");
		goto exit_ps_unregister;
	}


 /************************************************************************************************************/
 /* Set chip defaults */
/************************************************************************************************************/

    SENSOR_LOG_INFO("Probe ok.. %s\n",TMD3702_DRIVER_VERSION);
	return 0;

    /************************************************************************/
    /* Exit points for device functional failures (Prox, ALS)               */
    /* This must be unwound in the correct order,                           */
    /* reverse from initialization above                                    */
    /************************************************************************/
exit_ps_unregister:
	if (chip->pdata->has_ps)
		tmd3702_ps_device_unregister(chip);
exit_als_unregister:
	if (chip->pdata->has_als)
		tmd3702_als_device_unregister(chip);
exit_platform_init_failed:
	if (chip->pdata->ops->platform_exit)
		chip->pdata->ops->platform_exit(chip);
exit_regulator_put:
	sensor_regulator_configure(chip, false);
exit_alloc_failed:


	mutex_destroy(&chip->lock);
	mutex_destroy(&chip->pdata->lock);
	mutex_destroy(&chip->pdata->i2c_lock);

	if (chip->pdata)
		kfree(chip->pdata);
	if (chip)
		kfree(chip);


    return ret;
}

static int tmd3702_suspend(struct device *dev)
{
    	struct tmd3702_chip *chip = dev_get_drvdata(dev);
    /* struct tmd3702_i2c_platform_data *pdata = dev->platform_data; */

 	 SENSOR_LOG_INFO( "TMD3702: suspend()\n");
	 AMS_MUTEX_LOCK(&chip->lock);

	chip->wakeup_from_suspend = true;

	chip->in_suspend = 1;

	if (chip->als_enabled)
		cancel_delayed_work(&chip->als_work);

	if (chip->wake_irq) {
		irq_set_irq_wake(chip->client->irq, 1);
	}

   	AMS_MUTEX_UNLOCK(&chip->lock);
	return 0;

}

static int tmd3702_resume(struct device *dev)
{
	struct tmd3702_chip *chip = dev_get_drvdata(dev);
	bool als_on=0, prx_on=0;

	  chip->in_suspend = 0;

    SENSOR_LOG_INFO("TMD3702: resume()\n");
    AMS_MUTEX_LOCK(&chip->lock);

     SENSOR_LOG_INFO("powerd %d, als: needed %d  enabled %d",   !chip->unpowered, als_on,chip->als_enabled);
     SENSOR_LOG_INFO("prox: needed %d  enabled %d\n",   prx_on, chip->prx_enabled);

	if (chip->als_enabled)
		schedule_delayed_work(&chip->als_work, msecs_to_jiffies(0));

	if (chip->wake_irq) {
		irq_set_irq_wake(chip->client->irq, 0);
		//chip->wake_irq = 0;  ????
	}

	/* err_power: */
        AMS_MUTEX_UNLOCK(&chip->lock);
	return 0;
}

static int tmd3702_remove(struct i2c_client *client)
{
	struct tmd3702_chip *chip = i2c_get_clientdata(client);

	SENSOR_LOG_INFO("enter \n");

	if (chip->pdata->has_ps)
		tmd3702_ps_device_unregister(chip);
	if (chip->pdata->has_als)
		tmd3702_als_device_unregister(chip);

	if (chip->pdata->ops->platform_exit)
		chip->pdata->ops->platform_exit(chip);


	mutex_destroy(&chip->lock);
	mutex_destroy(&chip->pdata->lock);
	mutex_destroy(&chip->pdata->i2c_lock);

	i2c_set_clientdata(client, NULL);
	if (chip)
		kfree(chip);

 	 return 0;

}

static struct i2c_device_id tmd3702_idtable[] = {
    { "tmd3702", 0 },
    {}
};
MODULE_DEVICE_TABLE(i2c, tmd3702_idtable);

static const struct dev_pm_ops tmd3702_pm_ops = {
    .suspend = tmd3702_suspend,
    .resume  = tmd3702_resume,
};

static struct i2c_driver tmd3702_driver = {
    .driver = {
        .name = "tmd3702",
        .pm = &tmd3702_pm_ops,
#ifdef CONFIG_OF
        .of_match_table = of_match_ptr(tmd3702_of_match),
#endif
    },
    .id_table = tmd3702_idtable,
    .probe = tmd3702_probe,
    .remove = tmd3702_remove,
};

//module_i2c_driver(tmd3702_driver);
int tmd3702_init(void)
{
	int rc;
	SENSOR_LOG_INFO("TMD3702: init()\n");
	rc = i2c_add_driver(&tmd3702_driver);
	return rc;
}

void tmd3702_exit(void)
{
	SENSOR_LOG_INFO("TMD3702: exit()\n");
	i2c_del_driver(&tmd3702_driver);
}

/*
module_init(tmd3702_init);
module_exit(tmd3702_exit);

MODULE_AUTHOR("AMS AOS Software<cs.americas@ams.com>");
MODULE_DESCRIPTION("AMS-TAOS tmd3702 ALS, Prox, Color sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.1");
*/
