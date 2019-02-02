/* Copyright (c) 2017, The Linux Foundation. All rights reserved.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 and
* only version 2 as published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*/

#include "cam_eeprom_dev.h"
#include "cam_req_mgr_dev.h"
#include "cam_eeprom_soc.h"
#include "cam_eeprom_core.h"
#include "cam_debug_util.h"

#ifdef CONFIG_NUBIA_CAMERA_DUAL_CAMERA_IMX519
#define CAL_DATA_ADDR    0x0BEF
#define CAL_DATA_NUM    2048
#else
#define CAL_DATA_ADDR    0x0B81
#define CAL_DATA_NUM    2048
#endif
//IOCTRL CMD
#define CAM_NUBIA_3DTEST_WRITE_EEPROM            0x100

#define CAM_NUBIA_GET_MAIN_EEPROM_DATA          0x200
#define CAM_NUBIA_GET_AUX_EEPROM_DATA           0x201

#define CAM_NUBIA_SET_OIS_OFF                             0x310
#define CAM_NUBIA_SET_OIS_ON                              0x311

//ZTEMT:fengxun modify for AF calibration --- Start
#define CAM_NUBIA_SET_AF_CAL_DATA                         0x410
#define CAM_NUBIA_GET_AF_CAL_DATA                         0x411
struct af_eeprom_write_t
{
    uint32_t num_bytes;
    uint32_t data0;
};
static struct af_eeprom_write_t g_afdata;
//ZTEMT:fengxun modify for AF calibration --- End

extern void msm_ois_lc898124_enable(int enable);

static uint8_t *memptr_main = NULL;
static uint32_t num_main = 0;
static uint8_t *memptr_aux = NULL;
static uint32_t num_aux = 0;

static int nubia_eeprom_major;
struct nubia_eeprom_dev_t
{
	struct cdev cdev;
};
struct nubia_eeprom_dev_t *nubia_eeprom_dev;


static struct camera_io_master eeprom_master_info;


//ZTEMT:fengxun modify for AF calibration --- Start
int32_t cam_nubia_eeprom_read_af_data(void)
{
    int32_t rc=0;
    unsigned char buf[10] = {0};

    rc = camera_io_dev_read_seq(&eeprom_master_info,
        0x1C00, buf,CAMERA_SENSOR_I2C_TYPE_WORD,4);
    if(rc){
        CAM_ERR(CAM_EEPROM, "[FX] read g_afdata failed rc %d", rc);
        camera_io_release(&eeprom_master_info);
        return -1;
    }

    g_afdata.data0 = (buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8) | buf[3];
    g_afdata.num_bytes = 1;

    CAM_ERR(CAM_EEPROM, "[FX] read g_afdata: 0x%x 0x%x 0x%x 0x%x",
        buf[0],buf[1],buf[2],buf[3]);
    CAM_ERR(CAM_EEPROM, "[FX] g_afdata : %d",g_afdata.data0);

    return 0;
}

void cam_nubia_eeprom_write32(unsigned int addr, unsigned int data)
{
    uint8_t reqdata[4];
    int32_t rc=0;
    int i = 0;

    struct cam_sensor_i2c_reg_array reg_setting[4];
    struct cam_sensor_i2c_reg_setting write_setting;
    
    reqdata[0] = (data >> 24) & 0xFF;
    reqdata[1] = (data >> 16) & 0xFF;
    reqdata[2] = (data >> 8) & 0xFF;
    reqdata[3] = (data) & 0xFF;

    reg_setting[0].reg_addr = addr;
    for (i = 0; i < 4; i++) {
        reg_setting[i].reg_data = reqdata[i];
        reg_setting[i].delay = 0;
        reg_setting[i].data_mask = 0;
    }


    write_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
    write_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
    write_setting.delay = 0;
    write_setting.reg_setting = reg_setting;
    write_setting.size = 4;

    rc = cam_cci_i2c_write_continuous_table(&eeprom_master_info,&write_setting,0);

    if (rc < 0) {
        pr_err("%s: line %d rc = %d\n", __func__, __LINE__, rc);
    }
      
};


int32_t cam_nubia_eeprom_write_data_addr(uint32_t addr, unsigned int buf)
{
	int rc = 0;
	uint32_t readdata = 0;
	uint16_t sid_tmp = 0;

	struct cam_sensor_i2c_reg_setting write_setting;
	struct cam_sensor_i2c_reg_array unlock_reg[] =
		{
			{0xFFFF, 0x1D, 0x00, 0x00},
		};
	struct cam_sensor_i2c_reg_array lock_reg[] =
		{
			{0xFFFF, 0x1F, 0x00, 0x00},
		};

	CAM_ERR(CAM_EEPROM, "cam_nubia_eeprom_write_data_addr ---E");
	CAM_ERR(CAM_EEPROM, " CCI_Master %x\n", eeprom_master_info.cci_client->cci_i2c_master);
	CAM_ERR(CAM_EEPROM, " SID %x\n", eeprom_master_info.cci_client->sid);

	//save SID 0x50 (0xA0>>1)
	sid_tmp = eeprom_master_info.cci_client->sid;

	//modify SID to 0xB0 (CSP addr)
	eeprom_master_info.cci_client->sid = 0xB0 >> 1;

	//init io
	if (eeprom_master_info.master_type == CCI_MASTER)
	{
		rc = camera_io_init(&eeprom_master_info);
		if (rc)
		{
			CAM_ERR(CAM_EEPROM, "cci_init failed");
			return -EINVAL;
		}
	}

#if 1
	//for debug CSP
	rc = camera_io_dev_read(&eeprom_master_info, 0xFFFF,
			&readdata, CAMERA_SENSOR_I2C_TYPE_WORD,
			CAMERA_SENSOR_I2C_TYPE_BYTE);
	if (rc < 0)
	{
		CAM_ERR(CAM_EEPROM, "camera_io_dev_read error");
	}
	CAM_ERR(CAM_EEPROM, " read CSP =%x ", readdata);
	usleep_range(10000, 11000);
#endif

#if 1
	//unlock SWP
	write_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	write_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	write_setting.delay = 0;
	write_setting.reg_setting = unlock_reg;
	write_setting.size = 1;
	rc = camera_io_dev_write(&eeprom_master_info, &write_setting);
	if (rc < 0)
	{
		CAM_ERR(CAM_SENSOR, "camera_io_dev_write  [unlock] error");
		return -1;
	}
	usleep_range(10000, 11000);
#endif

#if 1
	//for debug CSP
	rc = camera_io_dev_read(&eeprom_master_info, 0xFFFF,
			&readdata, CAMERA_SENSOR_I2C_TYPE_WORD,
			CAMERA_SENSOR_I2C_TYPE_BYTE);
	if (rc < 0)
	{
		CAM_ERR(CAM_EEPROM, "camera_io_dev_read error");
	}
	CAM_ERR(CAM_EEPROM, " read CSP =%x ", readdata);
	usleep_range(10000, 11000);
#endif

#if 1
	//write data
	//restore SID to 0x50(0xA0 >> 1)
	eeprom_master_info.cci_client->sid = sid_tmp;

	cam_nubia_eeprom_write32(addr, buf);

	usleep_range(10000, 11000);
#endif

#if 1
	//lock SWP
	//modify SID to 0xB0 (CSP addr)
	eeprom_master_info.cci_client->sid = 0xB0 >> 1;

	write_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	write_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	write_setting.delay = 0;
	write_setting.reg_setting = lock_reg;
	write_setting.size = 1;
	rc = camera_io_dev_write(&eeprom_master_info, &write_setting);
	if (rc < 0)
	{
		CAM_ERR(CAM_SENSOR, "camera_io_dev_write  [lock] error");
		return -1;
	}
	usleep_range(10000, 11000);
#endif

#if 1
	//for debug CSP
	rc = camera_io_dev_read(&eeprom_master_info, 0xFFFF,
			&readdata, CAMERA_SENSOR_I2C_TYPE_WORD,
			CAMERA_SENSOR_I2C_TYPE_BYTE);
	if (rc < 0)
	{
		CAM_ERR(CAM_EEPROM, "camera_io_dev_read error");
	}
	CAM_ERR(CAM_EEPROM, " read CSP =%x ", readdata);
	usleep_range(10000, 11000);
#endif


	//restore SID to 0x50(0xA0 >> 1)
	eeprom_master_info.cci_client->sid = sid_tmp;

	//release io
	if (eeprom_master_info.master_type == CCI_MASTER)
	{
		camera_io_release(&eeprom_master_info);
	}

	CAM_ERR(CAM_EEPROM, "cam_nubia_eeprom_write_data_addr ---X");

	return rc;
}
//ZTEMT:fengxun modify for AF calibration --- End

int cam_save_eeprom_data(uint8_t *memptr, uint32_t size, uint32_t saddr)
{

	if (saddr == 0xA0)
	{
		memptr_main = kzalloc(size, GFP_KERNEL);
		if (!memptr_main )
		{
			CAM_ERR(CAM_EEPROM, "memptr_main kzalloc error ");
			return -1;
		}
		else
		{
			CAM_ERR(CAM_EEPROM, "save main eeprom data");
			num_main = size;
			memcpy(memptr_main, memptr, size);
		}
	}
	else if (saddr == 0xA8)
	{
		memptr_aux = kzalloc(size, GFP_KERNEL);
		if (!memptr_aux )
		{
			CAM_ERR(CAM_EEPROM, "memptr_aux kzalloc error ");
			return -1;
		}
		else
		{
			CAM_ERR(CAM_EEPROM, "save aux eeprom data");
			num_aux = size;
			memcpy(memptr_aux, memptr, size);
		}
	}
	return 0;
}
EXPORT_SYMBOL(cam_save_eeprom_data);



int32_t cam_nubia_eeprom_io_init(struct camera_io_master io_master)
{
	int rc = 0;
	eeprom_master_info = io_master;
	CAM_ERR(CAM_EEPROM, " SID %x\n", eeprom_master_info.cci_client->sid);

    //ZTEMT:fengxun modify for AF calibration --- Start
    if(0 == g_afdata.num_bytes){
        cam_nubia_eeprom_read_af_data();
    }
    //ZTEMT:fengxun modify for AF calibration --- End

	return rc;
}
EXPORT_SYMBOL(cam_nubia_eeprom_io_init);


int32_t cam_nubia_i2c_write_seq(uint32_t addr, uint8_t *data, uint32_t num_byte)
{
	int32_t rc = -EFAULT;
	int write_num = 0;
	int write_width = 32;
	int i = 0;
	int j = 0;

	struct cam_sensor_i2c_reg_array write_buf[32];
	struct cam_sensor_i2c_reg_setting write_setting;

	CAM_ERR(CAM_EEPROM, "%s  ---E\n", __func__);
	CAM_ERR(CAM_EEPROM, "num_byte = %d\n", num_byte);

	if (num_byte < 32)
	{
		CAM_ERR(CAM_EEPROM, "%s  num_byte is smaller than 32 ,error\n", __func__);
		goto END;
	}

	//frist_write_width
	write_width = 32 - (addr % 32);

	for (i = 0; i < num_byte;)
	{
		//update write_width
		if (num_byte - write_num < 32)
		{
			write_width = num_byte - write_num;
		}
		CAM_ERR(CAM_EEPROM, "remain num = %d addr= %x write_width = %d \n",
				num_byte - write_num, addr + i, write_width);

		//update write_buf
		for (j = 0; j < write_width; j++)
		{
			write_buf[j].reg_data = data[write_num + j];
			write_buf[j].delay = 0;
			write_buf[j].data_mask = 0;
		}

		write_buf[0].reg_addr = addr + i;
		write_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
		write_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
		write_setting.delay = 0;
		write_setting.reg_setting = write_buf;
		write_setting.size = write_width;

		rc = cam_cci_i2c_write_continuous_table(&eeprom_master_info, &write_setting, 0);
		if ( rc < 0)
		{
			CAM_ERR(CAM_EEPROM, "%s nubia_i2c_write_seq error\n", __func__);
			goto END;
		}

		write_num += write_width;
		i += write_width;
		write_width = 32;

		usleep_range(5000, 6000);
	}

	CAM_ERR(CAM_EEPROM, "%s  total write num = %d\n", __func__, write_num);

END:
	CAM_ERR(CAM_EEPROM, "%s  ---X\n", __func__);
	return rc;
}

int32_t cam_nubia_eeprom_write_data(unsigned char *buf, uint32_t num_bytes)
{
	int rc = 0;
	uint32_t readdata = 0;
	uint16_t sid_tmp = 0;

	struct cam_sensor_i2c_reg_setting write_setting;
	struct cam_sensor_i2c_reg_array unlock_reg[] =
		{
			{0xFFFF, 0x1D, 0x00, 0x00},
		};
	struct cam_sensor_i2c_reg_array lock_reg[] =
		{
			{0xFFFF, 0x1F, 0x00, 0x00},
		};

	CAM_ERR(CAM_EEPROM, "cam_nubia_eeprom_write_data ---E");
	CAM_ERR(CAM_EEPROM, " CCI_Master %x\n", eeprom_master_info.cci_client->cci_i2c_master);
	CAM_ERR(CAM_EEPROM, " SID %x\n", eeprom_master_info.cci_client->sid);

	//save SID 0x50 (0xA0>>1)
	sid_tmp = eeprom_master_info.cci_client->sid;

	//modify SID to 0xB0 (CSP addr)
	eeprom_master_info.cci_client->sid = 0xB0 >> 1;

	//init io
	if (eeprom_master_info.master_type == CCI_MASTER)
	{
		rc = camera_io_init(&eeprom_master_info);
		if (rc)
		{
			CAM_ERR(CAM_EEPROM, "cci_init failed");
			return -EINVAL;
		}
	}

#if 1
	//for debug CSP
	rc = camera_io_dev_read(&eeprom_master_info, 0xFFFF,
			&readdata, CAMERA_SENSOR_I2C_TYPE_WORD,
			CAMERA_SENSOR_I2C_TYPE_BYTE);
	if (rc < 0)
	{
		CAM_ERR(CAM_EEPROM, "camera_io_dev_read error");
	}
	CAM_ERR(CAM_EEPROM, " read CSP =%x ", readdata);
	usleep_range(10000, 11000);
#endif

#if 1
	//unlock SWP
	write_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	write_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	write_setting.delay = 0;
	write_setting.reg_setting = unlock_reg;
	write_setting.size = 1;
	rc = camera_io_dev_write(&eeprom_master_info, &write_setting);
	if (rc < 0)
	{
		CAM_ERR(CAM_SENSOR, "camera_io_dev_write  [unlock] error");
		return -1;
	}
	usleep_range(10000, 11000);
#endif

#if 1
	//for debug CSP
	rc = camera_io_dev_read(&eeprom_master_info, 0xFFFF,
			&readdata, CAMERA_SENSOR_I2C_TYPE_WORD,
			CAMERA_SENSOR_I2C_TYPE_BYTE);
	if (rc < 0)
	{
		CAM_ERR(CAM_EEPROM, "camera_io_dev_read error");
	}
	CAM_ERR(CAM_EEPROM, " read CSP =%x ", readdata);
	usleep_range(10000, 11000);
#endif

#if 1
	//write data
	//restore SID to 0x50(0xA0 >> 1)
	eeprom_master_info.cci_client->sid = sid_tmp;

	rc = cam_nubia_i2c_write_seq(CAL_DATA_ADDR, buf, num_bytes);

	usleep_range(10000, 11000);
#endif

#if 1
	//lock SWP
	//modify SID to 0xB0 (CSP addr)
	eeprom_master_info.cci_client->sid = 0xB0 >> 1;

	write_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	write_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	write_setting.delay = 0;
	write_setting.reg_setting = lock_reg;
	write_setting.size = 1;
	rc = camera_io_dev_write(&eeprom_master_info, &write_setting);
	if (rc < 0)
	{
		CAM_ERR(CAM_SENSOR, "camera_io_dev_write  [lock] error");
		return -1;
	}
	usleep_range(10000, 11000);
#endif

#if 1
	//for debug CSP
	rc = camera_io_dev_read(&eeprom_master_info, 0xFFFF,
			&readdata, CAMERA_SENSOR_I2C_TYPE_WORD,
			CAMERA_SENSOR_I2C_TYPE_BYTE);
	if (rc < 0)
	{
		CAM_ERR(CAM_EEPROM, "camera_io_dev_read error");
	}
	CAM_ERR(CAM_EEPROM, " read CSP =%x ", readdata);
	usleep_range(10000, 11000);
#endif


	//restore SID to 0x50(0xA0 >> 1)
	eeprom_master_info.cci_client->sid = sid_tmp;

	//release io
	if (eeprom_master_info.master_type == CCI_MASTER)
	{
		camera_io_release(&eeprom_master_info);
	}

	CAM_ERR(CAM_EEPROM, "cam_nubia_eeprom_write_data ---X");

	return rc;
}



static long cam_nubia_eeprom_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int rc = 0;
	CAM_ERR(CAM_EEPROM, " cam_nubia_eeprom_ioctl cmd %x ", cmd);
	return rc;
}

static long cam_nubia_eeprom_ioctl32 (struct file *filp, unsigned int cmd, unsigned long arg)
{
	int rc = 0;
	unsigned char *buf;
	int i =0;
	int check_sum = 0;

	struct eeprom_write_t
	{
		uint32_t dbuffer;
		uint32_t num_bytes;
	}
	cmd_data;

	//ZTEMT:fengxun modify for AF calibration --- Start
	struct af_eeprom_write_t af_data;
	//ZTEMT:fengxun modify for AF calibration --- End

	CAM_ERR(CAM_EEPROM, " cam_nubia_eeprom_ioctl32 cmd %x ", cmd);
	CAM_ERR(CAM_EEPROM, " CAL_DATA_ADDR %x ", CAL_DATA_ADDR);
	if(NULL == (void __user *)arg){
		CAM_ERR(CAM_EEPROM, "arg is NULL,error");
		return -EFAULT;
	}

    //for get main eeprom data
    if(0x200 == cmd){
	if (NULL == memptr_main){
		CAM_ERR(CAM_EEPROM, "memptr_main is NULL,error");
		return -EFAULT;
	}

	if (copy_from_user(&cmd_data, (void __user *)arg, sizeof(cmd_data)))
	{
		CAM_ERR(CAM_EEPROM, "MAIN Failed to copy from user_ptr=%pK size=%zu",
				(void __user *)arg, sizeof(cmd_data));
		return -EFAULT;
	}

	if(copy_to_user((void __user *)(unsigned long)cmd_data.dbuffer,memptr_main ,num_main))
	{
		CAM_ERR(CAM_EEPROM, "MAIN %s:%d failed\n", __func__, __LINE__);
		return -EFAULT;
	}
	cmd_data.num_bytes = num_main;


	if (copy_to_user((void __user *)arg, &cmd_data, sizeof(cmd_data)))
	{
		CAM_ERR(CAM_EEPROM, "MAIN Failed to copy to user_ptr=%pK size=%zu",
				(void __user *)arg, sizeof(cmd_data));
		return -EFAULT;
	}


	return rc;
    }

    //for get aux eeprom data
    if(0x201 == cmd){
	if (NULL == memptr_aux){
		CAM_ERR(CAM_EEPROM, "memptr_aux is NULL,error");
		return -EFAULT;
	}

	if (copy_from_user(&cmd_data, (void __user *)arg, sizeof(cmd_data)))
	{
		CAM_ERR(CAM_EEPROM, "AUX Failed to copy to user_ptr=%pK size=%zu",
				(void __user *)arg, sizeof(cmd_data));
		return -EFAULT;
	}

	if(copy_to_user((void __user *)(unsigned long)cmd_data.dbuffer,memptr_aux ,num_aux))
	{
		CAM_ERR(CAM_EEPROM, "AUX %s:%d failed\n", __func__, __LINE__);
		return -EFAULT;
	}
	cmd_data.num_bytes = num_aux;

	if (copy_to_user((void __user *)arg, &cmd_data, sizeof(cmd_data)))
	{
		CAM_ERR(CAM_EEPROM, "AUX Failed to copy to user_ptr=%pK size=%zu",
				(void __user *)arg, sizeof(cmd_data));
		return -EFAULT;
	}

	return rc;
    }


	//for 3Dtest write eeprom data
	if(0x100 == cmd){
	if (copy_from_user(&cmd_data, (void __user *)arg, sizeof(cmd_data)))
	{
		CAM_ERR(CAM_EEPROM, "Failed to copy from user_ptr=%pK size=%zu",
				(void __user *)arg, sizeof(cmd_data));
		return -EFAULT;
	}

	//CAM_ERR(CAM_EEPROM, " num_bytes = %x dbuffer = %p ", cmd_data.num_bytes, cmd_data.dbuffer);

	if (CAL_DATA_NUM == cmd_data.num_bytes)
	{
		buf = kzalloc(cmd_data.num_bytes + 1, GFP_KERNEL);
		if (!buf )
		{
			CAM_ERR(CAM_EEPROM, " kzalloc error ");
			rc = -ENOMEM;
			return rc;
		}

		if (copy_from_user(buf, (void __user *)(unsigned long)cmd_data.dbuffer , cmd_data.num_bytes))
		{
			CAM_ERR(CAM_EEPROM, "%s:%d failed\n", __func__, __LINE__);
			kfree(buf);
			rc = -EFAULT;
			return rc;
		}

		//new check_sum
		for(i = 0; i < cmd_data.num_bytes; i++){
			check_sum += buf[i];
		}
		check_sum = check_sum % 256;
		CAM_ERR(CAM_EEPROM, "%s:%d new check_sum is %x\n", __func__, __LINE__, check_sum);
		buf[cmd_data.num_bytes] = check_sum & 0xFF;
		CAM_ERR(CAM_EEPROM, "%s:%d new check_sum is %x\n", __func__, __LINE__, buf[cmd_data.num_bytes]);

		cam_nubia_eeprom_write_data(buf, CAL_DATA_NUM + 1);

		kfree(buf);

	}
	else
	{
		CAM_ERR(CAM_EEPROM, " num_bytes error ");
	}

	}

	//for enable ois
	if(CAM_NUBIA_SET_OIS_OFF == cmd){
		msm_ois_lc898124_enable(0);
	}else if(CAM_NUBIA_SET_OIS_ON == cmd){
		msm_ois_lc898124_enable(1);
	}


	//ZTEMT:fengxun modify for AF calibration --- Start
    //for write AF data to eeprom
    if(CAM_NUBIA_SET_AF_CAL_DATA == cmd){

		CAM_ERR(CAM_EEPROM, "[FX] CAM_NUBIA_SET_AF_CAL_DATA --- E");

		if(copy_from_user(&af_data, (void __user *)arg, sizeof(af_data)))
		{
			CAM_ERR(CAM_EEPROM, "CAM_NUBIA_SET_AF_CAL_DATA Failed to copy from user_ptr=%pK size=%zu",
					(void __user *)arg, sizeof(af_data));
			return -EFAULT;
		}

		if(1 == af_data.num_bytes){
			CAM_ERR(CAM_EEPROM, "[FX] af_data : %d",af_data.data0);
            //write eeprom
    		cam_nubia_eeprom_write_data_addr(0x1C00,af_data.data0);
		}else{
			CAM_ERR(CAM_EEPROM, "[FX] af_data error");
		}

		CAM_ERR(CAM_EEPROM, "[FX] CAM_NUBIA_SET_AF_CAL_DATA --- X");
		return rc;
    }else if(CAM_NUBIA_GET_AF_CAL_DATA == cmd){

		CAM_ERR(CAM_EEPROM, "[FX] CAM_NUBIA_GET_AF_CAL_DATA");

		//read eeprom
        af_data.data0 = g_afdata.data0;
		af_data.num_bytes = g_afdata.num_bytes;

        CAM_ERR(CAM_EEPROM, "[FX] af_data num_bytes: %d",af_data.num_bytes);
		CAM_ERR(CAM_EEPROM, "[FX] af_data data0: %d",af_data.data0);

		if(copy_to_user((void __user *)arg, &af_data, sizeof(af_data)))
		{
			CAM_ERR(CAM_EEPROM, "CAM_NUBIA_GET_AF_CAL_DATA Failed to copy to user_ptr=%pK size=%zu",
					(void __user *)arg, sizeof(af_data));
			return -EFAULT;
		}

		CAM_ERR(CAM_EEPROM, "[FX] CAM_NUBIA_GET_AF_CAL_DATA");
		return rc;
    }

	//ZTEMT:fengxun modify for AF calibration --- End

	return rc;
}

int cam_nubia_eeprom_open(struct inode *inode, struct file *filp)
{
	CAM_DBG(CAM_EEPROM, " cam_nubia_eeprom_open ");
	return 0;
}


static ssize_t cam_nubia_eeprom_read(struct file *filp, char __user *buf, size_t count, loff_t *ppos)
{
	CAM_DBG(CAM_EEPROM, " cam_nubia_eeprom_read ");
	return 0;
}


static const struct file_operations nubia_eeprom_fops =
{
        .owner = THIS_MODULE,
        .open = cam_nubia_eeprom_open,
        .release = NULL,
        .read = cam_nubia_eeprom_read,
        .unlocked_ioctl = cam_nubia_eeprom_ioctl,
        .compat_ioctl = cam_nubia_eeprom_ioctl32,
};

static void cam_nubia_eeprom_setup_cdev(struct nubia_eeprom_dev_t *dev, int index)
{
	int err, devno = MKDEV(nubia_eeprom_major, index);

	cdev_init(&dev->cdev, &nubia_eeprom_fops);
	dev->cdev.owner = THIS_MODULE;
	dev->cdev.ops = &nubia_eeprom_fops;
	err = cdev_add(&dev->cdev, devno, 1);
	if (err)
	{
		CAM_ERR(CAM_EEPROM, "Error %d add cdevdemo %d", err, index);
	}
}

static int __init cam_nubia_eeprom_init(void)
{
	int rc = 0;
	dev_t devno;

	struct class *cam_nubia_eeprom_class = NULL;

	CAM_ERR(CAM_EEPROM, " cam_nubia_eeprom_init");

	rc = alloc_chrdev_region(&devno, 0, 1, "nubia_eeprom");
	if (rc < 0)
	{
		CAM_ERR(CAM_EEPROM, " alloc_chrdev_region error");
		return rc;
	}
	nubia_eeprom_major = MAJOR(devno);

	nubia_eeprom_dev = kmalloc(sizeof(struct nubia_eeprom_dev_t), GFP_KERNEL);
	if (!nubia_eeprom_dev)
	{
		rc = -ENOMEM;
		CAM_ERR(CAM_EEPROM, " kmalloc error");
		goto fail_malloc;
	}

	memset(nubia_eeprom_dev, 0, sizeof(struct nubia_eeprom_dev_t));

	cam_nubia_eeprom_setup_cdev(nubia_eeprom_dev, 0);

	cam_nubia_eeprom_class = class_create(THIS_MODULE, "nubia_eeprom");
	device_create(cam_nubia_eeprom_class, NULL, MKDEV(nubia_eeprom_major, 0), NULL, "nubia_eeprom");

	CAM_ERR(CAM_EEPROM, " cam_nubia_eeprom_init successful");
	return 0;

fail_malloc:
	unregister_chrdev_region(devno, 1);
	return rc;
}

static void __exit cam_nubia_eeprom_exit(void)
{
	cdev_del(&nubia_eeprom_dev->cdev);
	kfree(nubia_eeprom_dev);
	unregister_chrdev_region(MKDEV(nubia_eeprom_major, 0), 1);
	CAM_ERR(CAM_EEPROM, " cam_eeprom_nubia_exit");
}

module_init(cam_nubia_eeprom_init);
module_exit(cam_nubia_eeprom_exit);
MODULE_DESCRIPTION("CAM NUBIA EEPROM driver");
MODULE_LICENSE("GPL v2");