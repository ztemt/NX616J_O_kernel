/* Copyright (c) 2011-2013, The Linux Foundation. All rights reserved.
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

#define pr_fmt(fmt) "%s:%d " fmt, __func__, __LINE__

#include <linux/err.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include "cam_sensor_io.h"
#include "OisInter.h"

//**************************
//	Include Header File		
//**************************
#include "LC898124EP2_Host_Code.h"	// INVEN 2030 & MTM Y3


//#define MSM_OIS_DEBUG
#undef CDBG
#ifdef MSM_OIS_DEBUG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...) pr_debug(fmt, ##args)
#endif

#define		DeviceAddr		0x7C  	// Device address of driver IC


static struct task_struct *read_task;
static char ois_status;
static char ois_disable;

static struct camera_io_master ois_master_info;

EXPORT_SYMBOL(msm_ois_lc898124_write_dac);
EXPORT_SYMBOL(msm_ois_lc898124_init_AF);
EXPORT_SYMBOL(msm_ois_lc898124_enable);
EXPORT_SYMBOL(CntWrt);
EXPORT_SYMBOL(WitTim);
EXPORT_SYMBOL(RamRead32A);
EXPORT_SYMBOL(RamWrite32A);


/*ZTEMT: fengxun add for AF--------Start*/

/* for I2C communication */ 
void RamWrite32A(unsigned int addr, unsigned int data)
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

    rc = cam_cci_i2c_write_continuous_table(&ois_master_info,&write_setting,0);

    if (rc < 0) {
        pr_err("%s: line %d rc = %d\n", __func__, __LINE__, rc);
    }
      
};
void RamRead32A( unsigned short addr, void * data)
{
    uint32_t *temp_read_data_32 = (uint32_t *)data;
    int32_t rc = 0;
    uint32_t readData = 0;

    rc = camera_io_dev_read(
        &ois_master_info,
        addr,
        &readData, CAMERA_SENSOR_I2C_TYPE_WORD,
        CAMERA_SENSOR_I2C_TYPE_DWORD);
    CDBG("RamRead32A %x \n", readData);
    
    if (rc < 0) 
    {
        pr_err("%s: line %d rc = %d\n", __func__, __LINE__, rc);
    }
    
    *temp_read_data_32=  readData;
    
}
/* for I2C Multi Translation : Burst Mode*/
void CntWrt( void *	PcSetDat, unsigned short UsDatNum )
{
    int rc = 0;
    int i = 0;
    unsigned char *SetDat = (unsigned char *)PcSetDat;
    uint16_t uNum = 0;

    struct cam_sensor_i2c_reg_array *reg_setting;
    struct cam_sensor_i2c_reg_setting write_setting;
    
    if((UsDatNum < 2)){
        pr_err("%s:%d UsDatNum is zero\n", __func__, __LINE__);
        return;
    }
    uNum = UsDatNum - 1;

    reg_setting = kzalloc(uNum *(sizeof(struct cam_sensor_i2c_reg_array)), GFP_KERNEL);
    if (!reg_setting) {
        pr_err("%s:%d no memory\n", __func__, __LINE__);
        return;
    }

    reg_setting[0].reg_addr = SetDat[0];
    for (i = 0; i < uNum; i++) {
        reg_setting[i].reg_data = SetDat[i+1];
        reg_setting[i].delay = 0;
        reg_setting[i].data_mask = 0;
    }


    write_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
    write_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
    write_setting.delay = 0;
    write_setting.reg_setting = reg_setting;
    write_setting.size = uNum;

    rc = cam_cci_i2c_write_continuous_table(&ois_master_info,&write_setting,0);

    if (rc < 0) {
        pr_err("%s: line %d rc = %d\n", __func__, __LINE__, rc);
    }

    return;
}
/* for Wait timer [Need to adjust for your system] */ 
void	WitTim( unsigned short	UsWitTim )
{
    msleep(UsWitTim);
}

/*ZTEMT: fengxun add for AF--------End*/




//****************************************************
//	DEFINES
//****************************************************
#define BURST_LENGTH_PM ( 12*5 ) 	
#define BURST_LENGTH_DM ( 10*6 ) 	
#define BURST_LENGTH BURST_LENGTH_PM 	

#define		CMD_IO_ADR_ACCESS				0xC000				// IO Write Access
#define		CMD_IO_DAT_ACCESS				0xD000				// IO Read Access

#define 	ROMINFO							0xE0500C
#define 	SYSDSP_REMAP					0xD000AC
#define		DmCheck_CheckSumDMB				0x540
#define			CommandDecodeTable_08		(0x0020 + 0x8568)
#define 	PmCheck_CheckSum				0x514

//********************************************************************************
// Function Name 	: DownloadToEP2
//********************************************************************************
unsigned char DownloadToEP2( const unsigned char* DataPM, 
							 unsigned long LengthPM,
							 unsigned long Parity,
							 const unsigned char* DataDM,
							  unsigned long LengthDM ) 
{
	unsigned long i, j;
	unsigned char data[64];
	unsigned char Remainder;
	unsigned long UlReadVal, UlCnt;
	uint32_t ReadVerifyPM, ReadVerifyDM;
	
//--------------------------------------------------------------------------------
      WitTim( 10 ) ;
      RamWrite32A(CMD_IO_ADR_ACCESS, 0xD00100);
      RamRead32A(CMD_IO_DAT_ACCESS, &UlReadVal);
      pr_err("LC898124 Chip type: %x \n", (unsigned char)UlReadVal);
      WitTim( 6 ) ;

	RamWrite32A( CMD_IO_ADR_ACCESS , ROMINFO );
	RamRead32A( CMD_IO_DAT_ACCESS, &UlReadVal );
      CDBG("read ROMINFO %x\n",(unsigned char)UlReadVal); 
    
	switch ( (unsigned char)UlReadVal ){
	case 0x0A:
		break;
	
	case 0x01:	
		RamWrite32A( CMD_IO_ADR_ACCESS, SYSDSP_REMAP ) ;
		RamWrite32A( CMD_IO_DAT_ACCESS, 0x00001000 ) ;
		WitTim( 6 ) ;
		break;

	default:	
		return( 1 );
	}
//--------------------------------------------------------------------------------
	data[0] = 0x30;		// Pmem address set
	data[1] = 0x00;		// Command High
	data[2] = 0x10;		// Command High
	data[3] = 0x00;		// Command High
	data[4] = 0x00;		// Command High
	CntWrt( data, 5 ); 	// I2C 1Byte address.

	// program start
	data[0] = 0x40;		// Pmem address set
	Remainder = ( (LengthPM*5) / BURST_LENGTH_PM ); 
	for(i=0 ; i< Remainder ; i++)
	{
		UlCnt = 1;
		for(j=0 ; j < BURST_LENGTH_PM; j++)	data[UlCnt++] = *DataPM++;
		
		CntWrt( data, BURST_LENGTH_PM+1 ); 
	}
	Remainder = ( (LengthPM*5) % BURST_LENGTH_PM); 
	if (Remainder != 0 )
	{
		UlCnt = 1;
		for(j=0 ; j < Remainder; j++)	data[UlCnt++] = *DataPM++;
		CntWrt( data, UlCnt ); 
	}
	// Chercksum start
	data[0] = 0xF0;											// Pmem address set
	data[1] = 0x0A;											// Command High
	data[2] = (unsigned char)(( LengthPM & 0xFF00) >> 8 );	// Size High
	data[3] = (unsigned char)(( LengthPM & 0x00FF) >> 0 );	// Size Low
	CntWrt( data, 4 ); 	// I2C 2Byte addresss.
//--------------------------------------------------------------------------------
	RamWrite32A( DmCheck_CheckSumDMB, 0 );
	for( i=0; i < LengthDM; i+=6 )
	{
		if ( (DataDM[0+i] == 0x80) && (DataDM[1+i] == 0x0C) )
		{
			RamWrite32A( CommandDecodeTable_08, (((unsigned long)(DataDM[3+i])<<16) + ((unsigned long)(DataDM[4+i])<<8) + DataDM[5+i] ) );
		}
	}
	// Data Send
	Remainder = ( (LengthDM*6/4) / BURST_LENGTH_DM ); 
	for(i=0 ; i< Remainder ; i++)
	{
		CntWrt( (unsigned char*)DataDM, BURST_LENGTH_DM );
		DataDM += BURST_LENGTH_DM;
	}
	Remainder = ( (LengthDM*6/4) % BURST_LENGTH_DM ); 
	if (Remainder != 0 )
	{
		CntWrt( (unsigned char*)DataDM, (unsigned char)Remainder );
	}
//--------------------------------------------------------------------------------

	RamRead32A( PmCheck_CheckSum, &ReadVerifyPM );
	RamRead32A( DmCheck_CheckSumDMB, &ReadVerifyDM );

	RamWrite32A( CommandDecodeTable_08, 0x000418C4 );	

      if( (ReadVerifyPM + ReadVerifyDM ) != Parity  ){
        return( 2 );
      }

	return(0);
}

//********************************************************************************
// Function Name 	: ReMapMain
//********************************************************************************
void RemapMain( void )
{
	RamWrite32A( 0xF000, 0x00000000 ) ;
}

//********************************************************************************
// Function Name 	: EP2Download
//********************************************************************************
unsigned char EP2Download(void )
{
	return( DownloadToEP2( LC898124EP2_PM, LC898124EP2_PMSize, (LC898124EP2_PMCheckSum + LC898124EP2_DMB_CheckSum ), LC898124EP2_DM, LC898124EP2_DMB_ByteSize ) ); 
}

////

void msm_ois_lc898124_write_dac(unsigned int data)
{
    if(0 == ois_status){
        pr_err("msm_ois_lc898124_write_dac ois status = 0 return \n");
        return;
    }
    RamWrite32A(0xF01A, data | 0x00010000);
}

void msm_ois_lc898124_enable(int enable)
{

    if(ois_master_info.cci_client == NULL){
        pr_err("msm_ois_lc898124_enable ois_master_info is NULL ,error return\n");
        return;
    }

    if(0 == ois_status){
        if(0 == enable){
            ois_disable = 1;
            pr_err("msm_ois_lc898124_enable ois_disable \n");
        }
        pr_err("msm_ois_lc898124_enable ois status = 0 return \n");
        return;
    }

    if (enable == 1)
    {
        RamWrite32A(0xF012, 0x00000001);
    }
    else
    {
        RamWrite32A(0xF012, 0x00000000);
    }
}


static int read_ois_status_task(void *data)
{
    uint32_t readData = 0;
    int cnt = 0;

    CDBG("read_ois_status_task E\n");
    do{
        RamRead32A(0xF100, &readData); 
        msleep(5);
    } while( ((readData & 0x1000000) != 0) && cnt++ < 20);
    pr_err("read0xF100 = %x  cnt=%d\n",readData,cnt);

    if(cnt >= 20){
        pr_err("read0xF100 = %x  error\n",readData);
        ois_status = 0;
    }else{
        ois_status = 1;
    }

    if(ois_disable){
        msm_ois_lc898124_enable(0);
    }else{
        msm_ois_lc898124_enable(1);
    }

    CDBG("read_ois_status_task X\n");
    return 0;
}

int32_t msm_ois_lc898124_init_AF(struct camera_io_master ois_master)
{
    int rc = 0;

    ois_status = 0;
    ois_disable = 0;
    
    CDBG("SID %x\n",ois_master.cci_client->sid);
    ois_master_info = ois_master;
    CDBG("msm_ois_lc898124_init_AF Enter\n");
    rc= EP2Download();
    CDBG("EP2Download rc = %d\n",rc);

    RemapMain();

    read_task = kthread_run(read_ois_status_task, NULL, "read_ois_status_task");
    if (IS_ERR(read_task)) {
        rc = PTR_ERR(read_task);
        pr_err("%s - read_task err: %d", __func__, rc);
    }

    if(rc != 0){
        pr_err("%s: EP2Download failed  line %d rc = %d\n", __func__, __LINE__, rc);
    }
    return rc;
}
////


