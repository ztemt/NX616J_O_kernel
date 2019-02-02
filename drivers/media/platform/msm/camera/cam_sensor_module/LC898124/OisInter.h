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
#ifndef MSM_OIS_H
#define MSM_OIS_H





void CntWrt( void *	PcSetDat, unsigned short UsDatNum );

void	WitTim( unsigned short	UsWitTim );

void RamRead32A( unsigned short addr, void * data);
void RamWrite32A(unsigned int addr, unsigned int data);
int32_t msm_ois_lc898124_init_AF(struct camera_io_master ois_master);
void msm_ois_lc898124_enable(int enable);
void msm_ois_lc898124_write_dac(unsigned int data);

#endif

