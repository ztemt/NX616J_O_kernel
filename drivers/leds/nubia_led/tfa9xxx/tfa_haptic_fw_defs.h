/*
 * tfa_haptic_fw_defs.h  - tfa9914 haptic defines
 *
 * Copyright (C) 2018 NXP Semiconductors, All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#ifndef TFA_HAPTIC_FW_DEFS_H_
#define TFA_HAPTIC_FW_DEFS_H_

/* base version */
#define FW_VERSION           0x0a0600  /* patch version in hex */
#define FW_XMEM_VERSION      0x11ff    /* TFA9894 xmem version offset */

/* firmware constants */
#define FW_XMEM_NR_OBJECTS 6
#define FW_XMEM_OBJECTSIZE 8
#define FW_XMEM_OBJECTSTATESIZE 8

#define FW_XMEM_CMDOBJSEL0      4102  /* pObjState */
#define FW_XMEM_CMDOBJSEL1      (FW_XMEM_CMDOBJSEL0 + FW_XMEM_OBJECTSTATESIZE) /*  */
#define FW_XMEM_SAMPCNT0        (FW_XMEM_CMDOBJSEL0 + 2) /* ->timeCnt */
#define FW_XMEM_SAMPCNT1        (FW_XMEM_CMDOBJSEL1 + 2)
#define FW_XMEM_F0_R0_RANGES     4134  /* ZfresMax */
#define FW_XMEM_DISF0TRC         4359  /* disF0Trc */
#define FW_XMEM_R0               4053  /* Rf0Out */
#define FW_XMEM_F0               4052  /* fResOut */
#define FW_XMEM_DELAY_ATTACK_SMP 4361  /* duckLraPar->delayAttackSmp */
#define FW_XMEM_GENOBJECTS       4054  /* objectArray */

#define FW_HB1_CAL_OBJ  (FW_XMEM_NR_OBJECTS - 2)  /* calibration object */
#define FW_HB1_STOP_OBJ (FW_XMEM_NR_OBJECTS - 1)  /* silence object */

#define FW_XMEM_R0_SHIFT 11 /* Q13.11 */
#define FW_XMEM_F0_SHIFT 11 /* Q13.11 */

enum tfa_haptic_object_type {
	object_wave = 0,
	object_tone = 1,
	object_silence = 2
};

/* Tone Generator object definition */
struct haptic_tone_object {
       int32_t type;
       int32_t freq;
       int32_t level;
       int32_t durationCntMax;
       int32_t boostBrakeOn;
       int32_t trackerOn;
       int32_t boostLength;
       int32_t reserved;
} ;

/* Wave table object definition */
struct haptic_wave_object {
       int32_t type;
       int32_t offset;
       int32_t level;
       int32_t durationCntMax;
       int32_t upSampSel;
       int32_t reserved[3];
} ;

#endif /* TFA_HAPTIC_FW_DEFS_H_ */
