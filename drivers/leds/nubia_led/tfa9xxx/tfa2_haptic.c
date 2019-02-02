/*
 * tfa2_haptic.c - tfa9xxx tfa2 device driver haptic1 functions
 *
 * Copyright (C) 2018 NXP Semiconductors, All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include "tfa2_dev.h"
#include "tfa2_haptic.h"
#include "tfa2_container.h"
#include "tfa_haptic_fw_defs.h"

/* print effect object settings to string */
static int tfa2_haptic_current_effect(struct haptic_data *data, char *str, int len)
{
	if (!str)
		return -EINVAL;

	return snprintf(str, len,
			"index:%d, amplitude:%d, duration:%d, frequency:%d\n",
			data->index, data->amplitude, data->duration,
			data->frequency ? data->frequency : -1);
}

/******************************************************************************
 * haptic front-end functions
 */
/* start resonance */
int tfa2_haptic_start(struct i2c_client *client, struct haptic_data *data, int index)
{
	int rc;
	char str[256];
	struct haptic_tone_object *obj = (struct haptic_tone_object *)&data->object_table_cache[index][0];
	int address = FW_XMEM_GENOBJECTS + index * FW_XMEM_OBJECTSIZE;
	int object_changed = 0;
	int cmdobjsel;
#ifdef MEASURE_START_TIMING
	ktime_t start_time, stop_time;
	u64 delta_time;

	start_time = ktime_get_boottime();
#endif

	tfa2_haptic_current_effect(data, str, sizeof(str));
	dev_dbg(&client->dev, "started (%d) %s\n", index, str);

	/* Make sure the DSP is running! */
	rc = tfa9894_i2c_clock_stable_wait(client);
	if (rc < 0)
		return rc;

	if (obj->type == object_tone) {
		int level, samples;

		level = data->amplitude * 0x7fffff / 100; /* Q1.23, percentage of max */
		object_changed += (obj->level != level);
		obj->level = level;

		/* duration is in sample count : 48k is 48 samples/ms */
		samples = data->duration * 48; /* use DSP timer */
		object_changed += (obj->durationCntMax != samples);
		obj->durationCntMax = samples;

		if (data->frequency) {
			int freq = data->frequency << 11; /* Q13.11 */
			object_changed += (obj->freq != freq);
			obj->freq = freq;
		}
	}

	if (object_changed > 0) {
		/* write parameters */
		rc = tfa2_i2c_write_cf_mem32(client, address, (int32_t *)obj,
				FW_XMEM_OBJECTSIZE, TFA2_CF_MEM_XMEM);
		if (rc < 0)
			return rc;
	}

	/* tone to cmdObjSel0 and wave to cmdObjSel1 */
	if (obj->type == object_tone)
		cmdobjsel = FW_XMEM_CMDOBJSEL0;
	else
		cmdobjsel = FW_XMEM_CMDOBJSEL1;

	/* to start write cmdObjSel = index */
	rc = tfa2_i2c_write_cf_mem32(client, cmdobjsel, &index,
			1, TFA2_CF_MEM_XMEM);

#ifdef MEASURE_START_TIMING
	stop_time = ktime_get_boottime();
	delta_time = ktime_to_ns(ktime_sub(stop_time, start_time));
	do_div(delta_time, 1000);
	dev_dbg(&client->dev, "tfa_haptic_start duration = %lld us (%lld )\n",  delta_time, delta_time+900);
#endif
	return rc;
}

/* stop resonance */
int tfa2_haptic_stop(struct i2c_client *client, struct haptic_data *data, int index)
{
	struct haptic_tone_object *obj = (struct haptic_tone_object *)&data->object_table_cache[index][0];
	int stop_obj = FW_HB1_STOP_OBJ;
	int cmdobjsel;

	if (obj->type == object_tone)
		cmdobjsel = FW_XMEM_CMDOBJSEL0;
	else
		cmdobjsel = FW_XMEM_CMDOBJSEL1;

	return tfa2_i2c_write_cf_mem32(client, cmdobjsel, &stop_obj,
			1, TFA2_CF_MEM_XMEM);
}

/* duration in msecs of current object */
int tfa2_haptic_get_duration(struct haptic_data *data, int index)
{
	struct haptic_tone_object *obj = (struct haptic_tone_object *)&data->object_table_cache[index][0];
	/* duration is in sample count : 48k is 48 samples/ms */
	int duration = obj->durationCntMax / 48;

	/* take into account that duration might get adjusted in tfa_haptic_start() */
	if (obj->type == object_tone) {
		duration = data->duration;

		/* take boost brake length into account */
		if (obj->boostBrakeOn == 1)
			duration += (obj->boostLength / 48);
	}

	return duration + data->delay_attack; /* add object hold time */
}

enum tfa_haptic_object_type tfa2_haptic_object_type(struct haptic_data *data, int index)
{
	struct haptic_tone_object *obj  = (struct haptic_tone_object *)&data->object_table_cache[index][0];
	return obj->type ? object_tone : object_wave;
}

/*
 * extract the effect settings from the negative input value
 *   byte[2/3] is the frequency of object (if non-0)
 *   byte[1] is index of object
 *   byte[0] is the amplitude % from level
 *  return 0 if to be ignored for playback
 */
int tfa2_haptic_parse_value(struct haptic_data *data, int value)
{
	uint32_t xvalue;
	int level;

	if (value < 0) {
		xvalue = value * -1;
		data->index = (xvalue >> 8) & 0xff; /* get byte[1] */
		level  =  xvalue  & 0xff; /* get byte[0] */
		if (level < 4)
			data->amplitude = level * 33;
		else
			data->amplitude = level;
		data->frequency = xvalue >> 16; /* freq */
	}

	return (value > 0) ? 1 : 0;
}

int tfa2_haptic_read_r0(struct i2c_client *client, int *p_value)
{
	return tfa2_i2c_read_cf_mem32(client, FW_XMEM_R0, p_value, 1, TFA2_CF_MEM_XMEM);
}

int tfa2_haptic_read_f0(struct i2c_client *client, int *p_value)
{
	return tfa2_i2c_read_cf_mem32(client, FW_XMEM_F0, p_value, 1, TFA2_CF_MEM_XMEM);
}

int tfa2_haptic_read_sampcnt0(struct i2c_client *client, int *p_value)
{
	return tfa2_i2c_read_cf_mem32(client, FW_XMEM_SAMPCNT0, p_value, 1,TFA2_CF_MEM_XMEM);
}

int tfa2_haptic_disable_f0_trc(struct i2c_client *client, int disable)
{
	int disable_f0_trc = (disable != 0);
	return tfa2_i2c_write_cf_mem32(client, FW_XMEM_DISF0TRC, &disable_f0_trc, 1, TFA2_CF_MEM_XMEM);
}

int tfa2_haptic_obj0_wait_finish_www(struct i2c_client *client) //TODO fix the smart recalc waitms based on sample count
{
	int rc, sampcnt0, waitms;

	rc = tfa2_haptic_read_sampcnt0(client, &sampcnt0);
	if (rc < 0)
		return rc;

	/* use  sample count to predict run length */
	waitms = sampcnt0/48;
	msleep_interruptible(waitms + 50+50+100+500);

	/* obj0 should be done now */
	rc = tfa2_haptic_read_sampcnt0(client, &sampcnt0);
	if (rc < 0)
		return rc;
	if (sampcnt0 > 0)
		return -ETIMEDOUT;

	return 0;
}
int tfa2_haptic_obj0_wait_finish(struct i2c_client *client)
{
	int rc, loop = 50, ready = 0, sampcnt0;

	do {
		rc = tfa2_haptic_read_sampcnt0(client, &sampcnt0);
		if (rc < 0)
			return rc;
		ready = sampcnt0 <= 0;
		if  (ready == 1)
			break;
		else if (ready < 0)
			return -EIO;
		msleep_interruptible(50); /* wait to avoid busload *///TODO decrease time?...
	} while (loop--);

	if (sampcnt0 > 0)
		return -ETIMEDOUT;

	return 0;
}
int tfa2_haptic_calibrate_wait(struct i2c_client *client)
{
	int loop = 50, ready = 0, sampcnt0;
	int f0;

	do {
		tfa2_haptic_read_sampcnt0(client, &sampcnt0);
		tfa2_haptic_read_f0(client, &f0);
		ready = (sampcnt0 <= 0) && ( f0 != 0);
		if  (ready == 1)
			break;
		else if (ready < 0)
			return -EIO;
		msleep_interruptible(50); /* wait to avoid busload *///TODO decrease time?...
	} while (loop--);

	if (sampcnt0 > 0)
		return -ETIMEDOUT;

	return 0;
}

/* handy wrappers */
static int get_hap_profile(struct tfa2_device *tfa, char *string)
{
	// TODO limit to use .hap names only
	return tfa2_cnt_grep_profile_names(tfa->cnt, tfa->dev_idx, string);
}
static int tfa2_hap_patch_version(struct tfa2_device *tfa, char *string)
{
	int rc, fw_version;

	/* this is called after patch load, keep the dsp in reset */
	rc = tfa2_i2c_read_cf_mem32_dsp_reset(tfa->i2c, FW_XMEM_VERSION, &fw_version, 1,TFA2_CF_MEM_XMEM );
	dev_info(&tfa->i2c->dev, "%s patch version %d.%d.%d\n", string,
			(fw_version>>16)&0xff, (fw_version>>8)&0xff, fw_version&0xff);

	return rc;
}

/*
 * execute recalculation
 */
static int tfa2_hap_recalc(struct tfa2_device *tfa) {
	int rc, hbprofile, f0, f0mtp, fresout;

	hbprofile = get_hap_profile(tfa, "lra_recalculation.hap");
	rc = tfa2_cnt_write_files_profile(tfa, hbprofile, 0); /* write recalc patch*/
	if ( rc < 0)
		return rc;

	/* print version */
	tfa2_hap_patch_version(tfa, "recalculation");

	/* if F0  == 0 assume not calibrated */
	f0mtp = tfa2_dev_mtp_get(tfa, TFA_MTP_F0); /* raw */
	f0 = f0mtp/ 2 + 80; /* start at 80 */
	dev_dbg(&tfa->i2c->dev, "%s:F0 MTP:%d, F0:%d\n", __func__, f0mtp, f0);

	/* allowed range is between 80 and 336 Hz */
	if ( ( 80 < f0 ) && (  f0 < 336 ) ) {
		int trigger = 1; /* recalculation object = 1 */

		tfa2_i2c_write_bf(tfa->i2c, TFA9XXX_BF_RST, 0);
		/* Go to the Operating state */
		rc = tfa2_dev_set_state(tfa, TFA_STATE_POWERUP);
		if ( rc < 0 )
			return rc;

		msleep_interruptible(1);
		dev_dbg(&tfa->i2c->dev, "%s: manstate:%d\n" , __func__,
	                tfa2_i2c_read_bf(tfa->i2c, tfa->bf_manstate));

		/* trigger recalculation */
		tfa2_i2c_write_cf_mem32(tfa->i2c, FW_XMEM_CMDOBJSEL0, &trigger, 1, TFA2_CF_MEM_XMEM);

		/* wait for recalculation to finish */
		rc = tfa2_haptic_obj0_wait_finish(tfa->i2c);
		if (rc < 0) {
			dev_err(&tfa->i2c->dev, "Error, recalculation obj did not finish\n");
			return rc;
		}

		rc = tfa2_haptic_read_f0(tfa->i2c, &fresout);
		if (rc < 0) {
			dev_err(&tfa->i2c->dev, "Error reading f0\n");
			return rc;
		}
		dev_dbg(&tfa->i2c->dev, "recalculation %s\n",
		        (fresout == 0) ? "NOT done!" : "done");

		/* stop DSP */
		tfa2_i2c_write_bf(tfa->i2c, TFA9XXX_BF_RST, 1);
	} else {
		dev_info(&tfa->i2c->dev,
				"Warning: f0:%d not in 80/336 Hz range,  cannot recalculate\n", f0);
		/* no error return, in case of 1st time calibration it would fail */
	}

	return 0;
}

int tfa2_hap_load_data(struct tfa2_device *tfa) {
	int rc, hbprofile;
	char profile_name[32];

	strcpy(profile_name, (tfa->need_hb_config & TFA_HB_ROLE_MASK) == tfa_hb_lra ?"lra":"ls");
	strcat(profile_name, "_data.hap");

	hbprofile = get_hap_profile(tfa, profile_name); /* get "*_data.hap" */
	if (hbprofile < 0) {
		dev_err(&tfa->i2c->dev, "No [%s] profile found\n", profile_name);
		return -EINVAL;
	}
	rc = tfa2_cnt_write_files_profile(tfa, hbprofile, 0); /* write patch from profile*/
	if ( rc < 0)
		return rc;

	/* print version */
	return tfa2_hap_patch_version(tfa, "data");

}

static void tfa2_haptic_obj_get_defaults(struct haptic_data *data)
{
	struct haptic_tone_object *obj;
	/* default tone effect */
	data->index = 0;
	obj = (struct haptic_tone_object *)&data->object_table_cache[0][0];
	data->amplitude = 100 * obj->level / 0x7fffff;
	data->duration = obj->durationCntMax / 48;
}
/*
 * load and activate hapticboost
 *   if entered cold a full fw boot + lra recalculation  will be done
 */
int tfa2_dev_start_hapticboost(struct tfa2_device *tfa)
{
	int rc;
	char profile_name[32];

	strcpy(profile_name, (tfa->need_hb_config & TFA_HB_ROLE_MASK) ==tfa_hb_lra ?"lra":"ls");
	dev_dbg(&tfa->i2c->dev,  "%s: cold starting as %s\n",
	        __FUNCTION__, profile_name);

	/* cold start, assume RST=1 */
//	tfa2_i2c_write_bf(tfa->i2c, TFA9XXX_BF_RST, 1); //TODO optimize RST ,fix patches

	/* load data patch first */
	rc = tfa2_hap_load_data(tfa);
	if ( rc < 0) {
		dev_err(&tfa->i2c->dev, "Error loading data patch\n");
		return rc;
	}

	if ( (tfa->need_hb_config & TFA_HB_ROLE_MASK) == tfa_hb_lra ) {
		/* run recalculation */
		rc = tfa2_hap_recalc(tfa);
	}


	/* cache object table */
	rc = tfa2_i2c_read_cf_mem32(tfa->i2c, FW_XMEM_GENOBJECTS, (int *)tfa->hap_data.object_table_cache,
			FW_XMEM_OBJECTSIZE * FW_XMEM_NR_OBJECTS, TFA2_CF_MEM_XMEM);
	if (rc < 0) {
		dev_err(&tfa->i2c->dev, "Error caching object table\n");
		return rc;
	}
	/* cache delay_attack  */
	rc = tfa2_i2c_read_cf_mem32(tfa->i2c, FW_XMEM_DELAY_ATTACK_SMP, &tfa->hap_data.delay_attack,
			1 , TFA2_CF_MEM_XMEM);
	tfa->hap_data.delay_attack /= 48; /* make it milliseconds */
	if (rc < 0) {
		dev_err(&tfa->i2c->dev, "Error caching delay_attack\n");
		return rc;
	}

	/* get the defaults */
	tfa2_haptic_obj_get_defaults(&tfa->hap_data);

	return rc;
}

int tfa2_hap_calibrate(struct tfa2_device *tfa)
{
	int cal_obj = FW_HB1_CAL_OBJ;
	int f0, r0;
	uint16_t mtp_f0;
	int rc, ret, current_profile;
	int range[4];

	/* Update  init state */
	tfa2_dev_update_config_init(tfa);

	if (  !tfa->need_hb_config   ) {
		dev_err(&tfa->i2c->dev, "HB calibration not done, no config found in container\n");
		return -EINVAL;
	}

	mtp_f0 = tfa2_dev_mtp_get(tfa, TFA_MTP_F0);
	if (mtp_f0 < 0) {
		dev_dbg(&tfa->i2c->dev, "Error reading MTP F0\n");
		return mtp_f0;
	} else {
		dev_dbg(&tfa->i2c->dev, "Current MTP F0:%d\n", mtp_f0);
	}

	/* if not 0, wipe it */
	if ( mtp_f0 ) {
		/* Go to the powerup state to allow MTP writing*/
		rc = tfa2_dev_set_state(tfa, TFA_STATE_POWERUP);
		if ( rc < 0 )
			return rc;

		/* Clear f0 in MTP */
		rc = tfa2_dev_mtp_set(tfa, TFA_MTP_F0, 0);
		if (rc < 0) {
			dev_err(&tfa->i2c->dev, "Error clearing F0 in MTP\n");
			return rc;
		}
		/* turn off again, so we are always enter with pll off */
		rc = tfa2_dev_set_state(tfa, TFA_STATE_POWERDOWN);
		if ( rc < 0 )
			return rc;
	}

	current_profile = tfa2_dev_get_swprofile(tfa);
	if ( current_profile < 0 ) {
		/* no current profile, take the 1st profile */
		current_profile = 0;
	}

	/* cold run of calibration profile */
	rc = tfa2_calibrate_profile_start(tfa);
	if (rc < 0) {
		dev_err(&tfa->i2c->dev, "Error, calibration profile start\n");
		return rc;
	}

	/* Trigger calibration */
	tfa2_i2c_write_cf_mem32(tfa->i2c, FW_XMEM_CMDOBJSEL0, &cal_obj, 1, TFA2_CF_MEM_XMEM);

	msleep_interruptible(1000 + tfa->hap_data.delay_attack ); //TODO determine optimal delay

	/* wait for calibration to finish */
	rc = tfa2_haptic_obj0_wait_finish(tfa->i2c);
	if (rc < 0) {
		dev_err(&tfa->i2c->dev, "Error, calibration did not finish\n");
		goto reload_profile;
	}

	rc = tfa2_haptic_read_f0(tfa->i2c, &f0);
	if (rc < 0) {
		dev_err(&tfa->i2c->dev, "Error reading f0\n");
		goto reload_profile;
	}
	dev_dbg(&tfa->i2c->dev, "F0 = %d.%03d Hz (0x%x)\n",
	         TFA2_HAPTIC_FP_INT(f0, FW_XMEM_F0_SHIFT),
	         TFA2_HAPTIC_FP_FRAC(f0, FW_XMEM_F0_SHIFT),
	          f0);

	rc = tfa2_haptic_read_r0(tfa->i2c, &r0);
	if (rc < 0) {
		dev_err(&tfa->i2c->dev, "Error reading r0\n");
		goto reload_profile;
	}
	dev_dbg(&tfa->i2c->dev, "R0 = %d.%03d ohm (0x%x)\n",
	         TFA2_HAPTIC_FP_INT(r0, FW_XMEM_R0_SHIFT),
	         TFA2_HAPTIC_FP_FRAC(r0, FW_XMEM_R0_SHIFT),
	          r0);

	/* Check F0/R0 ranges */
	rc = tfa2_i2c_read_cf_mem32(tfa->i2c, FW_XMEM_F0_R0_RANGES, range, 4, TFA2_CF_MEM_XMEM);
	if (rc < 0) {
		dev_err(&tfa->i2c->dev, "Error reading F0/R0 ranges\n");
		goto reload_profile;
	}

	rc = 0;
	if ((f0 < range[3]) || (f0 > range[2])) {
		dev_err(&tfa->i2c->dev, "F0 out of range: 0x%06x < 0x%06x < 0x%06x\n",
		         range[3], f0, range[2]);
		rc = -ERANGE;
	}
	if ((r0 < range[1]) || (r0 > range[0])) {
		dev_err(&tfa->i2c->dev, "R0 out of range: 0x%06x < 0x%06x < 0x%06x\n",
		         range[1], r0, range[0]);
		rc = -ERANGE;
	}
	if (rc != 0) {
		dev_err(&tfa->i2c->dev, "range check failed, result not written\n");
		goto reload_profile;
	}

	/* 16 bit f0 value to store in F0  MTP location
	 *  Encoding:
	 *              MTPValue = round ((  f0 -  80 Hz ) * 2 )
	 *              f0 [in Hz]  = MTPValue / 2 + 80 Hz
	 * */

	/* convert F0 to Hz */
	f0 = TFA2_HAPTIC_FP_INT(f0, FW_XMEM_F0_SHIFT) +
			(TFA2_HAPTIC_FP_FRAC(f0, FW_XMEM_F0_SHIFT)>499 ? 1 : 0);
	mtp_f0 = ( f0 - 80 ) * 2 ;
	dev_dbg(&tfa->i2c->dev, "MTPF0 = %d (0x%x)\n", mtp_f0/2 + 80, mtp_f0);

// TODO check F0 for non-zero; always make no sense if the FW uses MTP

		/* Store f0 in MTP */
	rc = tfa2_dev_mtp_set(tfa, TFA_MTP_F0, mtp_f0);
	if (rc < 0) {
		dev_err(&tfa->i2c->dev, "Error writing F0 to MTP\n");
		goto reload_profile;
	}

		/* TODO if calibrate always: Store f0 in shadow register */
	//	tfa2_i2c_write_bf(tfa->i2c, TFA9XXX_BF_CUSTINFO, mtp_f0);

reload_profile:
	/* force a cold start to do recalculation */
	ret = tfa2_dev_force_cold(tfa);
	if (ret <0) {
		dev_err(&tfa->i2c->dev, "%s: force cold failed\n", __func__);
		return ret;
	}

	/* re-load current profile  */
	ret = tfa2_dev_start(tfa, current_profile, 0);
	if (ret < 0) {
		dev_err(&tfa->i2c->dev, "%s: Cannot reload profile %d\n",
		        __func__, current_profile);
		return ret;
	}

	return rc;
}


