/*
 * Copyright (C) 2018 NXP Semiconductors, All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include "tfa9xxx.h"
#include "tfa2_dev.h"
#include "tfa2_haptic.h"

#define TFA_HAP_MAX_TIMEOUT (0x7fffff)
#define DEFAULT_PLL_OFF_DELAY 100 /* msec */

static const char *haptic_feature_id_str[] = {"wave", "sine", "silence","illegal"};

static const char *haptic_feature_id(int id)
{
	if (id > 3)
		id=3;
	return haptic_feature_id_str[id];
}

static void haptic_dump_obj(struct seq_file *f, struct haptic_tone_object *o)
{
	switch(o->type) {
	case object_wave:
		seq_printf(f, "\t type: %s\n"
		              "\t offset: %d\n"
		              "\t level: %d%%\n"
		              "\t durationCntMax: %d (%dms)\n"
		              "\t upSampSel: %d\n",
		            haptic_feature_id(o->type),
		             o->freq,
		             (100*o->level)/0x7fffff,  /* Q1.23, percentage of max */
		             o->durationCntMax,o->durationCntMax/48,  /* sapmples in time : 1/FS ms */
		             o->boostBrakeOn);
		break;
	case object_tone:
		seq_printf(f, "\t type: %s\n"
		              "\t freq: %dHz\n"
		              "\t level: %d%%\n"
		              "\t durationCntMax: %dms\n"
		              "\t boostBrakeOn: %d\n"
		              "\t trackerOn: %d\n"
		              "\t boostLength: %d\n",
		            haptic_feature_id(o->type),
		            o->freq>>11, /* Q13.11 */
		            (100*o->level)/0x7fffff,  /* Q1.23, percentage of max */
		            o->durationCntMax/48,  /* sapmples in time : 1/FS ms */
		            o->boostBrakeOn,
		            o->trackerOn,
		            o->boostLength);
		break;
	case object_silence:
		seq_printf(f, "\t type: %s\n"
		              "\t durationCntMax: %dms\n",
		            haptic_feature_id(o->type),
		             o->durationCntMax/48 /* sapmples in time : 1/FS ms */
			);
		break;
	default:
		seq_printf(f, "wrong feature id in object!\n");
		break;
	}
}

int tfa9xxx_haptic_dump_objs(struct seq_file *f, void *p)
{
	struct tfa9xxx *drv = f->private;
	int i;

	for (i=0; i<FW_XMEM_NR_OBJECTS; i++) {
		struct haptic_tone_object *o = (struct haptic_tone_object *)&drv->tfa->hap_data.object_table_cache[i][0];

		if (o->durationCntMax==0) /*  assume object is empty */
			continue;

		seq_printf(f, "object[%d]---------\n", i);
		haptic_dump_obj(f, o);
	}

	seq_printf(f, "\ndelay_attack: %dms\n", drv->tfa->hap_data.delay_attack);

	return 0;
}

static enum hrtimer_restart tfa_haptic_timer_func(struct hrtimer *timer)
{
	struct drv_object *obj = container_of(timer, struct drv_object,
					      active_timer);
	struct tfa9xxx *drv;

#ifdef PARALLEL_OBJECTS
	if (obj->type == object_tone)
		drv = container_of(obj, struct tfa9xxx, tone_object);
	else
		drv = container_of(obj, struct tfa9xxx, wave_object);
#else
	drv = container_of(obj, struct tfa9xxx, tone_object);
#endif

	dev_info(&drv->i2c->dev, "%s: type=%d\n", __func__, (int)obj->type);
	obj->state = STATE_STOP;
	schedule_work(&obj->update_work);

	return HRTIMER_NORESTART;
}

static void tfa_haptic_pll_off(struct work_struct *work)
{
	struct tfa9xxx *drv = container_of(work, struct tfa9xxx, pll_off_work.work);

	dev_info(&drv->i2c->dev,"%s\n", __func__);

	/* turn off PLL */
	tfa2_dev_stop(drv->tfa);
}

static void tfa9xxx_haptic_clock(struct tfa9xxx *drv, bool on)
{
	dev_dbg(&drv->i2c->dev,"%s: on=%d, clk_users=%d\n", __func__,(int)on, drv->clk_users);

	/* cancel delayed turning off of the PLL */
	if (delayed_work_pending(&drv->pll_off_work))
		cancel_delayed_work_sync(&drv->pll_off_work);

	if (on) {
		if (drv->clk_users == 0)
			/* turn on PLL */
			tfa2_dev_set_state(drv->tfa, TFA_STATE_POWERUP);
		drv->clk_users++;
	} else if (drv->clk_users > 0) {
		drv->clk_users--;
		if (drv->clk_users == 0)
			/* turn off PLL with a delay */
			schedule_delayed_work(&drv->pll_off_work,
			                  msecs_to_jiffies(drv->pll_off_delay));
	}
}

int tfa9xxx_disable_f0_tracking(struct tfa9xxx *drv, bool disable)
{
	int ret = 0;

	dev_dbg(&drv->i2c->dev, "%s: disable=%d, f0_trc_users=%d\n", __func__, (int)disable, drv->f0_trc_users);

	if (disable) {
		if (drv->f0_trc_users == 0)
			/* disable*/
			ret = tfa2_haptic_disable_f0_trc(drv->i2c, 1);
		drv->f0_trc_users++;
	} else if (drv->f0_trc_users > 0) {
		drv->f0_trc_users--;
		if (drv->f0_trc_users == 0)
			/* enable again */
			ret = tfa2_haptic_disable_f0_trc(drv->i2c, 0);
	}

	return ret;
}

static void tf9xxx_update_led_class(struct work_struct *work)
{
	struct drv_object *obj = container_of(work, struct drv_object, update_work);
	struct tfa9xxx *drv;
	struct haptic_data *data;

#ifdef PARALLEL_OBJECTS
	if (obj->type == object_tone)
		drv = container_of(obj, struct tfa9xxx, tone_object);
	else
		drv = container_of(obj, struct tfa9xxx, wave_object);
#else
	drv = container_of(obj, struct tfa9xxx, tone_object);
#endif

	/* should have been checked before scheduling this work */
	BUG_ON(!drv->patch_loaded);

	data = &drv->tfa->hap_data;

#ifdef NUBIA_HAPTIC_TFA9XXX
	dev_info(&drv->i2c->dev,"%s: type: %d, index: %d, state: %d, duration = %d\n",
			__func__,	(int)obj->type, obj->index,
			obj->state, data->duration);
#else
	dev_info(&drv->i2c->dev,"%s: type: %d, index: %d, state: %d\n", __func__, (int)obj->type, obj->index, obj->state);
#endif

	if (obj->state != STATE_STOP) {
		if (obj->state == STATE_RESTART)
			tfa2_haptic_stop(drv->i2c, data, obj->index);
		else
			tfa9xxx_haptic_clock(drv, true);

		if (tfa2_haptic_start(drv->i2c, data, obj->index)) {
			dev_err(&drv->i2c->dev, "%s: problem when starting\n",
			        __func__);
		}
	} else {
		tfa2_haptic_stop(drv->i2c, data, obj->index);
		tfa9xxx_haptic_clock(drv, false);
	 }
}

static void tfa9xxx_led_set(struct led_classdev *cdev,
                               enum led_brightness brightness)
{
	struct tfa9xxx *drv = container_of(cdev, struct tfa9xxx, led_dev);
	struct tfa2_device *tfa = drv->tfa;

	dev_dbg(&drv->i2c->dev,"%s: brightness = %d\n", __func__, brightness);

	/* set amplitude scaled to 0 - 100% */
	tfa->hap_data.amplitude = 100 * brightness / LED_FULL;
}

static enum led_brightness tfa9xxx_led_get(struct led_classdev *cdev)
{
	struct tfa9xxx *drv = container_of(cdev, struct tfa9xxx, led_dev);
	struct tfa2_device *tfa = drv->tfa;

	return LED_FULL * tfa->hap_data.amplitude / 100;
}

static ssize_t tfa9xxx_store_value(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct tfa9xxx *drv = container_of(cdev, struct tfa9xxx, led_dev);
	struct tfa2_device *tfa = drv->tfa;
	struct haptic_data *data = &tfa->hap_data;
	long value = 0;
	int ret;

	ret = kstrtol(buf, 10, &value);
	if (ret)
		return ret;

	ret = tfa2_haptic_parse_value(data, value);
	if (ret)
		return -EINVAL;

	dev_info(&drv->i2c->dev, "%s: index=%d, amplitude=%d, frequency=%d\n",
	        __func__, data->index, data->amplitude, data->frequency);

	return count;
}

static ssize_t tfa9xxx_show_value(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct tfa9xxx *drv = container_of(cdev, struct tfa9xxx, led_dev);
	struct tfa2_device *tfa = drv->tfa;

	return sprintf(buf, "index=%d, amplitude=%d, frequency=%d, duration=%d\n",
	               tfa->hap_data.index, tfa->hap_data.amplitude,
	               tfa->hap_data.frequency, tfa->hap_data.duration);
}

static ssize_t tfa9xxx_store_state(struct device *dev,
                                       struct device_attribute *attr,
                                       const char *buf, size_t count)
{
	return count;
}


static ssize_t tfa9xxx_show_state(struct device *dev,
                                       struct device_attribute *attr, char *buf)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct tfa9xxx *drv = container_of(cdev, struct tfa9xxx, led_dev);

#ifdef PARALLEL_OBJECTS
	int state = (drv->tone_object.state != STATE_STOP) ||
	            (drv->wave_object.state != STATE_STOP);
#else
	int state = (drv->tone_object.state != STATE_STOP);
#endif
	return sprintf(buf, "%d\n", state);
}

static ssize_t tfa9xxx_store_activate(struct device *dev,
                                          struct device_attribute *attr,
                                          const char *buf, size_t count)
{
	struct led_classdev *cdev =  dev_get_drvdata(dev);
	struct tfa9xxx *drv = container_of(cdev, struct tfa9xxx, led_dev);
	struct tfa2_device *tfa = drv->tfa;
	struct drv_object *obj;
	u32 val;
	int rc;
	int state;
#ifdef PARALLEL_OBJECTS
	bool object_is_tone;
#endif

	if (!drv->patch_loaded) {
		dev_err(&drv->i2c->dev, "%s: patch not loaded\n", __func__);
		return -EINVAL;
	}

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	state = (val != 0);

	dev_dbg(&drv->i2c->dev, "%s: update_object_index=%d, val=%d\n",
	        __func__, (int)drv->update_object_index, val);

#ifdef PARALLEL_OBJECTS
	object_is_tone = (tfa2_haptic_object_type(&tfa->hap_data, drv->object_index) == object_tone);
	if (object_is_tone)
		obj = &drv->tone_object;
	else
		obj = &drv->wave_object;
#else
	obj = &drv->tone_object;
#endif

	if (drv->update_object_index == true) {
		drv->update_object_index = false;
		obj->index = drv->object_index;
		return count;
	}

	/* when no bck and no mclk, we can not play vibrations */
	if (!drv->bck_running && !drv->mclk) {
		dev_warn(&drv->i2c->dev, "%s: no active clock\n", __func__);
		return -ENODEV;
	}

	if (state == 0) {
		if (obj->state == STATE_STOP)
			return count;
		obj->state = STATE_STOP;
	} else {
		if (obj->state == STATE_STOP)
			obj->state = STATE_START;
		else
			obj->state = STATE_RESTART;
	}

	hrtimer_cancel(&obj->active_timer);

	if (obj->state != STATE_STOP) {
		/* object duration + 5 msecs */
		int value = tfa2_haptic_get_duration(&tfa->hap_data, obj->index) + 5;
		/* clip value to max */
		value = (value > drv->timeout ?  drv->timeout : value);
		/* run ms timer */
		hrtimer_start(&obj->active_timer,
	              ktime_set(value / 1000,
	              (value % 1000) * 1000000),
	              HRTIMER_MODE_REL);

		dev_dbg(&drv->i2c->dev, "%s: start active %d timer of %d msecs\n",
		        __func__, (int)obj->type, value);
	}

	schedule_work(&obj->update_work);

	return count;
}

int tfa9xxx_bck_starts(struct tfa9xxx *drv)
{
	dev_dbg(&drv->i2c->dev, "%s: bck_running=%d, mclk=%p\n", __func__,
	        (int)drv->bck_running, drv->mclk);

	if (drv->bck_running)
		return 0;

	/* clock enabling is handled in tfa_start(), so update reference count
	 instead of calling tfa9xxx_haptic_clock(drv, true) */
	drv->clk_users++;

	drv->bck_running = true;

	return 0;
}


int tfa9xxx_bck_stops(struct tfa9xxx *drv)
{
	struct drv_object *obj;

	dev_dbg(&drv->i2c->dev, "%s: bck_running=%d, mclk=%p\n", __func__,
	        (int)drv->bck_running, drv->mclk);

	if (!drv->bck_running)
		return 0;

	drv->bck_running = false;

	/* clock disabling is handled in tfa_stop(), so update reference count
	 instead of calling tfa9xxx_haptic_clock(drv, false) */
	drv->clk_users--;

	if (drv->mclk)
		return 0;

	/* When there is no mclk, stop running objects */
#ifdef PARALLEL_OBJECTS
	obj = &drv->wave_object;
	if (obj->state != STATE_STOP) {
		obj->state = STATE_STOP;
		hrtimer_cancel(&obj->active_timer);
		schedule_work(&obj->update_work);
		flush_work(&obj->update_work);
	}
#endif
	obj = &drv->tone_object;
	if (obj->state != STATE_STOP) {
		obj->state = STATE_STOP;
		hrtimer_cancel(&obj->active_timer);
		schedule_work(&obj->update_work);
		flush_work(&obj->update_work);
	}

	return 0;
}


static ssize_t tfa9xxx_show_activate(struct device *dev,
                                        struct device_attribute *attr,
                                        char *buf)
{
	struct led_classdev *cdev =  dev_get_drvdata(dev);
	struct tfa9xxx *drv = container_of(cdev, struct tfa9xxx, led_dev);

#ifdef PARALLEL_OBJECTS
	int state = (drv->tone_object.state != STATE_STOP) ||
	            (drv->wave_object.state != STATE_STOP);
#else
	int state = (drv->tone_object.state != STATE_STOP);
#endif
	return sprintf(buf, "%d\n", state);
}

static ssize_t tfa9xxx_store_duration(struct device *dev,
                                         struct device_attribute *attr,
                                         const char *buf, size_t count)
{
	int val;
	int rc;
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct tfa9xxx *drv = container_of(cdev, struct tfa9xxx, led_dev);
	struct tfa2_device *tfa = drv->tfa;

	rc = kstrtoint(buf, 0, &val);
	if (rc < 0)
		return rc;

	dev_dbg(&drv->i2c->dev, "%s: val=%d\n", __func__, val);

	drv->update_object_index = false;

#ifdef NUBIA_HAPTIC_TFA9XXX
	if (val > 0) {
		dev_dbg(&drv->i2c->dev, "%s: duration = %d\n", __func__, val);
		tfa->hap_data.duration = val;
	}
#else
	if (val < 0) {
		rc = tfa9xxx_store_value(dev, attr, buf, count);
		if (rc < 0)
			return rc;
	} else if ((val > 0) && (val <= 10)) {
		/*
		   If the value is less or equal than 10, we assume it is
		   object index from App layer. Note that Android is not able
		   too pass a 0 value, so the object index has an offset of 1
		*/
		drv->object_index = val - 1;
		drv->update_object_index = true;
		tfa->hap_data.index = drv->object_index;
		dev_dbg(&drv->i2c->dev, "%s: new object index is %d\n",
		        __func__, drv->object_index);
	} else if (val > 10) {
		dev_dbg(&drv->i2c->dev, "%s: duration = %d\n", __func__, val);
		tfa->hap_data.duration = val;
	}
#endif

	return count;
}

static ssize_t tfa9xxx_show_duration(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return tfa9xxx_show_value(dev, attr, buf);
}

static ssize_t tfa9xxx_show_f0(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct tfa9xxx *drv = dev_get_drvdata(dev);
	int ret, f0;

	ret = tfa2_haptic_read_f0(drv->i2c, &f0);
	if (ret)
		return -EIO;

	return sprintf(buf, "%d.%03d\n",
	               TFA2_HAPTIC_FP_INT(f0, FW_XMEM_F0_SHIFT),
	               TFA2_HAPTIC_FP_FRAC(f0, FW_XMEM_F0_SHIFT));
}

static ssize_t tfa9xxx_store_pll_off_delay(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct tfa9xxx *drv = dev_get_drvdata(dev);
	int err;

	err = kstrtouint(buf, 10, &drv->pll_off_delay);
	if (err)
		return err;

	return count;
}

static ssize_t tfa9xxx_show_pll_off_delay(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct tfa9xxx *drv = dev_get_drvdata(dev);
	return sprintf(buf, "%u\n", drv->pll_off_delay);
}

static ssize_t tfa9xxx_store_haptic_mode(struct device *dev,
                                         struct device_attribute *attr,
                                         const char *buf, size_t count)
{

	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct tfa9xxx *drv = container_of(cdev, struct tfa9xxx, led_dev);

	u32 val;
	int rc;

	rc = kstrtoint(buf, 0, &val);
	if (rc < 0)
		return rc;

	dev_info(&drv->i2c->dev, "%s: val=%d\n", __func__, val);

	drv->object_index = val;

	return count;
}

static ssize_t tfa9xxx_show_haptic_mode(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct tfa9xxx *drv = container_of(cdev, struct tfa9xxx, led_dev);

	return sprintf(buf, "%d\n", drv->object_index);
}


static struct device_attribute tfa9xxx_haptic_attrs[] = {
	__ATTR(value, 0664, tfa9xxx_show_value, tfa9xxx_store_value),
	__ATTR(state, 0664, tfa9xxx_show_state, tfa9xxx_store_state),
	__ATTR(activate, 0664, tfa9xxx_show_activate, tfa9xxx_store_activate),
	__ATTR(duration, 0664, tfa9xxx_show_duration, tfa9xxx_store_duration),
	__ATTR(haptic_mode, 0664, tfa9xxx_show_haptic_mode, tfa9xxx_store_haptic_mode),
};

static DEVICE_ATTR(f0, 0444, tfa9xxx_show_f0, NULL);
static DEVICE_ATTR(pll_off_delay, 0644, tfa9xxx_show_pll_off_delay,
                                        tfa9xxx_store_pll_off_delay);


int tfa9xxx_haptic_probe(struct tfa9xxx *drv)
{
	struct i2c_client *i2c = drv->i2c;
	int failed_attr_idx = 0;
	int rc, i;

	drv->timeout = TFA_HAP_MAX_TIMEOUT;
	drv->pll_off_delay = DEFAULT_PLL_OFF_DELAY;

	INIT_WORK(&drv->tone_object.update_work, tf9xxx_update_led_class);
#ifdef PARALLEL_OBJECTS
	INIT_WORK(&drv->wave_object.update_work, tf9xxx_update_led_class);
#endif
	INIT_DELAYED_WORK(&drv->pll_off_work, tfa_haptic_pll_off);

	drv->tone_object.type = object_tone;
#ifdef PARALLEL_OBJECTS
	drv->wave_object.type = object_wave;
#endif
	/* add "f0" entry to i2c device */
	rc = device_create_file(&i2c->dev, &dev_attr_f0);
	if (rc < 0) {
		dev_err(&drv->i2c->dev, "%s: Error, creating sysfs f0 entry\n", __func__);
		goto error_sysfs_f0;
	}

	/* add "pll_off_delay" entry to i2c device */
	rc = device_create_file(&i2c->dev, &dev_attr_pll_off_delay);
	if (rc < 0) {
		dev_err(&drv->i2c->dev, "%s: Error, creating sysfs pll_off_delay entry\n", __func__);
		goto error_sysfs_pll;
	}

	/* active timers for led driver */
	hrtimer_init(&drv->tone_object.active_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	drv->tone_object.active_timer.function = tfa_haptic_timer_func;

#ifdef PARALLEL_OBJECTS
	hrtimer_init(&drv->wave_object.active_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	drv->wave_object.active_timer.function = tfa_haptic_timer_func;
#endif

	/* led driver */
#ifdef NUBIA_HAPTIC_TFA9XXX
	drv->led_dev.name = "tfa9xxx";
#else
	drv->led_dev.name = "vibrator";
#endif
	drv->led_dev.brightness_set = tfa9xxx_led_set;
	drv->led_dev.brightness_get = tfa9xxx_led_get;
	drv->led_dev.default_trigger = "none";

	rc = led_classdev_register(&i2c->dev, &drv->led_dev);
	if (rc < 0) {
		dev_err(&drv->i2c->dev, "%s: Error, led device\n", __func__);
		goto error_led_class;
	}

	for (i=0; i < ARRAY_SIZE(tfa9xxx_haptic_attrs); i++) {
		rc = sysfs_create_file(&drv->led_dev.dev->kobj,
		                       &tfa9xxx_haptic_attrs[i].attr);
		if (rc < 0) {
			dev_err(&drv->i2c->dev, "%s: Error, creating sysfs entry %d rc=%d\n",
			        __func__, i, rc);
			failed_attr_idx = i;
			goto error_sysfs_led;
		}
	}

	return 0;

error_sysfs_led:
	for (i=0; i<failed_attr_idx; i++)
		sysfs_remove_file(&drv->led_dev.dev->kobj,
		                  &tfa9xxx_haptic_attrs[i].attr);
	led_classdev_unregister(&drv->led_dev);

error_led_class:
	device_remove_file(&i2c->dev, &dev_attr_pll_off_delay);

error_sysfs_pll:
	device_remove_file(&i2c->dev, &dev_attr_f0);

error_sysfs_f0:

	return rc;
}

int tfa9xxx_haptic_remove(struct tfa9xxx *drv)
{
	struct i2c_client *i2c = drv->i2c;
	int i;

	hrtimer_cancel(&drv->tone_object.active_timer);
#ifdef PARALLEL_OBJECTS
	hrtimer_cancel(&drv->wave_object.active_timer);
#endif

	device_remove_file(&i2c->dev, &dev_attr_f0);
	device_remove_file(&i2c->dev, &dev_attr_pll_off_delay);

	for (i=0; i < ARRAY_SIZE(tfa9xxx_haptic_attrs); i++) {
		sysfs_remove_file(&drv->led_dev.dev->kobj,
		                  &tfa9xxx_haptic_attrs[i].attr);

	}

	led_classdev_unregister(&drv->led_dev);

	cancel_work_sync(&drv->tone_object.update_work);
#ifdef PARALLEL_OBJECTS
	cancel_work_sync(&drv->wave_object.update_work);
#endif
	cancel_delayed_work_sync(&drv->pll_off_work);

	return 0;
}

int __init tfa9xxx_haptic_init(void)
{
	return 0;
}

void __exit tfa9xxx_haptic_exit(void)
{

}
