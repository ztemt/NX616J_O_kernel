/*
 * nubia_disp_preference.c - nubia lcd display color enhancement and temperature setting
 *	      Linux kernel modules for mdss
 *
 * Copyright (c) 2015 nubia <nubia@nubia.com.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

/*
 * Supports NUBIA lcd display color enhancement and color temperature setting
 */

/*------------------------------ header file --------------------------------*/
#include "nubia_disp_preference.h"
#include <linux/delay.h>
#ifdef CONFIG_NUBIA_SWITCH_LCD
extern int goodix_ts_exist_register_notifier(struct notifier_block *nb);
extern int goodix_ts_exist_unregister_notifier(struct notifier_block *nb);
#endif

static struct dsi_display *nubia_display=NULL;
/*------------------------------- variables ---------------------------------*/
static struct kobject *enhance_kobj = NULL;
#ifdef CONFIG_NUBIA_SWITCH_LCD
extern unsigned int lcd_states;
#endif
struct nubia_disp_type nubia_disp_val = {
	.en_cabc = 1,
	.cabc = CABC_OFF,
	.en_saturation = 1,
	.saturation = SATURATION_SOFT,
	.en_colortmp = 1,
	.colortmp =  COLORTMP_NORMAL,
#ifdef CONFIG_NUBIA_SWITCH_LCD
	.en_rgbw_mode=1,
	.rgbw_mode=RGBW_NORMAL,
	.en_aod_mode=1,
	.aod_mode=AOD_OFF,
	.en_lcd_state =1,
	.lcd_state = 0,
#endif
};

static ssize_t cabc_show(struct kobject *kobj,
        struct kobj_attribute *attr, char *buf)
{
       if (nubia_disp_val.en_cabc)
                return snprintf(buf, PAGE_SIZE, "%d\n", nubia_disp_val.cabc);
        else
                return snprintf(buf, PAGE_SIZE, "NULL\n");

}

static ssize_t cabc_store(struct kobject *kobj,
        struct kobj_attribute *attr, const char *buf, size_t size)
{
        uint32_t val = 0;
        int ret = 0;
	if(!nubia_disp_val.en_cabc) {
                NUBIA_DISP_ERROR("no cabc\n");
                return size;
        }

        sscanf(buf, "%d", &val);

        if ((val != CABC_OFF) && (val != CABC_LEVER1) &&
                (val != CABC_LEVER2) && (val != CABC_LEVER3)) {
                NUBIA_DISP_ERROR("invalid cabc val = %d\n", val);
                return size;
        }

        NUBIA_DISP_INFO("cabc value = %d\n", val);

	if(nubia_display==NULL)
		return size;

	ret = nubia_dsi_panel_cabc(nubia_display->panel, val);	
        if (ret == 0) {
                nubia_disp_val.cabc = val;
                NUBIA_DISP_INFO("success to set cabc as = %d\n", val);
        }
        return size;
}


#ifdef CONFIG_NUBIA_SWITCH_LCD
static ssize_t rgbw_mode_show(struct kobject *kobj,
        struct kobj_attribute *attr, char *buf)
{
       if (nubia_disp_val.en_rgbw_mode)
                return snprintf(buf, PAGE_SIZE, "%d\n", nubia_disp_val.rgbw_mode);
        else
                return snprintf(buf, PAGE_SIZE, "NULL\n");

}

static ssize_t rgbw_mode_store(struct kobject *kobj,
        struct kobj_attribute *attr, const char *buf, size_t size)
{
        uint32_t val = 0;
        int ret = 0;
	if(!nubia_disp_val.en_rgbw_mode) {
                NUBIA_DISP_ERROR("no rgbw mode\n");
                return size;
        }

        sscanf(buf, "%d", &val);

        if ((val != RGBW_NORMAL) && (val != RGBW_BRIGHT) && (val != RGBW_OUTDOOR)) {
                NUBIA_DISP_ERROR("invalid rgbw mode val = %d\n", val);
                return size;
        }

        NUBIA_DISP_INFO("rgbw mode value = %d\n", val);

	if(nubia_display==NULL)
		return size;

	ret = nubia_dsi_panel_rgbw(nubia_display->panel, val);	
        if (ret == 0) {
                nubia_disp_val.rgbw_mode = val;
                NUBIA_DISP_INFO("success to set rgbw mode as = %d\n", val);
        }
        return size;
}




static ssize_t aod_mode_show(struct kobject *kobj,
        struct kobj_attribute *attr, char *buf)
{
       if (nubia_disp_val.en_aod_mode)
                return snprintf(buf, PAGE_SIZE, "%d\n", nubia_disp_val.aod_mode);
        else
                return snprintf(buf, PAGE_SIZE, "NULL\n");

}

static ssize_t aod_mode_store(struct kobject *kobj,
        struct kobj_attribute *attr, const char *buf, size_t size)
{
        uint32_t val = 0;
        int ret = 0;
	if(!nubia_disp_val.en_aod_mode) {
                NUBIA_DISP_ERROR("no aod mode\n");
                return size;
        }

        sscanf(buf, "%d", &val);

        if ((val != AOD_OFF) && (val != AOD_ON)) {
                NUBIA_DISP_ERROR("invalid AOD mode val = %d\n", val);
                return size;
        }

        NUBIA_DISP_INFO("aod mode value = %d\n", val);

	if(nubia_display==NULL)
		return size;

	ret = nubia_dsi_panel_aod(nubia_display->panel, val);	
        if (ret == 0) {
                nubia_disp_val.aod_mode = val;
                NUBIA_DISP_INFO("success to set aod mode as = %d\n", val);
        }
        return size;
}

static ssize_t lcd_state_show(struct kobject *kobj,
        struct kobj_attribute *attr, char *buf)
{
	 nubia_disp_val.lcd_state =  lcd_states;
       pr_err("%s(jbc): start lcd_states = %d\n", __func__,nubia_disp_val.lcd_state);	
       if (nubia_disp_val.en_lcd_state)
                return snprintf(buf, PAGE_SIZE, "%d\n", nubia_disp_val.lcd_state);
        else
                return snprintf(buf, PAGE_SIZE, "NULL\n");

}
static ssize_t lcd_state_store(struct kobject *kobj,
        struct kobj_attribute *attr, const char *buf, size_t size)
{
        uint32_t val = 0;

	if(!nubia_disp_val.en_lcd_state) {
                NUBIA_DISP_ERROR("no lcd_state\n");
                return size;
        }

        sscanf(buf, "%d", &val);

        if ((val != DISPLAY_POST_PRIMARY) && (val != DISPLAY_POST_SECOND)) {
                NUBIA_DISP_ERROR("invalid lcd_state val = %d\n", val);
                return size;
        }

	if(nubia_display==NULL)
		return size;
       NUBIA_DISP_INFO("lcd_state value = %d\n", val);
      nubia_disp_val.lcd_state  =  val;
      lcd_states = val;
       return size;
}
extern bool nubia_back_lcd_exist;

static int lcd_exist_nb_handler(struct notifier_block *cb, unsigned long code, void *unused)
{
	NUBIA_DISP_INFO("jbc %s code:%lu\n", __func__, code);
	switch (code) {
	case 0x88:
		nubia_back_lcd_exist = false;
		lcd_states = DISPLAY_ONLY_PRIMARY;
	     NUBIA_DISP_INFO("Back lcd and tp is not exist,force to %d",lcd_states);
	default:
		break;
	}
	return NOTIFY_DONE;
}

static struct notifier_block lcd_exist_nb = {
	.notifier_call = lcd_exist_nb_handler,
};
#endif


static struct kobj_attribute lcd_disp_attrs[] = {
	__ATTR(cabc,        0664, cabc_show,       cabc_store),
#ifdef CONFIG_NUBIA_SWITCH_LCD
	__ATTR(rgbw_mode,        0664, rgbw_mode_show,       rgbw_mode_store),
	__ATTR(aod_mode, 		 0664, aod_mode_show,		  aod_mode_store),
	__ATTR(lcd_state,        0664, lcd_state_show,       lcd_state_store),
#endif
};

void nubia_set_dsi_ctrl(struct dsi_display *display)
{
	NUBIA_DISP_INFO("start\n");
	nubia_display = display;
}

static int __init nubia_disp_preference_init(void)
{
	int retval = 0;
	int attr_count = 0;

	NUBIA_DISP_INFO("start\n");

	enhance_kobj = kobject_create_and_add("lcd_enhance", kernel_kobj);

	if (!enhance_kobj) {
		NUBIA_DISP_ERROR("failed to create and add kobject\n");
		return -ENOMEM;
	}

	/* Create attribute files associated with this kobject */
	for (attr_count = 0; attr_count < ARRAY_SIZE(lcd_disp_attrs); attr_count++) {
		retval = sysfs_create_file(enhance_kobj, &lcd_disp_attrs[attr_count].attr);
		if (retval < 0) {
			NUBIA_DISP_ERROR("failed to create sysfs attributes\n");
			goto err_sys_creat;
		}
	}
#ifdef CONFIG_NUBIA_SWITCH_LCD
	goodix_ts_exist_register_notifier(&lcd_exist_nb);
#endif
	NUBIA_DISP_INFO("success\n");

	return retval;

err_sys_creat:
//#ifdef CONFIG_NUBIA_SWITCH_LCD
	//cancel_delayed_work_sync(&nubia_disp_val.lcd_states_work);
//#endif
	for (--attr_count; attr_count >= 0; attr_count--)
		sysfs_remove_file(enhance_kobj, &lcd_disp_attrs[attr_count].attr);

	kobject_put(enhance_kobj);
	return retval;
}

static void __exit nubia_disp_preference_exit(void)
{
	int attr_count = 0;

	for (attr_count = 0; attr_count < ARRAY_SIZE(lcd_disp_attrs); attr_count++)
		sysfs_remove_file(enhance_kobj, &lcd_disp_attrs[attr_count].attr);

	kobject_put(enhance_kobj);
}

MODULE_AUTHOR("NUBIA LCD Driver Team Software");
MODULE_DESCRIPTION("NUBIA LCD DISPLAY Color Saturation and Temperature Setting");
MODULE_LICENSE("GPL");
module_init(nubia_disp_preference_init);
module_exit(nubia_disp_preference_exit);
