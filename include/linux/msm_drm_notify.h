/*
 * Copyright (c) 2017, The Linux Foundation. All rights reserved.
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
#ifndef _MSM_DRM_NOTIFY_H_
#define _MSM_DRM_NOTIFY_H_

#include <linux/notifier.h>

/* A hardware display blank change occurred */
#define MSM_DRM_EVENT_BLANK			0x01
/* A hardware display blank early change occurred */
#define MSM_DRM_EARLY_EVENT_BLANK		0x02

enum {
	/* panel: power on */
	MSM_DRM_BLANK_UNBLANK,
	/* panel: power off */
	MSM_DRM_BLANK_POWERDOWN,
};

enum msm_drm_display_id {
	/* primary display */
	MSM_DRM_PRIMARY_DISPLAY,
	/* external display */
	MSM_DRM_EXTERNAL_DISPLAY,
	MSM_DRM_DISPLAY_MAX
};

struct msm_drm_notifier {
	enum msm_drm_display_id id;
	void *data;
};

#ifdef CONFIG_NUBIA_SWITCH_LCD
/* A hardware display blank change occurred */
#define MSM_DRM_SWITCH_EVENT_BLANK			0x01
/* A hardware display blank early change occurred */
#define MSM_DRM_SWITCH_EARLY_EVENT_BLANK		0x02
/* A hardware display blank or aod mode change occurred */
#define FB_EARLY_EVENT_BLANK_FP		0x03

void dsi_panel_notifier(int event, unsigned long data);

enum {
	/* major panel: power on */
	MSM_DRM_MAJOR_BLANK_UNBLANK,
	/* major panel: power off */
	MSM_DRM_MAJOR_POWERDOWN,
	/* slave panel: power on */
	MSM_DRM_SLAVE_BLANK_UNBLANK,
	/* slave panel: power off */
	MSM_DRM_SLAVE_POWERDOWN,
	/* fingerprint: power on */
	FB_BLANK_UNBLANK_FP,
	/* fingerprint: power off */
	FB_BLANK_POWERDOWN_FP

};

enum msm_drm_switch_display_id {
	/* major display */
	MSM_DRM_SWITCH_MAJOR_PANEL,
	/* slave display */
	MSM_DRM_SWITCH_SLAVE_PANEL
};

struct msm_drm_switch_notifier {
	enum msm_drm_switch_display_id id;
	/* transfer panel status*/
	void *data;
};
int msm_drm_switch_register_client(struct notifier_block *nb);
int msm_drm_switch_unregister_client(struct notifier_block *nb);
#endif


int msm_drm_register_client(struct notifier_block *nb);
int msm_drm_unregister_client(struct notifier_block *nb);
#endif
