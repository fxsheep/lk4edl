/* 
 * Copyright (c) Yuvraj Saxena <ysaxenax@gmail.com>
 * All rights reserved.
 */

#ifndef _PANEL_HX8394F_BOE_720P_VIDEO_H_
#define _PANEL_HX8394F_BOE_720P_VIDEO_H_

	/* HEADER files */

#include "panel.h"

	/* Panel configuration */

static struct panel_config hx8394f_boe_720p_video_panel_data = {
	"qcom,mdss_dsi_hx8394f_boe_720p_video", "dsi:0:", "qcom,mdss-dsi-panel",
	10, 0, "DISPLAY_1", 0, 0, 60, 0, 0, 0, 1, 50000, 0, 0, 0, 0, 0, 0, NULL
};

	/* Panel resolution */

static struct panel_resolution hx8394f_boe_720p_video_panel_res = {
	720, 1280, 160, 160, 24, 0, 15, 12, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

	/* Panel color information */

static struct color_info hx8394f_boe_720p_video_color = {
	24, 0, 0xff, 0, 0, 0
};

	/* Panel on/off command information */

static char hx8394f_boe_720p_video_on_cmd0[] = {
	0x04, 0x00, 0x39, 0xC0,
	0xB9, 0xFF, 0x83, 0x94
};

static char hx8394f_boe_720p_video_on_cmd1[] = {
	0x11, 0x00, 0x05, 0x80,
};

static char hx8394f_boe_720p_video_on_cmd2[] = {
	0x29, 0x00, 0x05, 0x80,
};

static struct mipi_dsi_cmd hx8394f_boe_720p_video_on_command[] = {
	{0x8, hx8394f_boe_720p_video_on_cmd0, 0x00},
	{0x4, hx8394f_boe_720p_video_on_cmd1, 0xC8},
	{0x4, hx8394f_boe_720p_video_on_cmd2, 0xC8},
};

#define HX8394F_BOE_720P_VIDEO_ON_COMMAND 3

static char hx8394f_boe_720p_videooff_cmd0[] = {
	0x28, 0x00, 0x05, 0x80
};

static char hx8394f_boe_720p_videooff_cmd1[] = {
	0x10, 0x00, 0x05, 0x80
};

static struct mipi_dsi_cmd hx8394f_boe_720p_video_off_command[] = {
	{0x4, hx8394f_boe_720p_videooff_cmd0, 0x32},
	{0x4, hx8394f_boe_720p_videooff_cmd1, 0x78}
};

#define HX8394F_BOE_720P_VIDEO_OFF_COMMAND 2


static struct command_state hx8394f_boe_720p_video_state = {
	0, 1
};

	/* Command mode panel information */

static struct commandpanel_info hx8394f_boe_720p_video_command_panel = {
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

	/* Video mode panel information */

static struct videopanel_info hx8394f_boe_720p_video_video_panel = {
	1, 0, 0, 0, 1, 1, 2, 0, 0x9
};

	/* Lane configuration */

static struct lane_configuration hx8394f_boe_720p_video_lane_config = {
  4, 0, 1, 1, 1, 1, 0
};

	/* Panel timing */

static const uint32_t hx8394f_boe_720p_video_timings[] = {
	0x8C, 0x1E, 0x14, 0x00, 0x46, 0x48, 0x1A, 0x22, 0x18, 0x03, 0x04, 0x00
}

static struct panel_timing hx8394f_boe_720p_video_timing_info = {
	0, 4, 0x04, 0x1C
};

	/* Panel reset sequence */

static struct panel_reset_sequence hx8394f_boe_720p_video_reset_seq = {
	{1, 0, 1, }, {50, 20, 70, }, 2
};

	/* Backlight setting */

static struct backlight hx8394f_boe_720p_video_backlight = {
	1, 1, 4095, 100, 1, "PMIC_8941"
};

static struct labibb_desc hx8394f_boe_720p_video_labibb = {
	0, 1, 5000000,5000000, 5000000, 5000000, 3, 3, 1, 0
};

#define HX8394F_BOE_720P_VIDEO_SIGNATURE 0x83940F

#endif /*_PANEL_HX8394F_BOE_720P_VIDEO_H_*/
