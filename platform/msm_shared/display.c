/* Copyright (c) 2012-2016, The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of The Linux Foundation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <debug.h>
#include <err.h>
#include <msm_panel.h>
#include <mdp4.h>
#include <mipi_dsi.h>
#include <boot_stats.h>
#include <platform.h>
#include <malloc.h>
#include <qpic.h>
#include <target.h>
#ifdef DISPLAY_TYPE_MDSS
#include <target/display.h>
#endif

static struct msm_fb_panel_data *panel;

#ifdef EARLY_CAMERA_SUPPORT
static struct msm_fb_panel_data *panel_array= NULL;
static uint32_t num_panel = 0;
#endif

extern int lvds_on(struct msm_fb_panel_data *pdata);

static int msm_fb_alloc(struct fbcon_config *fb)
{
	if (fb == NULL)
		return ERROR;

#ifdef EARLY_CAMERA_SUPPORT
	/* Static splash buffer */
	int num_buffers = 1;

	if (target_animated_splash_screen()) {
		/* Static splash + animated splash buffers */
		dprintf(SPEW, "Allocate extra buffers for animates splash\n");
		num_buffers = 13;
	}
#endif

	if (fb->base == NULL)
		fb->base = memalign(4096, fb->width
							* fb->height
							* (fb->bpp / 8)
#ifdef EARLY_CAMERA_SUPPORT
							* num_buffers
#endif
							);

	if (fb->base == NULL) {
#ifdef EARLY_CAMERA_SUPPORT 
		dprintf(CRITICAL, "Error in Allocating %d buffer\n", num_buffers);
#endif
		return ERROR;
	}
	else {
#ifdef EARLY_CAMERA_SUPPORT 
		dprintf(SPEW, "Allocated %d buffers\n", num_buffers);
#endif
		return NO_ERROR;
	}
}

int msm_display_config()
{
	int ret = NO_ERROR;
#ifdef DISPLAY_TYPE_MDSS
	int mdp_rev;
#endif
	struct msm_panel_info *pinfo;

	if (!panel)
		return ERR_INVALID_ARGS;

	pinfo = &(panel->panel_info);

	/* Set MDP revision */
	mdp_set_revision(panel->mdp_rev);

	switch (pinfo->type) {
#ifdef DISPLAY_TYPE_MDSS
	case LVDS_PANEL:
		dprintf(INFO, "Config LVDS_PANEL.\n");
		ret = mdp_lcdc_config(pinfo, &(panel->fb));
		if (ret)
			goto msm_display_config_out;
		break;
	case MIPI_VIDEO_PANEL:
		dprintf(INFO, "Config MIPI_VIDEO_PANEL.\n");

		mdp_rev = mdp_get_revision();
#ifdef EARLY_CAMERA_SUPPORT
		if (pinfo->dest == DISPLAY_1) {
#endif
		if (mdp_rev == MDP_REV_50 || mdp_rev == MDP_REV_304 ||
						mdp_rev == MDP_REV_305)
			ret = mdss_dsi_config(panel);
		else
			ret = mipi_config(panel);

#ifdef EARLY_CAMERA_SUPPORT
		}
#endif
		if (ret)
			goto msm_display_config_out;

		if (pinfo->early_config)
			ret = pinfo->early_config((void *)pinfo);

		ret = mdp_dsi_video_config(pinfo, &(panel->fb));
		if (ret)
			goto msm_display_config_out;
		break;
	case MIPI_CMD_PANEL:
		dprintf(INFO, "Config MIPI_CMD_PANEL.\n");
		mdp_rev = mdp_get_revision();
		if (mdp_rev == MDP_REV_50 || mdp_rev == MDP_REV_304 ||
						mdp_rev == MDP_REV_305)
			ret = mdss_dsi_config(panel);
		else
			ret = mipi_config(panel);
		if (ret)
			goto msm_display_config_out;

		ret = mdp_dsi_cmd_config(pinfo, &(panel->fb));
		if (ret)
			goto msm_display_config_out;
		break;
	case LCDC_PANEL:
		dprintf(INFO, "Config LCDC PANEL.\n");
		ret = mdp_lcdc_config(pinfo, &(panel->fb));
		if (ret)
			goto msm_display_config_out;
		break;
	case HDMI_PANEL:
		dprintf(INFO, "Config HDMI PANEL.\n");
		ret = mdss_hdmi_config(pinfo, &(panel->fb));
		if (ret)
			goto msm_display_config_out;
		break;
	case EDP_PANEL:
		dprintf(INFO, "Config EDP PANEL.\n");
		ret = mdp_edp_config(pinfo, &(panel->fb));
		if (ret)
			goto msm_display_config_out;
		break;
#endif
#ifdef DISPLAY_TYPE_QPIC
	case QPIC_PANEL:
		dprintf(INFO, "Config QPIC_PANEL.\n");
		qpic_init(pinfo, (int) panel->fb.base);
		break;
#endif
	default:
		return ERR_INVALID_ARGS;
	};

	if (pinfo->config)
		ret = pinfo->config((void *)pinfo);

#ifdef DISPLAY_TYPE_MDSS
msm_display_config_out:
#endif
	return ret;
}

int msm_display_on()
{
	int ret = NO_ERROR;
#ifdef DISPLAY_TYPE_MDSS
	int mdp_rev;
#endif
	struct msm_panel_info *pinfo;

	if (!panel)
		return ERR_INVALID_ARGS;

	bs_set_timestamp(BS_SPLASH_SCREEN_DISPLAY);

	pinfo = &(panel->panel_info);

	if (pinfo->pre_on) {
		ret = pinfo->pre_on();
		if (ret)
			goto msm_display_on_out;
	}

	switch (pinfo->type) {
#ifdef DISPLAY_TYPE_MDSS
	case LVDS_PANEL:
		dprintf(INFO, "Turn on LVDS PANEL.\n");
		ret = mdp_lcdc_on(panel);
		if (ret)
			goto msm_display_on_out;
		ret = lvds_on(panel);
		if (ret)
			goto msm_display_on_out;
		break;
	case MIPI_VIDEO_PANEL:
		dprintf(INFO, "Turn on MIPI_VIDEO_PANEL.\n");
		ret = mdp_dsi_video_on(pinfo);
		if (ret)
			goto msm_display_on_out;

		ret = mdss_dsi_post_on(panel);
		if (ret)
			goto msm_display_on_out;

		ret = mipi_dsi_on(pinfo);
		if (ret)
			goto msm_display_on_out;
		break;
	case MIPI_CMD_PANEL:
		dprintf(INFO, "Turn on MIPI_CMD_PANEL.\n");
		ret = mdp_dma_on(pinfo);
		if (ret)
			goto msm_display_on_out;
		mdp_rev = mdp_get_revision();
		if (mdp_rev != MDP_REV_50 && mdp_rev != MDP_REV_304 &&
						mdp_rev != MDP_REV_305) {
			ret = mipi_cmd_trigger();
			if (ret)
				goto msm_display_on_out;
		}

		ret = mdss_dsi_post_on(panel);
		if (ret)
			goto msm_display_on_out;

		break;
	case LCDC_PANEL:
		dprintf(INFO, "Turn on LCDC PANEL.\n");
		ret = mdp_lcdc_on(panel);
		if (ret)
			goto msm_display_on_out;
		break;
	case HDMI_PANEL:
		dprintf(INFO, "Turn on HDMI PANEL.\n");
		ret = mdss_hdmi_init();
		if (ret)
			goto msm_display_on_out;

		ret = mdss_hdmi_on(pinfo);
		if (ret)
			goto msm_display_on_out;
		break;
	case EDP_PANEL:
		dprintf(INFO, "Turn on EDP PANEL.\n");
		ret = mdp_edp_on(pinfo);
		if (ret)
			goto msm_display_on_out;
		break;
#endif
#ifdef DISPLAY_TYPE_QPIC
	case QPIC_PANEL:
		dprintf(INFO, "Turn on QPIC_PANEL.\n");
		ret = qpic_on();
		if (ret) {
			dprintf(CRITICAL, "QPIC panel on failed\n");
			goto msm_display_on_out;
		}
		qpic_update();
		break;
#endif
	default:
		return ERR_INVALID_ARGS;
	};

	if (pinfo->on)
		ret = pinfo->on();

msm_display_on_out:
	return ret;
}

#ifdef EARLY_CAMERA_SUPPORT
struct fbcon_config* msm_display_get_fb(uint32_t disp_id)
{
	if (panel_array == NULL)
		return NULL;
	else
		return &(panel_array[disp_id].fb);
}

int msm_display_update(struct fbcon_config *fb, uint32_t pipe_id, uint32_t pipe_type,
	uint32_t zorder, uint32_t width, uint32_t height, uint32_t disp_id)
{
	struct msm_panel_info *pinfo;
	struct msm_fb_panel_data *panel_local;
	int ret = 0;
	if (!panel_array || !fb) {
		dprintf(CRITICAL, "Error! Inalid args\n");
		return ERR_INVALID_ARGS;
	}
	panel_local = &(panel_array[disp_id]);
	panel_local->fb = *fb;
	pinfo = &(panel_local->panel_info);
	pinfo->pipe_type = pipe_type;
	pinfo->zorder = zorder;
	pinfo->border_top = fb->height/2 - height/2;
	pinfo->border_bottom = pinfo->border_top;
	pinfo->border_left = fb->width/2 - width/2;
	pinfo->border_right = pinfo->border_left;

	switch (pinfo->type) {
		case MIPI_VIDEO_PANEL:
			ret = mdp_dsi_video_config(pinfo, fb);
			if (ret) {
				dprintf(CRITICAL, "ERROR in display config\n");
				goto msm_display_update_out;
			}
			ret = mdp_dsi_video_update(pinfo);
			if (ret) {
				dprintf(CRITICAL, "ERROR in display upate\n");
				goto msm_display_update_out;
			}
			break;
		case HDMI_PANEL:
			ret = mdss_hdmi_config(pinfo, fb);
			if (ret) {
				dprintf(CRITICAL, "ERROR in display config\n");
				goto msm_display_update_out;
			}
			ret = mdss_hdmi_update(pinfo);
			if (ret) {
				dprintf(CRITICAL, "ERROR in display upate\n");
				goto msm_display_update_out;
			}
			break;
		default:
			dprintf(SPEW, "Update not supported right now\n");
			break;
	}

msm_display_update_out:
	return ret;
}

int msm_display_remove_pipe(uint32_t pipe_id, uint32_t pipe_type, uint32_t disp_id)
{
	struct msm_panel_info *pinfo;
	struct msm_fb_panel_data *panel_local;
	int ret = 0;
	panel_local = &(panel_array[disp_id]);

	if (!panel_array) {
		dprintf(CRITICAL, "Error! Inalid args\n");
		return ERR_INVALID_ARGS;
	}
	pinfo = &(panel_local->panel_info);
	pinfo->pipe_type = pipe_type;
	pinfo->pipe_id = pipe_id;

	ret = mdss_layer_mixer_remove_pipe(pinfo);
	if (ret) {
		dprintf(CRITICAL, "Error in mdss_layer_mixer_remove_pipe\n");
		return ret;
	} else {
		if (pinfo->type == MIPI_VIDEO_PANEL)
			return mdp_dsi_video_update(pinfo);
		else
			return mdss_hdmi_update(pinfo);
	}
}
#endif

int msm_display_init(struct msm_fb_panel_data *pdata)
{
	int ret = NO_ERROR;

#ifdef EARLY_CAMERA_SUPPORT
	if (panel_array == NULL) {
		int num_target_display;
		num_target_display = target_get_max_display();
		panel_array = malloc(num_target_display * sizeof(struct  msm_fb_panel_data));
	}
#endif
	panel = pdata;
	if (!panel) {
		ret = ERR_INVALID_ARGS;
		goto msm_display_init_out;
	}

	/* Turn on panel */
	if (pdata->power_func)
		ret = pdata->power_func(1, &(panel->panel_info));

	if (ret)
		goto msm_display_init_out;

	if (pdata->dfps_func)
		ret = pdata->dfps_func(&(panel->panel_info));

	/* Enable clock */
	if (pdata->clk_func)
		ret = pdata->clk_func(1, &(panel->panel_info));

	if (ret)
		goto msm_display_init_out;

	/* Read specifications from panel if available.
	 * If further clocks should be enabled, they can be enabled
	 * using pll_clk_func
	 */
	if (pdata->update_panel_info)
		ret = pdata->update_panel_info();

	if (ret)
		goto msm_display_init_out;

	/* Enabled for auto PLL calculation or to enable
	 * additional clocks
	 */
	if (pdata->pll_clk_func)
		ret = pdata->pll_clk_func(1, &(panel->panel_info));

	if (ret)
		goto msm_display_init_out;

	/* pinfo prepare  */
	if (pdata->panel_info.prepare) {
		/* this is for edp which pinfo derived from edid */
		ret = pdata->panel_info.prepare();
		panel->fb.width =  panel->panel_info.xres;
		panel->fb.height =  panel->panel_info.yres;
		panel->fb.stride =  panel->panel_info.xres;
		panel->fb.bpp =  panel->panel_info.bpp;
	}

	if (ret)
		goto msm_display_init_out;

	ret = msm_fb_alloc(&(panel->fb));
	if (ret)
		goto msm_display_init_out;

	fbcon_setup(&(panel->fb));
	display_image_on_screen();

	if ((panel->dsi2HDMI_config) && (panel->panel_info.has_bridge_chip))
		ret = panel->dsi2HDMI_config(&(panel->panel_info));
	if (ret)
		goto msm_display_init_out;

	ret = msm_display_config();
	if (ret)
		goto msm_display_init_out;

	ret = msm_display_on();
	if (ret)
		goto msm_display_init_out;

	if (pdata->post_power_func)
		ret = pdata->post_power_func(1);
	if (ret)
		goto msm_display_init_out;

	/* Turn on backlight */
	if (pdata->bl_func)
		ret = pdata->bl_func(1);

	if (ret)
		goto msm_display_init_out;

#ifdef EARLY_CAMERA_SUPPORT
	// if panel init correctly, save the panel struct in the array
	memcpy((void*) &panel_array[num_panel], (void*) panel, sizeof(struct  msm_fb_panel_data));
	dprintf (INFO, "Panel %d init, width:%d height:%d\n", num_panel, panel->fb.width, panel->fb.height);
	num_panel ++;
#endif

msm_display_init_out:
	return ret;
}

int msm_display_off()
{
	int ret = NO_ERROR;
	struct msm_panel_info *pinfo;

	if (!panel)
		return ERR_INVALID_ARGS;

	pinfo = &(panel->panel_info);

	if (pinfo->pre_off) {
		ret = pinfo->pre_off();
		if (ret)
			goto msm_display_off_out;
	}

	switch (pinfo->type) {
#ifdef DISPLAY_TYPE_MDSS
	case LVDS_PANEL:
		dprintf(INFO, "Turn off LVDS PANEL.\n");
		mdp_lcdc_off();
		break;
	case MIPI_VIDEO_PANEL:
		dprintf(INFO, "Turn off MIPI_VIDEO_PANEL.\n");
		ret = mdp_dsi_video_off(pinfo);
		if (ret)
			goto msm_display_off_out;
		ret = mipi_dsi_off(pinfo);
		if (ret)
			goto msm_display_off_out;
		break;
	case MIPI_CMD_PANEL:
		dprintf(INFO, "Turn off MIPI_CMD_PANEL.\n");
		ret = mdp_dsi_cmd_off();
		if (ret)
			goto msm_display_off_out;
		ret = mipi_dsi_off(pinfo);
		if (ret)
			goto msm_display_off_out;
		break;
	case LCDC_PANEL:
		dprintf(INFO, "Turn off LCDC PANEL.\n");
		mdp_lcdc_off();
		break;
	case EDP_PANEL:
		dprintf(INFO, "Turn off EDP PANEL.\n");
		ret = mdp_edp_off();
		if (ret)
			goto msm_display_off_out;
		break;
	case HDMI_PANEL:
		dprintf(INFO, "Turn off HDMI PANEL.\n");
		ret = mdss_hdmi_off(pinfo);
		break;

#endif
#ifdef DISPLAY_TYPE_QPIC
	case QPIC_PANEL:
		dprintf(INFO, "Turn off QPIC_PANEL.\n");
		qpic_off();
		break;
#endif
	default:
		return ERR_INVALID_ARGS;
	};

	if (target_cont_splash_screen()) {
		dprintf(INFO, "Continuous splash enabled, keeping panel alive.\n");
		return NO_ERROR;
	}

	if (panel->post_power_func)
		ret = panel->post_power_func(0);
	if (ret)
		goto msm_display_off_out;

	/* Turn off backlight */
	if (panel->bl_func)
		ret = panel->bl_func(0);

	if (pinfo->off)
		ret = pinfo->off();

	/* Disable clock */
	if (panel->clk_func)
		ret = panel->clk_func(0, pinfo);

	/* Only for AUTO PLL calculation */
	if (panel->pll_clk_func)
		ret = panel->pll_clk_func(0, pinfo);

	if (ret)
		goto msm_display_off_out;

	/* Disable panel */
	if (panel->power_func)
		ret = panel->power_func(0, pinfo);

msm_display_off_out:
	return ret;
}
