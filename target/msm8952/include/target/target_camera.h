/* Copyright (c) 2014-2017, The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of The Linux Foundation nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
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

#ifndef _TARGET_CAMERA_H
#define _TARGET_CAMERA_H

struct msm_vfe_error_info {
	uint32_t overflow_state;
	uint32_t error_mask0;
	uint32_t error_mask1;
	uint32_t violation_status;
	uint32_t camif_status;
	uint8_t stream_framedrop_count[4];
	uint8_t stats_framedrop_count[4];
	uint32_t info_dump_frame_count;
	uint32_t error_count;
	uint32_t framedrop_flag;
};

struct vfe_device {

	struct msm_vfe_error_info error_info;
	/* irq info */
	uint32_t irq0_mask;
	uint32_t irq1_mask;

	uint32_t vfe_hw_version;

	uint32_t id;
};

struct camera_hw_reg_array {
  unsigned int reg_addr;
  unsigned int reg_data;
};

void early_camera_fastrvc_thread(void);
void camera_csiphy_init();
void camera_csiphy_lane_config_start();
void camera_csiphy_release();
int  camera_csid_init();
int  camera_csid_config_start();
int  msm_ispif_init();
int  msm_vfe_irq(struct vfe_device *vfe_dev);
int  msm_isp_open_node(struct vfe_device *vfe_dev);
void msm_isp_vfe_start_stream(struct vfe_device *vfe_dev);
void msm_ispif_cfg();
int msm_vfe_gds_enable();
void msm_hw_init(struct camera_hw_reg_array *pdata,int size);
uint32_t msm_vfe_poll_irq0();
uint32_t msm_vfe_poll_irq1();
void msm_vfe_irq_mask_cfg(uint32_t irq0,uint32_t irq1);
uint32_t msm_vfe_pingpong_status();
void msm_get_camera_frame(uint8_t notify_status);
void vfe_cfg_start();
void msm_vfe_pingping_cfg(uint32_t ping_addr,uint32_t pong_addr);
void camera_csid_release();
void msm_ispif_release();
void msm_isp_close_node(struct vfe_device *vfe_dev);
void camera_csiphy_clock_disable();
void camera_csid_clock_disable();
void msm_isp_clock_disable();
void msm_ispif_clock_disable();
void nv12_to_rgb888(bool gpio_status);
void vfe_test_cfg(int i);
#endif
