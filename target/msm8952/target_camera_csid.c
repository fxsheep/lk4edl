/*
Copyright (c) 2017, The Linux Foundation. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above
      copyright notice, this list of conditions and the following
      disclaimer in the documentation and/or other materials provided
      with the distribution.
    * Neither the name of The Linux Foundation nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <sys/types.h>
#include <platform/gpio.h>
#include <gsbi.h>
#include <regulator.h>
#include <rpm-ipc.h>
#include <pm8x41.h>
#include <platform/timer.h>
#include <string.h>
#include <stdlib.h>
#include <reg.h>
#include <target/display.h>
#include <kernel/thread.h>
#include <target/target_camera.h>
#include <platform/clock.h>
#include <clock.h>
#include "target_camera_common.h"

#define CSID_VERSION_V34_2                    0x30040002

#define CSID_BASE                     0x1b30400

///#define CSID_BASE                     0x001b30000

#define SHORT_PKT_OFFSET              0x200
#define MAX_POLL_COUNT                1000

#define msm_camera_io_w_mb(d,a) writel(d,a)
#define msm_camera_io_w(d,a)    writel(d,a)
#define msm_camera_io_r(a)      readl(a)
#define msm_camera_io_r_mb(a)   readl(a)

struct msm_camera_csid_params ov8865_csid_params = {
	.lane_cnt = 2,
	.lane_assign = 0x4320,
	.phy_sel = 1,
	.csi_clk = 0,
	.lut_params = {
		.num_cid = 1,
		.vc_cfg  = {
			.cid = 0,
			.dt = CSI_RAW10,
			.decode_format = CSI_DECODE_10BIT,
		},
	},
	.csi_3p_sel = 0,
};

enum csiphy_lane_assign {
	PHY_LANE_D0,
	PHY_LANE_CLK,
	PHY_LANE_D1,
	PHY_LANE_D2,
	PHY_LANE_D3,
	PHY_LANE_MAX,
};

struct csid_reg_parms_t {
    /* MIPI	CSID registers */
	uint32_t csid_hw_version_addr;
	uint32_t csid_core_ctrl_0_addr;
	uint32_t csid_core_ctrl_1_addr;
	uint32_t csid_rst_cmd_addr;
	uint32_t csid_cid_lut_vc_0_addr;
	uint32_t csid_cid_lut_vc_1_addr;
	uint32_t csid_cid_lut_vc_2_addr;
	uint32_t csid_cid_lut_vc_3_addr;
	uint32_t csid_cid_n_cfg_addr;
	uint32_t csid_irq_clear_cmd_addr;
	uint32_t csid_irq_mask_addr;
	uint32_t csid_irq_status_addr;
	uint32_t csid_captured_unmapped_long_pkt_hdr_addr;
	uint32_t csid_captured_mmaped_long_pkt_hdr_addr;
	uint32_t csid_captured_short_pkt_addr;
	uint32_t csid_captured_long_pkt_hdr_addr;
	uint32_t csid_captured_long_pkt_ftr_addr;
	uint32_t csid_pif_misr_dl0_addr;
	uint32_t csid_pif_misr_dl1_addr;
	uint32_t csid_pif_misr_dl2_addr;
	uint32_t csid_pif_misr_dl3_addr;
	uint32_t csid_stats_total_pkts_rcvd_addr;
	uint32_t csid_stats_ecc_addr;
	uint32_t csid_stats_crc_addr;
	uint32_t csid_tg_ctrl_addr;
	uint32_t csid_tg_vc_cfg_addr;
	uint32_t csid_tg_dt_n_cfg_0_addr;
	uint32_t csid_tg_dt_n_cfg_1_addr;
	uint32_t csid_tg_dt_n_cfg_2_addr;
	uint32_t csid_rst_done_irq_bitshift;
	uint32_t csid_rst_stb_all;
	uint32_t csid_dl_input_sel_shift;
	uint32_t csid_phy_sel_shift;
	uint32_t csid_version;
	uint32_t csid_3p_ctrl_0_addr;
	uint32_t csid_3p_pkt_hdr_addr;
	uint32_t csid_test_bus_ctrl;
	uint32_t csid_irq_mask_val;
	uint32_t csid_err_lane_overflow_offset_2p;
	uint32_t csid_err_lane_overflow_offset_3p;
	uint32_t csid_phy_sel_shift_3p;
};

uint8_t csid_lane_assign[PHY_LANE_MAX] = {0, 4, 1, 2, 3};
struct csid_reg_parms_t csid_reg = {
	/* MIPI	CSID registers */
	0x0,
	0x4,
	0x8,
	0xC,
	0x10,
	0x14,
	0x18,
	0x1C,
	0x20,
	0x60,
	0x64,
	0x68,
	0x6C,
	0x70,
	0x74,
	0x78,
	0x7C,
	0x80,
	0x84,
	0x88,
	0x8C,
	0x90,
	0x94,
	0x98,
	0xA0,
	0xA4,
	0xAC,
	0xB0,
	0xB4,
	11,
	0x7FFF,
	0x4,
	17,
	0x30040002,
	0xFFFFFFFF,
	0xFFFFFFFF,
	0xFFFFFFFF,
	0x7f010800,
	20,
	0xFFFFFFFF,
	0xFFFFFFFF,
};

#define CSI_D_INIT(_csid_base_) \
	{_csid_base_+0x04,0x32101}, \
	{_csid_base_+0x08,0x2000f}, \
	{_csid_base_+0x010,0x2b}, \
	{_csid_base_+0x020,0x23},

struct camera_hw_reg_array hw_csi1_d_init_regs[] = {
	CSI_D_INIT(CSID_BASE)
};

static uint32_t ldo2[][11]=
{
	{
		LDOA_RES_TYPE, 2,
		KEY_SOFTWARE_ENABLE, 4, GENERIC_DISABLE,
		KEY_MICRO_VOLT, 4, 0,
		KEY_CURRENT, 4, 0,
	},

	{
		LDOA_RES_TYPE, 2,
		KEY_SOFTWARE_ENABLE, 4, GENERIC_ENABLE,
		KEY_MICRO_VOLT, 4, 1200000,
		KEY_CURRENT, 4, 40,
	},
};

void camera_csid_clk_config(int csid_id,int enable){
	clk_get_set_enable("gcc_camss_top_ahb_clk",0,enable);
	clk_get_set_enable("gcc_camss_ispif_ahb_clk",61540000,enable);

	if(csid_id == 0){
		clk_get_set_enable("gcc_camss_csi0_ahb_clk",0,enable);
		clk_get_set_enable("csi0_clk_src",200000000,enable);
		clk_get_set_enable("gcc_camss_csi0_clk",0,enable);
		clk_get_set_enable("gcc_camss_csi0pix_clk",0,enable);
		clk_get_set_enable("gcc_camss_csi0rdi_clk",0,enable);
	}
	else if(csid_id == 1){
		clk_get_set_enable("gcc_camss_csi1_ahb_clk",0,enable);
		clk_get_set_enable("csi1_clk_src",200000000,enable);
		clk_get_set_enable("gcc_camss_csi1pix_clk",0,enable);
		clk_get_set_enable("gcc_camss_csi1rdi_clk",0,enable);
		clk_get_set_enable("gcc_camss_csi1_clk",0,enable);
	}
	clk_get_set_enable("gcc_camss_ahb_clk",0,enable);
}

int camera_cisd_irq(){
	uint32_t irq;
	uint32_t poll_count = 0;

	irq = msm_camera_io_r(CSID_BASE +csid_reg.csid_irq_status_addr);
	while(!(irq & (0x1 <<csid_reg.csid_rst_done_irq_bitshift))){
		irq = msm_camera_io_r(CSID_BASE +csid_reg.csid_irq_status_addr);
		poll_count++;
		if(poll_count > MAX_POLL_COUNT){
			dprintf(CRITICAL,"camera_cisd_irq: wait irq timeout\n");
			goto error_exit;
		}
	}
	msm_camera_io_w(irq, CSID_BASE +csid_reg.csid_irq_clear_cmd_addr);
	return 1;

error_exit:
	msm_camera_io_w(irq, CSID_BASE + csid_reg.csid_irq_clear_cmd_addr);
	msm_camera_io_w(0, CSID_BASE + csid_reg.csid_irq_mask_addr);
	return -1;
}

int camera_csid_reset(){
	int32_t rc = 0;
	msm_camera_io_w(csid_reg.csid_rst_stb_all,CSID_BASE +csid_reg.csid_rst_cmd_addr);
	rc = camera_cisd_irq();
	return rc;
}

void camera_csid_release(){
	uint32_t irq;

	irq = msm_camera_io_r(CSID_BASE + csid_reg.csid_irq_status_addr);
	msm_camera_io_w(irq, CSID_BASE + csid_reg.csid_irq_clear_cmd_addr);
	msm_camera_io_w(0, CSID_BASE + csid_reg.csid_irq_mask_addr);

}

void msm_hw_init(struct camera_hw_reg_array *pdata,int size) {
	int i = 0;

	for (i = 0; i < size; i++) {
		msm_camera_io_w_mb(pdata->reg_data,pdata->reg_addr);
		pdata++;
	}
}

int  camera_csid_init(){
	int32_t rc = 0;
	uint32_t hw_version = 0;
	rpm_send_data(&ldo2[1][0],36,RPM_REQUEST_TYPE); //1.2V DVDD

	camera_csid_clk_config(1,1);

	hw_version = msm_camera_io_r(CSID_BASE +csid_reg.csid_hw_version_addr);
	dprintf(CRITICAL,"camera_csid_init: read cisd hw_version = 0x%x\n",hw_version);

	rc = camera_csid_reset();
	if(rc > 0){
		msm_hw_init(&hw_csi1_d_init_regs[0],
			sizeof(hw_csi1_d_init_regs) / sizeof(hw_csi1_d_init_regs[0]));
	}
	else {
		camera_csid_release();
	}
	return rc;
}

void camera_csid_clock_disable(){
	camera_csid_clk_config(1,0);
}
