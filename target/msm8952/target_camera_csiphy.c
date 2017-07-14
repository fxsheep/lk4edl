/**
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

#define msm_camera_io_w_mb(d,a) writel(d,a)
#define msm_camera_io_w(d,a)    writel(d,a)
#define msm_camera_io_r(a)      readl(a)
#define msm_camera_io_r_mb(a)   readl(a)

#define CSIPHY_ID  1
#define CSID_ID    1

// #define LANE_MASK  0x1f
// #define LANE_CNT   4
// #define SETTLE_CNT 0x14
// #define COMBO_MODE   0

#define CSIPHY_VERSION_V342                       0x342
#define CSIPHY_VERSION_V30                        0x10
#define CSIPHY_HW_VERSION                         CSIPHY_VERSION_V342

#define CSIPHY0_BASE                              0x1b34000
#define CSIPHY1_BASE                              0x1b35000
#define CSIPHY0_CLK_MUX_BASE                      0x1b00030
#define CSIPHY1_CLK_MUX_BASE                      0x1b00038

#define ULPM_WAKE_UP_TIMER_MODE                   2
#define GLITCH_ELIMINATION_NUM                    0x12 /* bit [6:4] */

#define MAX_LANES                                   4
#define CLOCK_OFFSET                              0x700

struct msm_camera_csiphy_params ov8865_csiphy_params = {
	.lane_cnt = 2,
	.settle_cnt = 0xE,
	.lane_mask = 0x7,
	.combo_mode = 0,
	.csid_core = 1,
	.csiphy_clk = 0,
	.csi_3phase = 0,
};

uint32_t csiphybase = 0;
uint32_t csiphy_clk_mux_base = 0;
uint32_t num_irq_registers = 0;


struct csiphy_reg_t {
	uint32_t addr;
	uint32_t data;
};

struct csiphy_reg_parms_t {
/*MIPI CSI PHY registers*/
	uint32_t mipi_csiphy_lnn_cfg1_addr;
	uint32_t mipi_csiphy_lnn_cfg2_addr;
	uint32_t mipi_csiphy_lnn_cfg3_addr;
	uint32_t mipi_csiphy_lnn_cfg4_addr;
	uint32_t mipi_csiphy_lnn_cfg5_addr;
	uint32_t mipi_csiphy_lnck_cfg1_addr;
	uint32_t mipi_csiphy_lnck_cfg2_addr;
	uint32_t mipi_csiphy_lnck_cfg3_addr;
	uint32_t mipi_csiphy_lnck_cfg4_addr;
	uint32_t mipi_csiphy_lnn_test_imp;
	uint32_t mipi_csiphy_lnn_misc1_addr;
	uint32_t mipi_csiphy_glbl_reset_addr;
	uint32_t mipi_csiphy_glbl_pwr_cfg_addr;
	uint32_t mipi_csiphy_glbl_irq_cmd_addr;
	uint32_t mipi_csiphy_hw_version_addr;
	uint32_t mipi_csiphy_interrupt_status0_addr;
	uint32_t mipi_csiphy_interrupt_mask0_addr;
	uint32_t mipi_csiphy_interrupt_mask_val;
	uint32_t mipi_csiphy_interrupt_mask_addr;
	uint32_t mipi_csiphy_interrupt_clear0_addr;
	uint32_t mipi_csiphy_interrupt_clear_addr;
	uint32_t mipi_csiphy_mode_config_shift;
	uint32_t mipi_csiphy_glbl_t_init_cfg0_addr;
	uint32_t mipi_csiphy_t_wakeup_cfg0_addr;
	uint32_t csiphy_version;
	uint32_t combo_clk_mask;
};

struct csiphy_reg_3ph_parms_t {
    /*MIPI CSI PHY registers*/
	struct csiphy_reg_t mipi_csiphy_3ph_cmn_ctrl5;
	struct csiphy_reg_t mipi_csiphy_3ph_cmn_ctrl6;
	struct csiphy_reg_t mipi_csiphy_3ph_lnn_ctrl34;
	struct csiphy_reg_t mipi_csiphy_3ph_lnn_ctrl35;
	struct csiphy_reg_t mipi_csiphy_3ph_lnn_ctrl36;
	struct csiphy_reg_t mipi_csiphy_3ph_lnn_ctrl1;
	struct csiphy_reg_t mipi_csiphy_3ph_lnn_ctrl2;
	struct csiphy_reg_t mipi_csiphy_3ph_lnn_ctrl3;
	struct csiphy_reg_t mipi_csiphy_3ph_lnn_ctrl5;
	struct csiphy_reg_t mipi_csiphy_3ph_lnn_ctrl6;
	struct csiphy_reg_t mipi_csiphy_3ph_lnn_ctrl7;
	struct csiphy_reg_t mipi_csiphy_3ph_lnn_ctrl8;
	struct csiphy_reg_t mipi_csiphy_3ph_lnn_ctrl9;
	struct csiphy_reg_t mipi_csiphy_3ph_lnn_ctrl10;
	struct csiphy_reg_t mipi_csiphy_3ph_lnn_ctrl11;
	struct csiphy_reg_t mipi_csiphy_3ph_lnn_ctrl12;
	struct csiphy_reg_t mipi_csiphy_3ph_lnn_ctrl13;
	struct csiphy_reg_t mipi_csiphy_3ph_lnn_ctrl14;
	struct csiphy_reg_t mipi_csiphy_3ph_lnn_ctrl15;
	struct csiphy_reg_t mipi_csiphy_3ph_lnn_ctrl16;
	struct csiphy_reg_t mipi_csiphy_3ph_lnn_ctrl17;
	struct csiphy_reg_t mipi_csiphy_3ph_lnn_ctrl18;
	struct csiphy_reg_t mipi_csiphy_3ph_lnn_ctrl19;
	struct csiphy_reg_t mipi_csiphy_3ph_lnn_ctrl21;
	struct csiphy_reg_t mipi_csiphy_3ph_lnn_ctrl23;
	struct csiphy_reg_t mipi_csiphy_3ph_lnn_ctrl24;
	struct csiphy_reg_t mipi_csiphy_3ph_lnn_ctrl25;
	struct csiphy_reg_t mipi_csiphy_3ph_lnn_ctrl26;
	struct csiphy_reg_t mipi_csiphy_3ph_lnn_ctrl27;
	struct csiphy_reg_t mipi_csiphy_3ph_lnn_ctrl28;
	struct csiphy_reg_t mipi_csiphy_3ph_lnn_ctrl29;
	struct csiphy_reg_t mipi_csiphy_3ph_lnn_ctrl30;
	struct csiphy_reg_t mipi_csiphy_3ph_lnn_ctrl31;
	struct csiphy_reg_t mipi_csiphy_3ph_lnn_ctrl32;
	struct csiphy_reg_t mipi_csiphy_3ph_lnn_ctrl33;
	struct csiphy_reg_t mipi_csiphy_3ph_lnn_ctrl51;
	struct csiphy_reg_t mipi_csiphy_3ph_cmn_ctrl7;
	struct csiphy_reg_t mipi_csiphy_3ph_cmn_ctrl11;
	struct csiphy_reg_t mipi_csiphy_3ph_cmn_ctrl12;
	struct csiphy_reg_t mipi_csiphy_3ph_cmn_ctrl13;
	struct csiphy_reg_t mipi_csiphy_3ph_cmn_ctrl14;
	struct csiphy_reg_t mipi_csiphy_3ph_cmn_ctrl15;
	struct csiphy_reg_t mipi_csiphy_3ph_cmn_ctrl16;
	struct csiphy_reg_t mipi_csiphy_3ph_cmn_ctrl17;
	struct csiphy_reg_t mipi_csiphy_3ph_cmn_ctrl18;
	struct csiphy_reg_t mipi_csiphy_3ph_cmn_ctrl19;
	struct csiphy_reg_t mipi_csiphy_3ph_cmn_ctrl20;
	struct csiphy_reg_t mipi_csiphy_3ph_cmn_ctrl21;
	struct csiphy_reg_t mipi_csiphy_2ph_lnn_misc1;
	struct csiphy_reg_t mipi_csiphy_3ph_cmn_ctrl0;
	struct csiphy_reg_t mipi_csiphy_2ph_lnn_cfg1;
	struct csiphy_reg_t mipi_csiphy_2ph_lnn_cfg2;
	struct csiphy_reg_t mipi_csiphy_2ph_lnn_cfg3;
	struct csiphy_reg_t mipi_csiphy_2ph_lnn_cfg4;
	struct csiphy_reg_t mipi_csiphy_2ph_lnn_cfg5;
	struct csiphy_reg_t mipi_csiphy_2ph_lnn_cfg6;
	struct csiphy_reg_t mipi_csiphy_2ph_lnn_cfg7;
	struct csiphy_reg_t mipi_csiphy_2ph_lnn_cfg8;
	struct csiphy_reg_t mipi_csiphy_2ph_lnn_cfg9;
	struct csiphy_reg_t mipi_csiphy_2ph_lnn_ctrl15;
	struct csiphy_reg_t mipi_csiphy_2ph_lnn_test_imp;
	struct csiphy_reg_t mipi_csiphy_2ph_lnn_test_force;
	struct csiphy_reg_t mipi_csiphy_2ph_lnn_ctrl5;
	struct csiphy_reg_t mipi_csiphy_3ph_lnck_cfg1;
};

struct csiphy_reg_parms_t csiphy_reg = {
	.mipi_csiphy_interrupt_status0_addr = 0x8B0,
	.mipi_csiphy_interrupt_clear0_addr = 0x858,
	.mipi_csiphy_glbl_irq_cmd_addr = 0x828,
	.combo_clk_mask = 0x10,
};

struct csiphy_reg_3ph_parms_t csiphy_3ph_reg = {
	/*MIPI CSI PHY registers*/
	{0x814, 0x0},
	{0x818, 0x1},
	{0x188, 0x7F},
	{0x18C, 0x7F},
	{0x190, 0x0},
	{0x104, 0x6},
	{0x108, 0x0},
	{0x10c, 0x0},
	{0x114, 0x20},
	{0x118, 0x3E},
	{0x11c, 0x41},
	{0x120, 0x41},
	{0x124, 0x7F},
	{0x128, 0x0},
	{0x12c, 0x0},
	{0x130, 0x1},
	{0x134, 0x0},
	{0x138, 0x0},
	{0x13C, 0x10},
	{0x140, 0x1},
	{0x144, GLITCH_ELIMINATION_NUM},
	{0x148, 0xFE},
	{0x14C, 0x1},
	{0x154, 0x0},
	{0x15C, 0x33},
	{0x160, ULPM_WAKE_UP_TIMER_MODE},
	{0x164, 0x48},
	{0x168, 0xA0},
	{0x16C, 0x17},
	{0x170, 0x41},
	{0x174, 0x41},
	{0x178, 0x3E},
	{0x17C, 0x0},
	{0x180, 0x0},
	{0x184, 0x7F},
	{0x1cc, 0x10},
	{0x81c, 0x6},
	{0x82c, 0xFF},
	{0x830, 0xFF},
	{0x834, 0xFB},
	{0x838, 0xFF},
	{0x83c, 0x7F},
	{0x840, 0xFF},
	{0x844, 0xFF},
	{0x848, 0xEF},
	{0x84c, 0xFF},
	{0x850, 0xFF},
	{0x854, 0xFF},
	{0x28, 0x0},
	{0x800, 0x2},
	{0x0, 0x8E},
	{0x4, 0x8},
	{0x8, 0x0},
	{0xC, 0xFF},
	{0x10, 0x56},
	{0x2C, 0x1},
	{0x30, 0x0},
	{0x34, 0x3},
	{0x38, 0xfe},
	{0x3C, 0xB8},
	{0x1C, 0xE7},
	{0x14, 0x0},
	{0x14, 0x60},
	{0x700, 0x80}
};


#define CSI_PHY_INIT(_csi_phy_base_) \
{_csi_phy_base_ + 0x814 ,0x85},\
{_csi_phy_base_ + 0x818 ,0x1},\
{_csi_phy_base_ + 0x030 ,0x0},\
{_csi_phy_base_ + 0x02c ,0x1},\
{_csi_phy_base_ + 0x034 ,0x3},\
{_csi_phy_base_ + 0x028 ,0x0},\
{_csi_phy_base_ + 0x03c ,0xb8},\
{_csi_phy_base_ + 0x004 ,0x8},\
{_csi_phy_base_ + 0x008 ,0xe},\
{_csi_phy_base_ + 0x000 ,0x8e},\
{_csi_phy_base_ + 0x010 ,0x56},\
{_csi_phy_base_ + 0x038 ,0xfe},\
{_csi_phy_base_ + 0x01c ,0xe7},\
{_csi_phy_base_ + 0x014 ,0x60},\
{_csi_phy_base_ + 0x730 ,0x0},\
{_csi_phy_base_ + 0x72c ,0x1},\
{_csi_phy_base_ + 0x734 ,0x3},\
{_csi_phy_base_ + 0x728 ,0x4},\
{_csi_phy_base_ + 0x73c ,0xb8},\
{_csi_phy_base_ + 0x704 ,0x8},\
{_csi_phy_base_ + 0x708 ,0xe},\
{_csi_phy_base_ + 0x700 ,0x80},\
{_csi_phy_base_ + 0x70c ,0xff},\
{_csi_phy_base_ + 0x710 ,0x56},\
{_csi_phy_base_ + 0x738 ,0x1f},\
{_csi_phy_base_ + 0x71c ,0xe7},\
{_csi_phy_base_ + 0x714 ,0x60},\
{_csi_phy_base_ + 0x230 ,0x0},\
{_csi_phy_base_ + 0x22c ,0x1},\
{_csi_phy_base_ + 0x234 ,0x3},\
{_csi_phy_base_ + 0x228 ,0x0},\
{_csi_phy_base_ + 0x23c ,0xb8},\
{_csi_phy_base_ + 0x204 ,0x8},\
{_csi_phy_base_ + 0x208 ,0xe},\
{_csi_phy_base_ + 0x200 ,0x8e},\
{_csi_phy_base_ + 0x210 ,0x56},\
{_csi_phy_base_ + 0x238 ,0xfe},\
{_csi_phy_base_ + 0x21c ,0xe7},\
{_csi_phy_base_ + 0x214 ,0x60},\
{_csi_phy_base_ + 0x800 ,0x2},\
{_csi_phy_base_ + 0x82c, 0xFF},\
{_csi_phy_base_ + 0x830, 0xFF},\
{_csi_phy_base_ + 0x834, 0xFB},\
{_csi_phy_base_ + 0x838, 0xFF},\
{_csi_phy_base_ + 0x83c, 0x7F},\
{_csi_phy_base_ + 0x840, 0xFF},\
{_csi_phy_base_ + 0x844, 0xFF},\
{_csi_phy_base_ + 0x848, 0xEF},\
{_csi_phy_base_ + 0x84c, 0xFF},\
{_csi_phy_base_ + 0x850, 0xFF},\
{_csi_phy_base_ + 0x854, 0xFF},


struct camera_hw_reg_array hw_csi2_phy_init_regs[] = {
	CSI_PHY_INIT(CSIPHY1_BASE)
};

void camera_csiphy_set_base(){
	if(CSIPHY_ID == 0){
		csiphybase = CSIPHY0_BASE;
		csiphy_clk_mux_base = CSIPHY0_CLK_MUX_BASE;
	}
	else{
		csiphybase = CSIPHY1_BASE;
		csiphy_clk_mux_base = CSIPHY1_CLK_MUX_BASE;
	}
}

void camera_csiphy_clk_config(int enable){
	clk_get_set_enable("gcc_camss_top_ahb_clk",0,enable);
	clk_get_set_enable("gcc_camss_ispif_ahb_clk",61540000,enable);
	clk_get_set_enable("csi1phytimer_clk_src",200000000,enable);
	clk_get_set_enable("gcc_camss_csi1phytimer_clk",0,enable);
	clk_get_set_enable("camss_top_ahb_clk_src",0,enable);
	clk_get_set_enable("gcc_camss_csi1phy_clk",0,enable);
	clk_get_set_enable("gcc_camss_ahb_clk",0,enable);
}

void camera_csiphy_reset(){
	msm_camera_io_w(0x1,csiphybase + csiphy_3ph_reg.
	mipi_csiphy_3ph_cmn_ctrl0.addr);
	mdelay(7);
	msm_camera_io_w(0x0,csiphybase + csiphy_3ph_reg.
	mipi_csiphy_3ph_cmn_ctrl0.addr);
}

void camera_csiphy_lane_config(struct msm_camera_csiphy_params *csiphy_params){
	uint32_t val = 0;
	if(CSIPHY_HW_VERSION > CSIPHY_VERSION_V30){
		val  = msm_camera_io_r(csiphy_clk_mux_base);
		if((csiphy_params->combo_mode) && ((csiphy_params->lane_mask & 0x18) == 0x18)){
			val &= ~0xf0;
			val |= csiphy_params->csid_core << 4;
		}else{
			val &= ~0xf;
			val |= csiphy_params->csid_core;
		}
		msm_camera_io_w(val, csiphy_clk_mux_base);
	}
	msm_hw_init(&hw_csi2_phy_init_regs[0],
	sizeof(hw_csi2_phy_init_regs) / sizeof(hw_csi2_phy_init_regs[0]));
}

void camera_csiphy_lane_config_start(){
	camera_csiphy_lane_config(&ov8865_csiphy_params);
}

void camera_csiphy_init(){
	camera_csiphy_set_base();
	camera_csiphy_clk_config(1);
	camera_csiphy_reset();
	camera_csiphy_lane_config_start();
}

void camera_csiphy_release(){
	msm_camera_io_w(0x0,csiphybase + csiphy_3ph_reg.mipi_csiphy_3ph_cmn_ctrl5.addr);
	msm_camera_io_w(0x0,csiphybase + csiphy_3ph_reg.mipi_csiphy_3ph_cmn_ctrl6.addr);

}

void camera_csiphy_clock_disable(){
	camera_csiphy_clk_config(0);
}