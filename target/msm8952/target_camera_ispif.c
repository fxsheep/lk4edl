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

#define msm_camera_io_w_mb(d,a) writel(d,a)
#define msm_camera_io_w(d,a)    writel(d,a)
#define msm_camera_io_r(a)      readl(a)
#define msm_camera_io_r_mb(a)   readl(a)

/* common registers */
#define ISPIF_RST_CMD_ADDR                       0x008
#define ISPIF_RST_CMD_1_ADDR                     0x00C
#define ISPIF_IRQ_GLOBAL_CLEAR_CMD_ADDR          0x01C
#define PIX0_LINE_BUF_EN_BIT                     6

#define ISPIF_VFE(m)                             ((m) * 0x200)

#define ISPIF_VFE_m_CTRL_0(m)                    (0x200 + ISPIF_VFE(m))
#define ISPIF_VFE_m_IRQ_MASK_0(m)                (0x208 + ISPIF_VFE(m))
#define ISPIF_VFE_m_IRQ_MASK_1(m)                (0x20C + ISPIF_VFE(m))
#define ISPIF_VFE_m_IRQ_MASK_2(m)                (0x210 + ISPIF_VFE(m))
#define ISPIF_VFE_m_IRQ_STATUS_0(m)              (0x21C + ISPIF_VFE(m))
#define ISPIF_VFE_m_IRQ_STATUS_1(m)              (0x220 + ISPIF_VFE(m))
#define ISPIF_VFE_m_IRQ_STATUS_2(m)              (0x224 + ISPIF_VFE(m))
#define ISPIF_VFE_m_IRQ_CLEAR_0(m)               (0x230 + ISPIF_VFE(m))
#define ISPIF_VFE_m_IRQ_CLEAR_1(m)               (0x234 + ISPIF_VFE(m))
#define ISPIF_VFE_m_IRQ_CLEAR_2(m)               (0x238 + ISPIF_VFE(m))
#define ISPIF_VFE_m_INPUT_SEL(m)                 (0x244 + ISPIF_VFE(m))
#define ISPIF_VFE_m_INTF_CMD_0(m)                (0x248 + ISPIF_VFE(m))
#define ISPIF_VFE_m_INTF_CMD_1(m)                (0x24C + ISPIF_VFE(m))
#define ISPIF_VFE_m_PIX_INTF_n_CID_MASK(m, n)    (0x254 + ISPIF_VFE(m) + 4*(n))
#define ISPIF_VFE_m_RDI_INTF_n_CID_MASK(m, n)    (0x264 + ISPIF_VFE(m) + 4*(n))
#define ISPIF_VFE_m_PIX_INTF_n_CROP(m, n)        (0x278 + ISPIF_VFE(m) + 4*(n))
#define ISPIF_VFE_m_3D_THRESHOLD(m)              (0x288 + ISPIF_VFE(m))
#define ISPIF_VFE_m_OUTPUT_SEL(m)                (0x28C + ISPIF_VFE(m))
#define ISPIF_VFE_m_PIX_OUTPUT_n_MISR(m, n)      (0x290 + ISPIF_VFE(m) + 4*(n))
#define ISPIF_VFE_m_RDI_OUTPUT_n_MISR_0(m, n)    (0x298 + ISPIF_VFE(m) + 8*(n))
#define ISPIF_VFE_m_RDI_OUTPUT_n_MISR_1(m, n)    (0x29C + ISPIF_VFE(m) + 8*(n))
#define ISPIF_VFE_m_PIX_INTF_n_STATUS(m, n)      (0x2C0 + ISPIF_VFE(m) + 4*(n))
#define ISPIF_VFE_m_RDI_INTF_n_STATUS(m, n)      (0x2D0 + ISPIF_VFE(m) + 4*(n))
#define ISPIF_VFE_m_3D_DESKEW_SIZE(m)            (0x2E4 + ISPIF_VFE(m))

/* CSID CLK MUX SEL REGISTERS */
#define ISPIF_RDI_CLK_MUX_SEL_ADDR               0x8

/*ISPIF RESET BITS*/
#define VFE_CLK_DOMAIN_RST                       BIT(31)
#define PIX_1_CLK_DOMAIN_RST                     BIT(30)
#define PIX_CLK_DOMAIN_RST                       BIT(29)
#define RDI_2_CLK_DOMAIN_RST                     BIT(28)
#define RDI_1_CLK_DOMAIN_RST                     BIT(27)
#define RDI_CLK_DOMAIN_RST                       BIT(26)
#define AHB_CLK_DOMAIN_RST                       BIT(25)
#define RDI_2_VFE_RST_STB                        BIT(12)
#define RDI_2_CSID_RST_STB                       BIT(11)
#define RDI_1_VFE_RST_STB                        BIT(10)
#define RDI_1_CSID_RST_STB                       BIT(9)
#define RDI_0_VFE_RST_STB                        BIT(8)
#define RDI_0_CSID_RST_STB                       BIT(7)
#define PIX_1_VFE_RST_STB                        BIT(6)
#define PIX_1_CSID_RST_STB                       BIT(5)
#define PIX_0_VFE_RST_STB                        BIT(4)
#define PIX_0_CSID_RST_STB                       BIT(3)
#define SW_REG_RST_STB                           BIT(2)
#define MISC_LOGIC_RST_STB                       BIT(1)
#define STROBED_RST_EN                           BIT(0)

#define ISPIF_RST_CMD_MASK                       0xFE0F1FFF
#define ISPIF_RST_CMD_1_MASK                     0xFC0F1FF9

#define ISPIF_RST_CMD_MASK_RESTART               0x00001FF9
#define ISPIF_RST_CMD_1_MASK_RESTART             0x00001FF9

#define PIX_INTF_0_OVERFLOW_IRQ                  BIT(12)
#define RAW_INTF_0_OVERFLOW_IRQ                  BIT(25)
#define RAW_INTF_1_OVERFLOW_IRQ                  BIT(25)
#define RAW_INTF_2_OVERFLOW_IRQ                  BIT(12)
#define RESET_DONE_IRQ                           BIT(27)

#define ISPIF_IRQ_STATUS_MASK                    0x0A493249
#define ISPIF_IRQ_STATUS_1_MASK                  0x02493249
#define ISPIF_IRQ_STATUS_2_MASK                  0x00001249

#define ISPIF_IRQ_STATUS_PIX_SOF_MASK            0x249
#define ISPIF_IRQ_STATUS_RDI0_SOF_MASK           0x492000
#define ISPIF_IRQ_STATUS_RDI1_SOF_MASK           0x492000
#define ISPIF_IRQ_STATUS_RDI2_SOF_MASK           0x249

#define ISPIF_IRQ_GLOBAL_CLEAR_CMD               0x1

#define ISPIF_STOP_INTF_IMMEDIATELY              0xAAAAAAAA

#define VFE0 0
#define VFE1 1
#define ISPIF_BASE    0x1b31000
#define ISPIF_CLK_MUX 0x1b00020

int msm_ispif_read_irq_status(){
	uint32_t VFE0_ispifIrqStatus0=0,VFE1_ispifIrqStatus0=0;
	uint32_t count = 0;

	VFE0_ispifIrqStatus0 = msm_camera_io_r(ISPIF_BASE + ISPIF_VFE_m_IRQ_STATUS_0(VFE0));
	VFE1_ispifIrqStatus0 = msm_camera_io_r(ISPIF_BASE + ISPIF_VFE_m_IRQ_STATUS_0(VFE1));
	dprintf(CRITICAL,"msm_ispif_read_irq_status : VFE0_ispifIrqStatus0=0x%x VFE1_ispifIrqStatus0=0x%x\n",VFE0_ispifIrqStatus0,VFE1_ispifIrqStatus0);

	while(!(VFE0_ispifIrqStatus0 & RESET_DONE_IRQ)){
		VFE0_ispifIrqStatus0 = msm_camera_io_r(ISPIF_BASE + ISPIF_VFE_m_IRQ_STATUS_0(VFE0));
		dprintf(CRITICAL,"msm_ispif_read_irq_status : VFE0_ispifIrqStatus0=0x%x\n",VFE0_ispifIrqStatus0);
		count ++;
		if(count > 20){
			dprintf(CRITICAL,"msm_ispif_read_irq_status: wait reset VFE0 irq timeount\n");
			return -1;
		}
	}
	msm_camera_io_w(VFE0_ispifIrqStatus0,ISPIF_BASE + ISPIF_VFE_m_IRQ_CLEAR_0(VFE0));


	while(!(VFE1_ispifIrqStatus0 & RESET_DONE_IRQ)){
		VFE1_ispifIrqStatus0 = msm_camera_io_r(ISPIF_BASE + ISPIF_VFE_m_IRQ_STATUS_0(VFE1));
		dprintf(CRITICAL,"msm_ispif_read_irq_status : VFE1_ispifIrqStatus0=0x%x\n",VFE1_ispifIrqStatus0);
		count ++;
		if(count > 20){
			dprintf(CRITICAL,"msm_ispif_read_irq_status: wait reset VFE1 irq timeount\n");
			return -1;
		}
	}
	msm_camera_io_w(VFE1_ispifIrqStatus0,ISPIF_BASE + ISPIF_VFE_m_IRQ_CLEAR_0(VFE1));

	msm_camera_io_w_mb(ISPIF_IRQ_GLOBAL_CLEAR_CMD, ISPIF_BASE + ISPIF_IRQ_GLOBAL_CLEAR_CMD_ADDR);

	return 0;
}

void msm_isp_clk_enable(int enable){
	clk_get_set_enable("vfe1_clk_src",266670000,enable);
    clk_get_set_enable("gcc_camss_vfe1_clk",0,enable);
    clk_get_set_enable("gcc_camss_csi_vfe1_clk",0,enable);
}

void msm_ispif_cfg(){
	int i;
	uint32_t data = 0;
	uint16_t cids_mask = 0;
	uint32_t intf_addr;

	msm_camera_io_w(0x0, ISPIF_BASE + ISPIF_VFE_m_IRQ_MASK_0(1));
	msm_camera_io_w(0x0, ISPIF_BASE + ISPIF_VFE_m_IRQ_MASK_1(1));
	msm_camera_io_w_mb(0x0, ISPIF_BASE +ISPIF_VFE_m_IRQ_MASK_2(1));

    //msm_ispif_select_clk_mux
	data = msm_camera_io_r(ISPIF_CLK_MUX);
	data &= ~(0xf << (1 * 8));
	data |= (1 << (1 * 8));
	dprintf(CRITICAL,"msm_ispif_select_clk_mux data=0x%x\n",data);
	msm_camera_io_w(data, ISPIF_CLK_MUX);

    //msm_ispif_sel_csid_core
	data = msm_camera_io_r(ISPIF_BASE + ISPIF_VFE_m_INPUT_SEL(1));
	data &= ~(BIT(1) | BIT(0));
	data |= (uint32_t) 1;
	msm_camera_io_w_mb(data, ISPIF_BASE + ISPIF_VFE_m_INPUT_SEL(1));

	//msm_ispif_get_cids_mask_from_cfg
	cids_mask = 0x1;

    //msm_ispif_enable_intf_cids
	intf_addr = ISPIF_VFE_m_PIX_INTF_n_CID_MASK(1, 0);
	data = msm_camera_io_r(ISPIF_BASE + intf_addr);
	data |=  (uint32_t) cids_mask;
	msm_camera_io_w_mb(data, ISPIF_BASE + intf_addr);

	for (i = 0; i < 2; i++) {
		msm_camera_io_w(ISPIF_IRQ_STATUS_MASK, ISPIF_BASE + ISPIF_VFE_m_IRQ_MASK_0(i));

		msm_camera_io_w(ISPIF_IRQ_STATUS_MASK, ISPIF_BASE + ISPIF_VFE_m_IRQ_CLEAR_0(i));

		msm_camera_io_w(ISPIF_IRQ_STATUS_1_MASK, ISPIF_BASE + ISPIF_VFE_m_IRQ_MASK_1(i));

		msm_camera_io_w(ISPIF_IRQ_STATUS_1_MASK, ISPIF_BASE + ISPIF_VFE_m_IRQ_CLEAR_1(i));

		msm_camera_io_w(ISPIF_IRQ_STATUS_2_MASK, ISPIF_BASE + ISPIF_VFE_m_IRQ_MASK_2(i));

		msm_camera_io_w(ISPIF_IRQ_STATUS_2_MASK, ISPIF_BASE + ISPIF_VFE_m_IRQ_CLEAR_2(i));
	}

	msm_camera_io_w_mb(ISPIF_IRQ_GLOBAL_CLEAR_CMD, ISPIF_BASE + ISPIF_IRQ_GLOBAL_CLEAR_CMD_ADDR);

	uint32_t intf_cmd = 0xFFFFFFFF;
	intf_cmd &= ~(0x3 << (0 * 2 + 0* 8));
	intf_cmd |= (0x01 << (0 * 2 + 0 * 8));
	msm_camera_io_w_mb(intf_cmd,ISPIF_BASE + ISPIF_VFE_m_INTF_CMD_0(1));
}

int msm_ispif_init(){
	int rc = 0;
	int i;

	msm_isp_clk_enable(1);
	msm_camera_io_w(ISPIF_RST_CMD_MASK,ISPIF_BASE + ISPIF_RST_CMD_ADDR);
	rc = msm_ispif_read_irq_status();
	if(rc < 0){
		dprintf(CRITICAL,"msm_ispif_init rest ispif mode failed\n");
		return -1;
	}

	msm_camera_io_w(ISPIF_RST_CMD_1_MASK,ISPIF_BASE + ISPIF_RST_CMD_1_ADDR);
	rc = msm_ispif_read_irq_status();
	if(rc < 0){
		dprintf(CRITICAL,"msm_ispif_init rest ispif mode failed\n");
		return -1;
	}

	//msm_ispif_reset
	for (i = 0; i < 2; i++){
		msm_camera_io_w(1 << PIX0_LINE_BUF_EN_BIT,ISPIF_BASE + ISPIF_VFE_m_CTRL_0(i));
		msm_camera_io_w(0, ISPIF_BASE + ISPIF_VFE_m_IRQ_MASK_0(i));
		msm_camera_io_w(0, ISPIF_BASE+ ISPIF_VFE_m_IRQ_MASK_1(i));
		msm_camera_io_w(0, ISPIF_BASE + ISPIF_VFE_m_IRQ_MASK_2(i));
		msm_camera_io_w(0xFFFFFFFF, ISPIF_BASE + ISPIF_VFE_m_IRQ_CLEAR_0(i));
		msm_camera_io_w(0xFFFFFFFF, ISPIF_BASE + ISPIF_VFE_m_IRQ_CLEAR_1(i));
		msm_camera_io_w(0xFFFFFFFF, ISPIF_BASE + ISPIF_VFE_m_IRQ_CLEAR_2(i));

		msm_camera_io_w(0, ISPIF_BASE + ISPIF_VFE_m_INPUT_SEL(i));

		msm_camera_io_w(ISPIF_STOP_INTF_IMMEDIATELY, ISPIF_BASE + ISPIF_VFE_m_INTF_CMD_0(i));
		msm_camera_io_w(ISPIF_STOP_INTF_IMMEDIATELY, ISPIF_BASE + ISPIF_VFE_m_INTF_CMD_1(i));
		msm_camera_io_w(0, ISPIF_BASE + ISPIF_VFE_m_PIX_INTF_n_CID_MASK(i, 0));
		msm_camera_io_w(0, ISPIF_BASE + ISPIF_VFE_m_PIX_INTF_n_CID_MASK(i, 1));
		msm_camera_io_w(0, ISPIF_BASE + ISPIF_VFE_m_RDI_INTF_n_CID_MASK(i, 0));
		msm_camera_io_w(0, ISPIF_BASE + ISPIF_VFE_m_RDI_INTF_n_CID_MASK(i, 1));
		msm_camera_io_w(0, ISPIF_BASE + ISPIF_VFE_m_RDI_INTF_n_CID_MASK(i, 2));

		msm_camera_io_w(0, ISPIF_BASE + ISPIF_VFE_m_PIX_INTF_n_CROP(i, 0));
		msm_camera_io_w(0, ISPIF_BASE + ISPIF_VFE_m_PIX_INTF_n_CROP(i, 1));
	}
	msm_camera_io_w_mb(ISPIF_IRQ_GLOBAL_CLEAR_CMD, ISPIF_BASE + ISPIF_IRQ_GLOBAL_CLEAR_CMD_ADDR);
	return 1;
}

void msm_ispif_release(){
	uint32_t intf_cmd = 0xFFFFFFFF;
	intf_cmd &= ~(0x3 << (0 * 2 + 0* 8));
	intf_cmd |= (0x02 << (0 * 2 + 0 * 8));
	msm_camera_io_w_mb(intf_cmd,ISPIF_BASE + ISPIF_VFE_m_INTF_CMD_0(1));

	uint32_t intf_addr = ISPIF_VFE_m_PIX_INTF_n_CID_MASK(1, 0);
	uint32_t data = msm_camera_io_r(ISPIF_BASE + intf_addr);
    data &= ~((uint32_t) 0x1);
    msm_camera_io_w_mb(data, ISPIF_BASE + intf_addr);
}

void msm_ispif_clock_disable(){
	msm_isp_clk_enable(0);
}


