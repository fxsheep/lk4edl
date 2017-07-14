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
#include <platform.h>

#define msm_camera_io_w_mb(d,a) writel(d,a)
#define msm_camera_io_w(d,a)    writel(d,a)
#define msm_camera_io_r(a)      readl(a)
#define msm_camera_io_r_mb(a)   readl(a)
#define msm_camera_io_r_ab(a)   readb(a)
#define msm_camera_io_w_ab(d,a)   writeb(d,a)

#define VFE_BASE       0x1b14000
#define VFE_VBIF_BASE  0x1ba0000

#define VFE40_WM_BASE(idx) (0x6C + 0x24 * idx)
#define VFE40_XBAR_BASE(idx) (0x58 + 0x4 * (idx / 2))
#define VFE40_XBAR_SHIFT(idx) ((idx%2) ? 16 : 0)
#define VFE40_PING_PONG_BASE(wm, ping_pong) \
        (VFE40_WM_BASE(wm) + 0x4 * (1 + ((~ping_pong) & 0x1)))
#define CAL_WORD(width, M, N) ((width * M + N - 1) / N)
#define SENSOR_WIDTH  3264
#define SENSOR_HEIGHT 2448

#define PWR_ON_MASK		BIT(31)
#define EN_REST_WAIT_MASK	(0xF << 20)
#define EN_FEW_WAIT_MASK	(0xF << 16)
#define CLK_DIS_WAIT_MASK	(0xF << 12)
#define SW_OVERRIDE_MASK	BIT(2)
#define HW_CONTROL_MASK		BIT(1)
#define SW_COLLAPSE_MASK	BIT(0)
#define GMEM_CLAMP_IO_MASK	BIT(0)
#define GMEM_RESET_MASK		BIT(4)
#define BCR_BLK_ARES_BIT	BIT(0)

#define EN_REST_WAIT_VAL	(0x2 << 20)
#define EN_FEW_WAIT_VAL		(0x8 << 16)
#define CLK_DIS_WAIT_VAL	(0x2 << 12)

#define VFE40_STATS_BASE(idx) (0x168 + 0x18 * idx)

#define VFE_DDR_PING_BASE  0xA400102f
#define VFE_DDR_PONG_BASE  0xA400102f//(0xA400102f + 1920*1080*3/2)

#define VFE_FRAME_SIZE     (1920*1080*3/2)

#define VFE_DDR_STATS_PING_BASE (0xA400102f + 1920*1080*3/2)
#define VFE_DDR_STATS_PONG_BASE  (0xA400102f + 1920*1080*3)

#define VFE_PING_FLAG 0xFFFFFFFF
#define VFE_PONG_FLAG 0x0

enum ISP_START_PIXEL_PATTERN {
	ISP_BAYER_RGRGRG,
	ISP_BAYER_GRGRGR,
	ISP_BAYER_BGBGBG,
	ISP_BAYER_GBGBGB,
	ISP_YUV_YCbYCr,
	ISP_YUV_YCrYCb,
	ISP_YUV_CbYCrY,
	ISP_YUV_CrYCbY,
	ISP_PIX_PATTERN_MAX
};

enum msm_vfe_plane_fmt {
	Y_PLANE,
	CB_PLANE,
	CR_PLANE,
	CRCB_PLANE,
	CBCR_PLANE,
	VFE_PLANE_FMT_MAX
};

enum msm_isp_irq_operation {
	/* enable the irq bits in given parameters */
	MSM_ISP_IRQ_ENABLE = 1,
	/* disable the irq bits in the given parameters */
	MSM_ISP_IRQ_DISABLE = 2,
	/* set the irq bits to the given parameters */
	MSM_ISP_IRQ_SET = 3,
};

enum earlycamera_buffer_status {
  MSM_BUFFER_EMPTY = 1,
  MSM_BUFFER_SET,
  MSM_BUFFER_DATA_FULL,
  MSM_BUFFER_PING_SET,
  MSM_BUFFER_PONG_SET,
  MSM_BUFFER_BUSY,
  MSM_BUFFER_IDLE,
};

struct msm_isp_buffer_info{
	uint32_t buf_idx;
    uint32_t status;
    uint32_t addr;
};

struct msm_camera_bufq{
    int num_bufs;
    struct msm_isp_buffer_info buffer_info[4];
};

struct msm_camera_bufq vfe_bufq;

void msm_camera_io_dump(uint32_t base,int size);
void vfe_send_hw_cfg();

void msm_vfe_bufq_init(){
	int i;
	vfe_bufq.num_bufs = 4;
	for(i = 0;i < vfe_bufq.num_bufs; i++){
        vfe_bufq.buffer_info[i].buf_idx = i;
        vfe_bufq.buffer_info[i].status = MSM_BUFFER_EMPTY;
        vfe_bufq.buffer_info[i].addr = VFE_DDR_PING_BASE + VFE_FRAME_SIZE*i;
	}
}

int get_msm_vfe_buffer_idle(){
	int i;
	int ret = 0;
	for(i = 0; i < vfe_bufq.num_bufs; i++){
        if(vfe_bufq.buffer_info[i].status == MSM_BUFFER_EMPTY){
            ret = i;
            break;
        }
    }
    return ret;
}

int msm_vfe_gds_enable(){
	uint32_t val,regval;
	int count  = 100;
	regval = msm_camera_io_r(0x185806c);
	regval &= ~(HW_CONTROL_MASK | SW_OVERRIDE_MASK);
	regval &= ~(EN_REST_WAIT_MASK | EN_FEW_WAIT_MASK | CLK_DIS_WAIT_MASK);
	regval |= EN_REST_WAIT_VAL | EN_FEW_WAIT_VAL | CLK_DIS_WAIT_VAL;
	msm_camera_io_w(regval, 0x185806c);
	regval &= ~SW_COLLAPSE_MASK;
	msm_camera_io_w(regval, 0x185806c);

	for(;count > 0; count--){
		val = msm_camera_io_r(0x185806c);
		val &= PWR_ON_MASK;
		dprintf(CRITICAL,"msm_vfe_gds_enable: val=0x%x\n",val);
		if(val)
			return 0;
	}
	return -1;
}

int msm_vfe_gds_disable(){
	uint32_t val,regval;
	int count  = 100;

	regval = msm_camera_io_r(0x185806c);
	regval |= SW_COLLAPSE_MASK;
	msm_camera_io_w(regval, 0x185806c);

	for (; count > 0; count--) {
		val = msm_camera_io_r(0x185806c);
		val &= PWR_ON_MASK;
		dprintf(CRITICAL,"msm_vfe_gds_disable: val=0x%x\n",val);
		if (!val)
			return 1;
	}
	return -1;
}

void msm_camera_clk_enable(int enable){

	clk_get_set_enable("vfe1_clk_src",266670000,enable);
    clk_get_set_enable("gcc_camss_vfe1_clk",0,enable);
    clk_get_set_enable("gcc_camss_csi_vfe1_clk",0,enable);

	clk_get_set_enable("gcc_camss_vfe1_ahb_clk",0,enable);
	clk_get_set_enable("gcc_camss_vfe1_axi_clk",0,enable);

	clk_get_set_enable("gcc_camss_micro_ahb_clk",0,enable);

	clk_get_set_enable("gcc_camss_ispif_ahb_clk",0,enable);
}

void msm_vfe40_config_irq(struct vfe_device *vfe_dev,
		uint32_t irq0_mask, uint32_t irq1_mask,
		enum msm_isp_irq_operation oper)
{
	switch (oper) {
	case MSM_ISP_IRQ_ENABLE:
		vfe_dev->irq0_mask |= irq0_mask;
		vfe_dev->irq1_mask |= irq1_mask;
		break;
	case MSM_ISP_IRQ_DISABLE:
		vfe_dev->irq0_mask &= ~irq0_mask;
		vfe_dev->irq1_mask &= ~irq1_mask;
		break;
	case MSM_ISP_IRQ_SET:
		vfe_dev->irq0_mask = irq0_mask;
		vfe_dev->irq1_mask = irq1_mask;
	}
	msm_camera_io_w_mb(vfe_dev->irq0_mask, VFE_BASE + 0x28);
	msm_camera_io_w_mb(vfe_dev->irq1_mask, VFE_BASE + 0x2C);
}

void msm_vfe40_clear_status_reg(struct vfe_device *vfe_dev){
	vfe_dev->irq0_mask = (1 << 31);
	vfe_dev->irq1_mask = 0;

	msm_vfe40_config_irq(vfe_dev, (1 << 31), 0,
			MSM_ISP_IRQ_SET);

	msm_camera_io_w(0xFFFFFFFF, VFE_BASE + 0x30);
	msm_camera_io_w_mb(0xFFFFFFFF, VFE_BASE + 0x34);
	msm_camera_io_w_mb(0x1, VFE_BASE + 0x24);
}

void msm_vfe40_read_irq_status(struct vfe_device *vfe_dev,
	uint32_t *irq_status0, uint32_t *irq_status1){
	*irq_status0 = msm_camera_io_r(VFE_BASE + 0x38);
	*irq_status1 = msm_camera_io_r(VFE_BASE + 0x3C);

	dprintf(CRITICAL,"msm_vfe40_read_irq_status: irq_status0 = 0X%x,irq_status1 = 0x%x\n",*irq_status0,*irq_status1);
	if (*irq_status0 & 0x6000000)
		*irq_status0 &= ~(0x18000000);

	msm_camera_io_w(*irq_status0, VFE_BASE + 0x30);
	msm_camera_io_w(*irq_status1, VFE_BASE + 0x34);
	msm_camera_io_w_mb(1, VFE_BASE + 0x24);

	if (*irq_status0 & 0x18000000) {
		dprintf(CRITICAL,"msm_vfe40_read_irq_status: Protection triggered\n");
		*irq_status0 &= ~(0x18000000);
	}
	*irq_status0 &= vfe_dev->irq0_mask;
	*irq_status1 &= vfe_dev->irq1_mask;
}

int msm_vfe40_reset_hardware(struct vfe_device *vfe_dev,
	uint32_t first_start, uint32_t blocking_call)
{
	uint32_t irq_status0, irq_status1;
	uint32_t count = 0;

	if (first_start) {
		msm_camera_io_w_mb(0x1FF, VFE_BASE + 0xC);
	} else {
		msm_camera_io_w_mb(0x1EF, VFE_BASE + 0xC);
		msm_camera_io_w(0x7FFFFFFF, VFE_BASE + 0x30);
		msm_camera_io_w(0xFEFFFEFF, VFE_BASE + 0x34);
		msm_camera_io_w(0x1, VFE_BASE + 0x24);
		msm_camera_io_w_mb(0x0001FFFF, VFE_BASE + 0x4C);
	}

	if (blocking_call) {
		msm_vfe40_read_irq_status(vfe_dev,&irq_status0,&irq_status1);
	    while(!(irq_status0 & (1 << 31))){
	    	msm_vfe40_read_irq_status(vfe_dev,&irq_status0,&irq_status1);
	    	count++;
	    	if(count > 20){
	    		dprintf(CRITICAL,"msm_vfe40_reset_hardware: read_irq_status timeout\n");
	    		return -1;
	    	}
	    }
	}
	return 1;
}

void msm_vfe40_get_error_mask(uint32_t *error_mask0, uint32_t *error_mask1)
{
	*error_mask0 = 0x00000000;
	*error_mask1 = 0x00FFFEFF;
}

void msm_vfe40_init_hardware_reg(struct vfe_device *vfe_dev){

	msm_camera_io_w(0xaa55aa55,VFE_BASE + 0x2c4);
	msm_camera_io_w(0xaa55aa55,VFE_BASE + 0x2c8);
	msm_camera_io_w(0xaa55aa55,VFE_BASE + 0x2cc);
	msm_camera_io_w(0xaa55aa55,VFE_BASE + 0x2d0);
	msm_camera_io_w(0xaa55aa55,VFE_BASE + 0x2d4);
	msm_camera_io_w(0xaa55aa55,VFE_BASE + 0x2d8);
	msm_camera_io_w(0xaa55aa55,VFE_BASE + 0x2dc);
	msm_camera_io_w(0x0001aa55,VFE_BASE + 0x2e0);

	msm_camera_io_w(0xcccc1111,VFE_BASE + 0x988);
	msm_camera_io_w(0xcccc1111,VFE_BASE + 0x98c);
	msm_camera_io_w(0xcccc1111,VFE_BASE + 0x990);
	msm_camera_io_w(0xcccc1111,VFE_BASE + 0x994);
	msm_camera_io_w(0xcccc1111,VFE_BASE + 0x998);
	msm_camera_io_w(0xcccc1111,VFE_BASE + 0x99c);
	msm_camera_io_w(0xcccc1111,VFE_BASE + 0x9a0);
	msm_camera_io_w(0xcccc1111,VFE_BASE + 0x9a4);
	msm_camera_io_w(0xcccc1111,VFE_BASE + 0x9a8);
	msm_camera_io_w(0xcccc1111,VFE_BASE + 0x9ac);
	msm_camera_io_w(0xcccc1111,VFE_BASE + 0x9b0);
	msm_camera_io_w(0xcccc1111,VFE_BASE + 0x9b4);
	msm_camera_io_w(0xcccc1111,VFE_BASE + 0x9b8);
	msm_camera_io_w(0xcccc1111,VFE_BASE + 0x9bc);
	msm_camera_io_w(0xcccc1111,VFE_BASE + 0x9c0);
	msm_camera_io_w(0xcccc1111,VFE_BASE + 0x9c4);
	msm_camera_io_w(0x00000110,VFE_BASE + 0x9c8);

	msm_camera_io_w(0x3,VFE_VBIF_BASE + 0x124);

	/* BUS_CFG */
	msm_camera_io_w(0x10000001, VFE_BASE + 0x50);
	msm_vfe40_config_irq(vfe_dev, 0x800000E0, 0xFEFFFF7E,MSM_ISP_IRQ_ENABLE);
	msm_camera_io_w(0xFFFFFFFF, VFE_BASE + 0x30);
	msm_camera_io_w_mb(0xFEFFFFFF, VFE_BASE + 0x34);
	msm_camera_io_w(1, VFE_BASE + 0x24);

	msm_camera_io_w(0, VFE_BASE + 0x30);
	msm_camera_io_w_mb(0, VFE_BASE + 0x34);
	msm_camera_io_w(1, VFE_BASE + 0x24);
}

uint32_t msm_vfe_poll_irq0(){
	uint32_t irq0;
	irq0 = msm_camera_io_r(VFE_BASE + 0x38);
	return irq0;
}

uint32_t msm_vfe_poll_irq1(){
	uint32_t irq1;
	irq1 = msm_camera_io_r(VFE_BASE + 0x3c);
	return irq1;
}

uint32_t msm_vfe_pingpong_status(){
	uint32_t ping_pong_status;
	ping_pong_status = msm_camera_io_r(VFE_BASE + 0x268);
	return ping_pong_status;
}

void msm_vfe_irq_mask_cfg(uint32_t irq0,uint32_t irq1){
	msm_camera_io_w(irq0, VFE_BASE + 0x30);
	msm_camera_io_w(irq1, VFE_BASE + 0x34);
	msm_camera_io_w_mb(1, VFE_BASE + 0x24);
}

void msm_vfe_update_pingpong_addr(uint32_t addr,int pingping_status){
	if(pingping_status){
	    msm_camera_io_w(addr,VFE_BASE + 0x70); //ping 0
        msm_camera_io_w(addr + 1080*1920,VFE_BASE + 0x94); //ping 1
    }else{
        msm_camera_io_w(addr,VFE_BASE + 0x74); //pong 0
        msm_camera_io_w(addr + 1080*1920,VFE_BASE + 0x98); //pong 1
    }
}

int get_msm_vfe_buffer_full(){
	int i;
	for(i = 0; i < vfe_bufq.num_bufs; i++){
        if(vfe_bufq.buffer_info[i].status == MSM_BUFFER_DATA_FULL){
            return i;
        }
	}
	return -1;
}

int msm_vfe_check_buffer_status(int pingping_status){
	uint32_t temp;
    int i;
	if(pingping_status){
		temp = MSM_BUFFER_PING_SET;
	}else{
		temp = MSM_BUFFER_PONG_SET;
	}

	for(i = 0; i < vfe_bufq.num_bufs; i++){
        if(vfe_bufq.buffer_info[i].status == MSM_BUFFER_DATA_FULL){
            vfe_bufq.buffer_info[i].status = MSM_BUFFER_EMPTY;
        }
    }
    for(i = 0; i < vfe_bufq.num_bufs; i++){
        if(vfe_bufq.buffer_info[i].status == temp){
            vfe_bufq.buffer_info[i].status = MSM_BUFFER_DATA_FULL;
            break;
        }
    }

    for(i = 0; i < vfe_bufq.num_bufs; i++){
        if(vfe_bufq.buffer_info[i].status == MSM_BUFFER_EMPTY){
            msm_vfe_update_pingpong_addr(vfe_bufq.buffer_info[i].addr,pingping_status);
            vfe_bufq.buffer_info[i].status = temp;
            break;
        }
    }
    return 1;
}

void msm_get_camera_frame(){
	uint32_t pingpong;
	pingpong = msm_vfe_pingpong_status();
	msm_vfe_check_buffer_status(pingpong & 0x1);
}

#define FRAME_WIDTH 1920
#define FRAME_HEIGHT 1080
#define RGB_ADDR (0xA400102f + 1920*1080*6)

static long int yuv_rgb_crv_tab[256];
static long int yuv_rgb_cbu_tab[256];
static long int yuv_rgb_cgu_tab[256];
static long int yuv_rgb_cgv_tab[256];
static long int yuv_rgb_tab_cgy[256];
static unsigned char yuv_rgb_clp[1024];

void init_yuv420p_table(){
	long int crv,cbu,cgu,cgv;
    int i,ind;
    static int init = 0;

    if (init == 1) return;

    crv = 104597; cbu = 132201;
    cgu = 25675;  cgv = 53279;

    for (i = 0; i < 256; i++)
    {
        yuv_rgb_crv_tab[i] = (i-128) * crv;
        yuv_rgb_cbu_tab[i] = (i-128) * cbu;
        yuv_rgb_cgu_tab[i] = (i-128) * cgu;
        yuv_rgb_cgv_tab[i] = (i-128) * cgv;
        yuv_rgb_tab_cgy[i] = 76309*(i-16);
    }

    for (i = 0; i < 384; i++)
        yuv_rgb_clp[i] = 0;
    ind = 384;
    for (i = 0;i < 256; i++)
       yuv_rgb_clp[ind++] = i;
    ind = 640;
    for (i = 0;i < 384; i++)
       yuv_rgb_clp[ind++] = 255;
    init = 1;
}

void nv12_to_rgb888()
{
	static int yuv_line = -1;
    static unsigned char* y_addr = NULL;
    static unsigned char* uv_addr = NULL;
    static unsigned char* rgb_addr = NULL;
    static int idx = -1;
    static int flag = 0;
    int i = 0, j = 0;
    int c1, c2, c3, c4, y1,y2;
    unsigned char tmp,u,v;
    int rgb_stride = 3 * FRAME_HEIGHT;

    init_yuv420p_table();

    if(yuv_line < 0) {
        idx = get_msm_vfe_buffer_full();
        if(idx < 0)
            return;
        vfe_bufq.buffer_info[idx].status = MSM_BUFFER_BUSY;
        y_addr =  (unsigned char*)vfe_bufq.buffer_info[idx].addr;
        uv_addr = y_addr + FRAME_WIDTH*FRAME_HEIGHT;
        flag = !flag;
    }

    for(i = 0; i < 500 && yuv_line < FRAME_HEIGHT; i += 2, yuv_line += 2) {
        rgb_addr = (unsigned char*)(RGB_ADDR + 3 * (FRAME_HEIGHT - yuv_line - 1));
        for(j = 0; j < FRAME_WIDTH; j += 2) {
            v = *uv_addr++;
            u = *uv_addr++;
            c1 = yuv_rgb_crv_tab[v];
            c2 = yuv_rgb_cgu_tab[u];
            c3 = yuv_rgb_cgv_tab[v];
            c4 = yuv_rgb_cbu_tab[u];
            if(flag) {
                tmp = 255 - *y_addr > 60 ? *y_addr + 60 : 255;
                y1 = yuv_rgb_tab_cgy[tmp];
                *(rgb_addr - 3) = *(rgb_addr) = yuv_rgb_clp[384+((y1 + c1)>>16)];
                *(rgb_addr - 2) = *(rgb_addr + 1) = yuv_rgb_clp[384+((y1 - c2 - c3)>>16)];
                *(rgb_addr - 1) = *(rgb_addr + 2) = yuv_rgb_clp[384+((y1 + c4)>>16)];
            }
            rgb_addr += rgb_stride;
            y_addr += 2;
            if(!flag) {
                tmp = 255 - *(y_addr + FRAME_WIDTH) > 60 ? *(y_addr + FRAME_WIDTH) + 60 : 255;
                y2 = yuv_rgb_tab_cgy[tmp];
                *(rgb_addr - 3) = *(rgb_addr) = yuv_rgb_clp[384+((y2 + c1)>>16)];
                *(rgb_addr - 2) = *(rgb_addr + 1) = yuv_rgb_clp[384+((y2 - c2 - c3)>>16)];
                *(rgb_addr - 1) = *(rgb_addr + 2) = yuv_rgb_clp[384+((y2 + c4)>>16)];
            }
            rgb_addr += rgb_stride;
        }
        y_addr += FRAME_WIDTH;
    }

    if(yuv_line >= FRAME_HEIGHT) {
        yuv_line = -1;
        display_camera_on_screen(RGB_ADDR, flag);
        vfe_bufq.buffer_info[idx].status = MSM_BUFFER_EMPTY;
    }
}

void msm_isp_cfg_io_format(){
	uint32_t io_format_reg = 0;
	int bpp_reg = 0;
	io_format_reg = msm_camera_io_r(VFE_BASE + 0x54);

	//bpp = 10;
	bpp_reg = 1 << 0;

	io_format_reg &= 0xFFFFCFFF;
	io_format_reg |= bpp_reg << 12;
	msm_camera_io_w(io_format_reg, VFE_BASE + 0x54);
}

void msm_isp_stats_bg_stream_cfg(struct vfe_device *vfe_dev){
	uint32_t module_cfg;
	uint32_t stats_base = VFE40_STATS_BASE(1);

	msm_camera_io_w(0x8b40007f,VFE_BASE + stats_base + 0xC);

	module_cfg = msm_camera_io_r(VFE_BASE + 0x974);
	module_cfg |= 0x200;
	msm_camera_io_w(module_cfg, VFE_BASE + 0x974);

	module_cfg = msm_camera_io_r(VFE_BASE + 0x18);
	module_cfg |= 0x40;
	msm_camera_io_w(module_cfg, VFE_BASE + 0x18);

	msm_vfe40_config_irq(vfe_dev,1 << (0 + 29), 0,MSM_ISP_IRQ_ENABLE);
	msm_camera_io_w(0x20000, VFE_BASE + 0x44);
	//msm_vfe40_config_irq(vfe_dev,1 << (1 + 29), 0,MSM_ISP_IRQ_ENABLE);
	//msm_camera_io_w(0x20000, VFE_BASE + 0x44);

    msm_camera_io_w(VFE_DDR_STATS_PING_BASE,VFE_BASE + 0x180);
    msm_camera_io_w(VFE_DDR_STATS_PONG_BASE,VFE_BASE + 0x184);

    msm_camera_io_w(0,VFE_BASE + stats_base + 0x8);
    msm_camera_io_w(1,VFE_BASE + stats_base + 0x10);
    msm_camera_io_w(0xFFFFFFFF,VFE_BASE + stats_base + 0x14);
}

void msm_isp_cfg_input(){
	//int rc = 0;
	//rc = msm_isp_cfg_pix(vfe_dev, input_cfg);
	uint32_t core_cfg = 0;
	uint32_t val = 0;
	uint32_t update_mask = 0;

	core_cfg =  msm_camera_io_r(VFE_BASE + 0x1C);
	core_cfg &= 0xFFFCFFFF;

	core_cfg |= 0x0 << 16;
	msm_camera_io_w_mb(core_cfg, VFE_BASE + 0x1C);

	//msm_vfe40_cfg_camif
	msm_camera_io_w(0 << 16 | ISP_BAYER_BGBGBG,VFE_BASE + 0x1C);

	msm_camera_io_w(2448 << 16 | 3264, VFE_BASE + 0x300);

	msm_camera_io_w(0 << 16 | 3263,VFE_BASE + 0x304);
	msm_camera_io_w(0 << 16 | 2447,VFE_BASE + 0x308);

	msm_camera_io_w(0xFFFFFFFF, VFE_BASE + 0x314);

	val = msm_camera_io_r(VFE_BASE + 0x2E8);
	val |= 3;
	msm_camera_io_w(val, VFE_BASE + 0x2E8);

	update_mask = 1;
	msm_camera_io_w_mb(update_mask,VFE_BASE + 0x378);
}

void msm_vfe40_cfg_axi_ub_equal_default(){
	msm_camera_io_w(0x2e4,VFE_BASE + VFE40_WM_BASE(0) + 0x10); //0xb3
	msm_camera_io_w(0x2e50191,VFE_BASE + VFE40_WM_BASE(1) + 0x10); //0xb40079
}

void msm_isp_cfg_axi_stream(struct vfe_device *vfe_dev){
	//msm_isp_axi_update_cgc_override
	int i;
	uint32_t val = 0;
	uint32_t comp_mask, comp_mask_index;
	uint8_t stream_composite_mask = 0;

	//msm_vfe40_axi_update_cgc_override
	for(i = 0;i < 2;i++){
		val = msm_camera_io_r(VFE_BASE + 0x974);
		val |= (1 << i); //0 1 2 3
		msm_camera_io_w_mb(val, VFE_BASE + 0x974);
	}

	//msm_isp_start_axi_stream
	for (i = 0; i < 2; i++){
		stream_composite_mask |= 1 << i;
	}
	comp_mask_index = 0;
	comp_mask = msm_camera_io_r(VFE_BASE + 0x40);
	comp_mask &= ~(0x7F << (comp_mask_index * 8));
	comp_mask |= stream_composite_mask << (comp_mask_index * 8);
	msm_camera_io_w(comp_mask, VFE_BASE + 0x40);
	msm_vfe40_config_irq(vfe_dev, 1 << (comp_mask_index + 25), 0, MSM_ISP_IRQ_ENABLE);


	val = 0;
	stream_composite_mask = 0;
	val = msm_camera_io_r(VFE_BASE + VFE40_WM_BASE(0));
	val |= 0x1;
	msm_camera_io_w_mb(val,VFE_BASE + VFE40_WM_BASE(0));

	val = msm_camera_io_r(VFE_BASE + VFE40_WM_BASE(1));
	val |= 0x1;
	msm_camera_io_w_mb(val,VFE_BASE + VFE40_WM_BASE(1));

	msm_camera_io_w_mb(0x1,VFE_BASE + 0x378);

	//reload_wm
	msm_camera_io_w_mb(0x3, VFE_BASE + 0x4C);

	//update_camif_state
	msm_camera_io_w(0x0, VFE_BASE + 0x30);
	msm_camera_io_w_mb(0x81, VFE_BASE + 0x34);
	msm_camera_io_w_mb(0x1, VFE_BASE + 0x24);
	msm_vfe40_config_irq(vfe_dev, 0xF7, 0x81,MSM_ISP_IRQ_ENABLE);

	msm_camera_io_w_mb(0x140000, VFE_BASE + 0x318);

	val = msm_camera_io_r(VFE_BASE + 0x2F8);
	val &= 0xFFFFFF3F;
	val = val | 0 << 7 | 1 << 6;
	msm_camera_io_w(val, VFE_BASE + 0x2F8);
	msm_camera_io_w_mb(0x4, VFE_BASE + 0x2F4);
	msm_camera_io_w_mb(0x1, VFE_BASE + 0x2F4);
}

void msm_isp_vfe_start_stream(struct vfe_device *vfe_dev){

	uint32_t idx = 0;
	dprintf(CRITICAL,"msm_isp_open_node: msm_vfe40_cfg_axi_ub_equal_default\n");
    msm_vfe40_cfg_axi_ub_equal_default();

    msm_camera_io_w(0x0,VFE_BASE + 0x88);
    msm_camera_io_w(0x38,VFE_BASE + 0x78);
    msm_camera_io_w(0x0,VFE_BASE + 0xac);
	msm_camera_io_w(0x38,VFE_BASE + 0x9c);

    dprintf(CRITICAL,"msm_isp_open_node: msm_isp_cfg_axi_stream\n");

    msm_vfe_bufq_init();
    idx = get_msm_vfe_buffer_idle();

    msm_camera_io_w(vfe_bufq.buffer_info[idx].addr,VFE_BASE + 0x70); //ping 0
    msm_camera_io_w(vfe_bufq.buffer_info[idx].addr + 1080*1920,VFE_BASE + 0x94); //ping 1
    vfe_bufq.buffer_info[idx].status = MSM_BUFFER_PING_SET;

    idx = get_msm_vfe_buffer_idle();
    msm_camera_io_w(vfe_bufq.buffer_info[idx].addr,VFE_BASE + 0x74); //pong 0
    msm_camera_io_w(vfe_bufq.buffer_info[idx].addr + 1080*1920,VFE_BASE + 0x98); //pong 1
    vfe_bufq.buffer_info[idx].status = MSM_BUFFER_PONG_SET;

	msm_isp_cfg_axi_stream(vfe_dev);
	vfe_send_hw_cfg();
	msm_camera_io_w(0x1,VFE_BASE + 0x88);
	msm_camera_io_w(0x0,VFE_BASE + 0x78);
	msm_camera_io_w(0x1,VFE_BASE + 0xac);
	msm_camera_io_w(0x0,VFE_BASE + 0x9c);
	dprintf(CRITICAL,"msm_isp_open_node: msm_isp_open_node X\n");
}

void msm_isp_request_axi_stream(struct vfe_device *vfe_dev){
	uint32_t xbar_reg_cfg = 0;
	uint32_t wm_base;
	msm_isp_cfg_io_format();

	wm_base = VFE40_WM_BASE(0);
	msm_camera_io_w(0x0, VFE_BASE + wm_base);
	msm_camera_io_w(0x770437, VFE_BASE + wm_base + 0x14);
	msm_camera_io_w(0xf021bb, VFE_BASE + wm_base + 0x18);
	msm_camera_io_w(0xFFFFFFFF,VFE_BASE + wm_base + 0x20);
	xbar_reg_cfg = msm_camera_io_r(VFE_BASE + VFE40_XBAR_BASE(0));
	xbar_reg_cfg &= ~(0xFFFF << VFE40_XBAR_SHIFT(0));
	xbar_reg_cfg |= (0x1 << VFE40_XBAR_SHIFT(0));
	msm_camera_io_w(xbar_reg_cfg,VFE_BASE + VFE40_XBAR_BASE(0));

	wm_base = VFE40_WM_BASE(1);
	xbar_reg_cfg = 0;
	msm_camera_io_w(0x0, VFE_BASE + wm_base);
	msm_camera_io_w(0x77021b, VFE_BASE + wm_base + 0x14);
	msm_camera_io_w(0xf010db, VFE_BASE + wm_base + 0x18);
	msm_camera_io_w(0xFFFFFFFF,VFE_BASE + wm_base + 0x20);
	xbar_reg_cfg = msm_camera_io_r(VFE_BASE + VFE40_XBAR_BASE(1));
	xbar_reg_cfg &= ~(0xFFFF << VFE40_XBAR_SHIFT(1));
	xbar_reg_cfg |= (0x3 << VFE40_XBAR_SHIFT(1));
	msm_camera_io_w(xbar_reg_cfg,VFE_BASE + VFE40_XBAR_BASE(1));

}

int msm_isp_open_node(struct vfe_device *vfe_dev){
	int rc = 0;

	rc = msm_vfe_gds_enable();
	if(rc < 0){
		return -1;
	}
    msm_camera_clk_enable(1);

	memset(&vfe_dev->error_info, 0, sizeof(vfe_dev->error_info));
	msm_vfe40_clear_status_reg(vfe_dev);

	vfe_dev->vfe_hw_version = msm_camera_io_r(VFE_BASE);
	dprintf(CRITICAL,"msm_isp_open_node: HW Version: 0x%x\n", vfe_dev->vfe_hw_version);

	dprintf(CRITICAL,"msm_isp_open_node: msm_vfe40_reset_hardware\n");
	rc = msm_vfe40_reset_hardware(vfe_dev,1,1);
	if(rc < 0){
		dprintf(CRITICAL,"msm_isp_open_node: msm_vfe40_reset_hardware failed\n");
		return rc;
	}
	msm_vfe40_init_hardware_reg(vfe_dev);

    //VIDIOC_MSM_ISP_INPUT_CFG
	msm_isp_cfg_input();
	msm_isp_request_axi_stream(vfe_dev);
	return 1;
}

void msm_camera_io_dump(uint32_t base,int size){
	uint32_t data;
	uint32_t p = 0;
	int i;

	for (i = 0; i < size/4; i++) {
		data = msm_camera_io_r(base + p);
		dprintf(CRITICAL,"xxxxxxxx 0x%x 0x%x\n",p,data);
		p += 0x4;
	}
}

#define vfe_hw_0(_vfe_base_) \
    {_vfe_base_ + 0x400,0x0},      \
    {_vfe_base_ + 0x404,0x10e87}, \
    {_vfe_base_ + 0x408,0xde10f0f0},\
    {_vfe_base_ + 0x40c,0x2010},\
    {_vfe_base_ + 0x410,0x0},\
    {_vfe_base_ + 0x414,0x0}, \
    {_vfe_base_ + 0x418,0x0},\
    {_vfe_base_ + 0x41c,0x0},\
    {_vfe_base_ + 0x420,0x0},\
    {_vfe_base_ + 0x424,0x1},      \
    {_vfe_base_ + 0x428,0x830083},\
    {_vfe_base_ + 0x42c,0x830083},\
    {_vfe_base_ + 0x430,0x0},\
    {_vfe_base_ + 0x434,0x0},\
    {_vfe_base_ + 0x438,0xca},\
    {_vfe_base_ + 0x43c,0x9c},\
    {_vfe_base_ + 0x518,0x7c00cc},\
    {_vfe_base_ + 0x51c,0x210050},\
    {_vfe_base_ + 0x568,0x80ff00},\
    {_vfe_base_ + 0x56c,0x1a020},\
    {_vfe_base_ + 0x520,0x80003066},\
    {_vfe_base_ + 0x524,0x81003066},\
    {_vfe_base_ + 0x528,0x82003066},\
    {_vfe_base_ + 0x52c,0x83003066},\
    {_vfe_base_ + 0x530,0x84003066},\
    {_vfe_base_ + 0x534,0x85003066},\
    {_vfe_base_ + 0x538,0x86003066},\
    {_vfe_base_ + 0x53c,0x87003066},\
    {_vfe_base_ + 0x540,0x88003066},\
    {_vfe_base_ + 0x544,0x803fe066},\
    {_vfe_base_ + 0x548,0x813fe066},\
    {_vfe_base_ + 0x54c,0x823fe066},\
    {_vfe_base_ + 0x550,0x833fe066},\
    {_vfe_base_ + 0x558,0x853fe066},\
    {_vfe_base_ + 0x55c,0x863fe066},\
    {_vfe_base_ + 0x560,0x873fe066},\
    {_vfe_base_ + 0x564,0x883fe066},\
    {_vfe_base_ + 0xae8,0x0},\
    {_vfe_base_ + 0xaec,0x0},\

struct camera_hw_reg_array hw_vfe0_d_init_regs[] = {
	vfe_hw_0(VFE_BASE)
};

#define vfe_hw_1(_vfe_base_) \
    {_vfe_base_ + 0xb94,0x6600cc0},\
    {_vfe_base_ + 0xb98,0x30040000},\
    {_vfe_base_ + 0xb9c,0x0},\
    {_vfe_base_ + 0xba0,0x0},\
    {_vfe_base_ + 0xba4,0x1980},\
    {_vfe_base_ + 0xba8,0x0},\
    {_vfe_base_ + 0xaf0,0x13f3e3f},\
    {_vfe_base_ + 0xaf4,0x80a0805},\
    {_vfe_base_ + 0xaf8,0x3e3f0105},\
    {_vfe_base_ + 0xafc,0x3f},\
    {_vfe_base_ + 0xb00,0x5e9},\
    {_vfe_base_ + 0xb04,0xf8c5fa17},\
    {_vfe_base_ + 0xb08,0xcbd26d94},\
    {_vfe_base_ + 0xb0c,0x73b},\
    {_vfe_base_ + 0xb10,0xc67b77a9},\
    {_vfe_base_ + 0xb14,0x0},\
    {_vfe_base_ + 0xb18,0x0},\
    {_vfe_base_ + 0xb1c,0x0},\
    {_vfe_base_ + 0xb20,0x0},\
    {_vfe_base_ + 0xb24,0x4ff},\
    {_vfe_base_ + 0xb28,0xef8afb01},\
    {_vfe_base_ + 0xb2c,0xc9ff6f11},\
    {_vfe_base_ + 0xb30,0x1076},\
    {_vfe_base_ + 0xb34,0xcf5b5e88},\
    {_vfe_base_ + 0xb38,0x1010101},\
    {_vfe_base_ + 0xb3c,0x1},\
    {_vfe_base_ + 0xb40,0x5e9},\
    {_vfe_base_ + 0xb44,0xfa17},\
    {_vfe_base_ + 0xb48,0xcbd26d94},\
    {_vfe_base_ + 0xb4c,0x30d},\
    {_vfe_base_ + 0xb50,0x10000},\
    {_vfe_base_ + 0xb54,0x10410400},\
    {_vfe_base_ + 0xb58,0x10410410},\
    {_vfe_base_ + 0xb5c,0x10410410},\
    {_vfe_base_ + 0xb60,0x410},\
    {_vfe_base_ + 0xb64,0x10000},\
    {_vfe_base_ + 0xb68,0x10410400},\
    {_vfe_base_ + 0xb6c,0x10410410},\
    {_vfe_base_ + 0xb70,0x10410410},\
    {_vfe_base_ + 0xb74,0x410},\
    {_vfe_base_ + 0xb78,0x10000},\
    {_vfe_base_ + 0xb7c,0x10410400},\
    {_vfe_base_ + 0xb80,0x10410410},\
    {_vfe_base_ + 0xb84,0x10410410},\
    {_vfe_base_ + 0xb88,0x410},\
    {_vfe_base_ + 0xb8c,0x100010},\
    {_vfe_base_ + 0xb90,0x10},\
    {_vfe_base_ + 0xb94,0x0},\
    {_vfe_base_ + 0xb98,0x0},\
    {_vfe_base_ + 0xb9c,0x0},\
    {_vfe_base_ + 0xba0,0x0},\
    {_vfe_base_ + 0xba4,0x0},\
    {_vfe_base_ + 0xba8,0x0},\
    {_vfe_base_ + 0xbac,0x73b},\
    {_vfe_base_ + 0xbb0,0xf8c5},\
    {_vfe_base_ + 0xbb4,0xc67b77a9},\

struct camera_hw_reg_array hw_vfe1_d_init_regs[] = {
	vfe_hw_1(VFE_BASE)
};

#define vfe_hw_2(_vfe_base_) \
    {_vfe_base_ + 0x88c,0x0},\
    {_vfe_base_ + 0x890,0x177cca65},\
    {_vfe_base_ + 0x894,0xffffffff},\
    {_vfe_base_ + 0x898,0x0},\
    {_vfe_base_ + 0x89c,0x5efcc431},\
    {_vfe_base_ + 0x8a0,0xffffffff},\
    {_vfe_base_ + 0x8e4,0x40000000},\
    {_vfe_base_ + 0x8e8,0x32f3432f},\
    {_vfe_base_ + 0x8ec,0x40000000},\
    {_vfe_base_ + 0x8f0,0x43f98f2},\

struct camera_hw_reg_array hw_vfe2_d_init_regs[] = {
	vfe_hw_2(VFE_BASE)
};

#define vfe_hw_3(_vfe_base_) \
	  {_vfe_base_ + 0x640,0x4d},\
	  {_vfe_base_ + 0x644,0x96},\
	  {_vfe_base_ + 0x648,0x1d},\
	  {_vfe_base_ + 0x64c,0x0},\
	  {_vfe_base_ + 0x650,0x800080},\
	  {_vfe_base_ + 0x654,0xfa90fa9},\
	  {_vfe_base_ + 0x658,0x800080},\
	  {_vfe_base_ + 0x65c,0xfd70fd7},\
	  {_vfe_base_ + 0x660,0x800080},\
	  {_vfe_base_ + 0x444,0x806020},\
	  {_vfe_base_ + 0x448,0x205002},\
	  {_vfe_base_ + 0x44c,0x1405002},\
	  {_vfe_base_ + 0x450,0x5002},\
	  {_vfe_base_ + 0x454,0x806020},\
	  {_vfe_base_ + 0x458,0x205002},\
	  {_vfe_base_ + 0x45c,0x1405002},\
	  {_vfe_base_ + 0x460,0x5002},\
      {_vfe_base_ + 0x464,0xa00090},\
      {_vfe_base_ + 0x468,0x14008d0},\
      {_vfe_base_ + 0x46c,0x1c70010},\
      {_vfe_base_ + 0x470,0x1830183},\
      {_vfe_base_ + 0x474,0x1830183},\
      {_vfe_base_ + 0x478,0x145016c},\
      {_vfe_base_ + 0x47c,0xee011e},\
      {_vfe_base_ + 0x480,0x9e00cd},\
      {_vfe_base_ + 0x484,0x5c007a},\
      {_vfe_base_ + 0x488,0x22003c},\
      {_vfe_base_ + 0x48c,0x6000e},\
      {_vfe_base_ + 0x490,0x0},\
      {_vfe_base_ + 0x494,0x0},\
      {_vfe_base_ + 0x498,0x0},\
      {_vfe_base_ + 0x49c,0x0},\
      {_vfe_base_ + 0x4a0,0xc000b0},\
      {_vfe_base_ + 0x4a4,0x8d0},\
      {_vfe_base_ + 0x4a8,0x1740010},\
      {_vfe_base_ + 0x4ac,0x2c202c2},\
      {_vfe_base_ + 0x4b0,0x2c202c2},\
      {_vfe_base_ + 0x4b4,0x2500297},\
      {_vfe_base_ + 0x4b8,0x1b10209},\
      {_vfe_base_ + 0x4bc,0x1200176},\
      {_vfe_base_ + 0x4c0,0xa700de},\
      {_vfe_base_ + 0x4c4,0x3e006d},\
      {_vfe_base_ + 0x4c8,0xb001a},\
      {_vfe_base_ + 0x4cc,0x0},\
      {_vfe_base_ + 0x4d0,0x0},\
      {_vfe_base_ + 0x4d4,0x0},\
      {_vfe_base_ + 0x4d8,0x0},\
      {_vfe_base_ + 0x4dc,0xb000a0},\
      {_vfe_base_ + 0x4e0,0x8d0},\
      {_vfe_base_ + 0x4e4,0x19a0010},\
      {_vfe_base_ + 0x4e8,0x24a024a},\
      {_vfe_base_ + 0x4ec,0x24a024a},\
      {_vfe_base_ + 0x4f0,0x1eb0226},\
      {_vfe_base_ + 0x4f4,0x16701b1},\
      {_vfe_base_ + 0x4f8,0xef0136},\
      {_vfe_base_ + 0x4fc,0x8b00b8},\
      {_vfe_base_ + 0x500,0x33005b},\
      {_vfe_base_ + 0x504,0x90015},\
      {_vfe_base_ + 0x508,0x0},\
      {_vfe_base_ + 0x50c,0x0},\
      {_vfe_base_ + 0x510,0x0},\
      {_vfe_base_ + 0x514,0x0},\
      {_vfe_base_ + 0x5d0,0x0},\
      {_vfe_base_ + 0x5d4,0x0},\
      {_vfe_base_ + 0x5d8,0x0},\
      {_vfe_base_ + 0x5dc,0x0},\
      {_vfe_base_ + 0x5e0,0x0},\
      {_vfe_base_ + 0x5e4,0x0},\
      {_vfe_base_ + 0x5e8,0x0},\
      {_vfe_base_ + 0x5ec,0x0},\
      {_vfe_base_ + 0x5f0,0x0},\
      {_vfe_base_ + 0x5f4,0x0},\
      {_vfe_base_ + 0x5f8,0x0},\
      {_vfe_base_ + 0x5fc,0x0},\
      {_vfe_base_ + 0x600,0x0},\
      {_vfe_base_ + 0x5d0,0xb5},\
      {_vfe_base_ + 0x5d4,0xffb},\
      {_vfe_base_ + 0x5d8,0xfd0},\
      {_vfe_base_ + 0x5dc,0xf54},\
      {_vfe_base_ + 0x5e0,0x125},\
      {_vfe_base_ + 0x5e4,0x7},\
      {_vfe_base_ + 0x5e8,0xf7c},\
      {_vfe_base_ + 0x5ec,0xd},\
      {_vfe_base_ + 0x5f0,0xf7},\
      {_vfe_base_ + 0x5f4,0x0},\
      {_vfe_base_ + 0x5f8,0x0},\
      {_vfe_base_ + 0x5fc,0x0},\
      {_vfe_base_ + 0x600,0x0},\
      {_vfe_base_ + 0x5d0,0xb8},\
      {_vfe_base_ + 0x5d4,0xffa},\
      {_vfe_base_ + 0x5d8,0xfcd},\
      {_vfe_base_ + 0x5dc,0xf4e},\
      {_vfe_base_ + 0x5e0,0x12b},\
      {_vfe_base_ + 0x5e4,0x7},\
      {_vfe_base_ + 0x5e8,0xf74},\
      {_vfe_base_ + 0x5ec,0xf},\
      {_vfe_base_ + 0x5f0,0xfd},\
      {_vfe_base_ + 0x5f4,0x0},\
      {_vfe_base_ + 0x5f8,0x0},\
      {_vfe_base_ + 0x5fc,0x0},\
      {_vfe_base_ + 0x600,0x0},\
      {_vfe_base_ + 0x580,0x331f080},\


struct camera_hw_reg_array hw_vfe3_d_init_regs[] = {
	vfe_hw_3(VFE_BASE)
};
#define vfe_hw_4(_vfe_base_) \
    {_vfe_base_ + 0x670,0xebc81e0a},\
    {_vfe_base_ + 0x674,0x852a8000},\
    {_vfe_base_ + 0x678,0x197f2d},\
    {_vfe_base_ + 0x67c,0xebc82814},\
    {_vfe_base_ + 0x680,0x742a8000},\
    {_vfe_base_ + 0x684,0x10f4f6},\
    {_vfe_base_ + 0x688,0xffeb9650},\
    {_vfe_base_ + 0x68c,0x852a8000},\
    {_vfe_base_ + 0x690,0x1919ee},\
    {_vfe_base_ + 0x694,0xa1128},\
    {_vfe_base_ + 0x698,0x114628},\
    {_vfe_base_ + 0x69c,0x464128},\
    {_vfe_base_ + 0x6a0,0x412d28},\
    {_vfe_base_ + 0x6a4,0x2d0a28},\
    {_vfe_base_ + 0x6a8,0xecfde2},\
    {_vfe_base_ + 0x6ac,0xfde7e2},\
    {_vfe_base_ + 0x6b0,0xe7c9e2},\
    {_vfe_base_ + 0x6b4,0xc9a6e2},\
    {_vfe_base_ + 0x6b8,0xa6ece2},\
    {_vfe_base_ + 0x6bc,0x400},\
    {_vfe_base_ + 0x6c0,0x400},\
    {_vfe_base_ + 0x6c4,0x400},\
    {_vfe_base_ + 0x6c8,0x400},\
    {_vfe_base_ + 0x6cc,0x400},\
    {_vfe_base_ + 0x6d0,0x400},\
    {_vfe_base_ + 0x6d4,0x4000000},\
    {_vfe_base_ + 0x6d8,0x4000000},\
    {_vfe_base_ + 0x6dc,0x4000000},\
    {_vfe_base_ + 0x6e0,0x4000000},\
    {_vfe_base_ + 0x6e4,0x4000000},\
    {_vfe_base_ + 0x6e8,0x4000000},\
    {_vfe_base_ + 0x6ec,0x0},\
    {_vfe_base_ + 0x6f0,0x0},\
    {_vfe_base_ + 0x6f4,0x0},\
    {_vfe_base_ + 0x6f8,0x0},\
    {_vfe_base_ + 0x6fc,0x0},\
    {_vfe_base_ + 0x700,0x0},\
    {_vfe_base_ + 0x704,0xa00000},\
    {_vfe_base_ + 0x708,0xa00000},\
    {_vfe_base_ + 0x70c,0xa00000},\
    {_vfe_base_ + 0x710,0xa00000},\
    {_vfe_base_ + 0x714,0xa00000},\
    {_vfe_base_ + 0x718,0xa00000},\
    {_vfe_base_ + 0x73c,0x400},\
    {_vfe_base_ + 0x740,0x0},\
    {_vfe_base_ + 0x744,0x4000000},\
    {_vfe_base_ + 0x748,0x0},\
    {_vfe_base_ + 0x74c,0x0},\
    {_vfe_base_ + 0x750,0x400},\
    {_vfe_base_ + 0x754,0xffffff},\
    {_vfe_base_ + 0x758,0x0},\
    {_vfe_base_ + 0x7a4,0x3},\
    {_vfe_base_ + 0x7a8,0x7800cc0},\
    {_vfe_base_ + 0x7ac,0x31b333},\
    {_vfe_base_ + 0x7b0,0x0},\
    {_vfe_base_ + 0x7b4,0x5a00990},\
    {_vfe_base_ + 0x7b8,0x31b333},\
    {_vfe_base_ + 0x7bc,0x0},\
    {_vfe_base_ + 0x7c0,0x3},\
    {_vfe_base_ + 0x7c4,0x3c00cc0},\
    {_vfe_base_ + 0x7c8,0x336666},\
    {_vfe_base_ + 0x7cc,0x0},\
    {_vfe_base_ + 0x7d0,0x0},\
    {_vfe_base_ + 0x7d4,0xcc0},\
    {_vfe_base_ + 0x7d8,0x2d00990},\
    {_vfe_base_ + 0x7dc,0x336666},\
    {_vfe_base_ + 0x7e0,0x0},\
    {_vfe_base_ + 0x7e4,0x0},\
    {_vfe_base_ + 0x7e8,0x990},\
    {_vfe_base_ + 0x980,0xcc0},\
    {_vfe_base_ + 0x984,0x990},\
    {_vfe_base_ + 0x864,0x77f},\
    {_vfe_base_ + 0x868,0xb404eb},\
    {_vfe_base_ + 0x86c,0x3bf},\
    {_vfe_base_ + 0x870,0x5a0275},\
    {_vfe_base_ + 0x87c,0xffffff},\
    {_vfe_base_ + 0x880,0x0},\


struct camera_hw_reg_array hw_vfe4_d_init_regs[] = {
	vfe_hw_4(VFE_BASE)
};

#define vfe_hw_start(_vfe_base_) \
    {_vfe_base_ + 0x37c,0x1},      \
    {_vfe_base_ + 0x380,0x48023e}, \
    {_vfe_base_ + 0x384,0x435062c},\
    {_vfe_base_ + 0x388,0x8230a1a},\
    {_vfe_base_ + 0x38c,0xc110e08},\
    {_vfe_base_ + 0x390,0x43023a}, \
    {_vfe_base_ + 0x394,0x4320629},\
    {_vfe_base_ + 0x398,0x8210a18},\
    {_vfe_base_ + 0x39c,0xc100e07},\
    {_vfe_base_ + 0x3a0,0x43023a}, \
    {_vfe_base_ + 0x3a4,0x4320629},\
    {_vfe_base_ + 0x3a8,0x8210a18},\
    {_vfe_base_ + 0x3ac,0xc100e07},\
    {_vfe_base_ + 0x3b0,0x48023e}, \
    {_vfe_base_ + 0x3b4,0x435062c},\
    {_vfe_base_ + 0x3b8,0x8230a1a},\
    {_vfe_base_ + 0x3bc,0xc110e08},\


struct camera_hw_reg_array hw_vfe_start_d_init_regs[] = {
	vfe_hw_start(VFE_BASE)
};

#define vfe_hw_start_11(_vfe_base_) \
    {_vfe_base_ + 0x37c,0x0},      \
    {_vfe_base_ + 0x380,0x48023e}, \
    {_vfe_base_ + 0x384,0x435062c},\
    {_vfe_base_ + 0x388,0x8230a1a},\
    {_vfe_base_ + 0x38c,0xc110e08},\
    {_vfe_base_ + 0x390,0x43023a}, \
    {_vfe_base_ + 0x394,0x4320629},\
    {_vfe_base_ + 0x398,0x8210a18},\
    {_vfe_base_ + 0x39c,0xc100e07},\
    {_vfe_base_ + 0x3a0,0x43023a}, \
    {_vfe_base_ + 0x3a4,0x4320629},\
    {_vfe_base_ + 0x3a8,0x8210a18},\
    {_vfe_base_ + 0x3ac,0xc100e07},\
    {_vfe_base_ + 0x3b0,0x48023e}, \
    {_vfe_base_ + 0x3b4,0x435062c},\
    {_vfe_base_ + 0x3b8,0x8230a1a},\
    {_vfe_base_ + 0x3bc,0xc110e08},\

struct camera_hw_reg_array hw_vfe_start_11_d_init_regs[] = {
	vfe_hw_start_11(VFE_BASE)
};

void  vfe_wite_mb(uint32_t data0,uint32_t data1){
	msm_camera_io_w(data0,VFE_BASE + 0x910);
	msm_camera_io_w(data1,VFE_BASE + 0x914);
}

#define vfe_hw_dmi0(_vfe_base_)\
    {_vfe_base_ + 0x91c,0x0},\
    {_vfe_base_ + 0x91c,0x0},\
    {_vfe_base_ + 0x91c,0x0},\
    {_vfe_base_ + 0x91c,0x0},\
    {_vfe_base_ + 0x91c,0x200000 },\
    {_vfe_base_ + 0x91c,0x200000 },\
    {_vfe_base_ + 0x91c,0x200000 },\
    {_vfe_base_ + 0x91c,0x200000 },\
    {_vfe_base_ + 0x91c,0x2001f6 },\
    {_vfe_base_ + 0x91c,0x2001f6 },\
    {_vfe_base_ + 0x91c,0x2001f7 },\
    {_vfe_base_ + 0x91c,0x2001f7 },\
    {_vfe_base_ + 0x91c,0x2003ed },\
    {_vfe_base_ + 0x91c,0x2003ed },\
    {_vfe_base_ + 0x91c,0x2003ef },\
    {_vfe_base_ + 0x91c,0x2003ef },\
    {_vfe_base_ + 0x91c,0x2005e4 },\
    {_vfe_base_ + 0x91c,0x2005e4 },\
    {_vfe_base_ + 0x91c,0x2005e6 },\
    {_vfe_base_ + 0x91c,0x2005e6 },\
    {_vfe_base_ + 0x91c,0x2007db },\
    {_vfe_base_ + 0x91c,0x2007db },\
    {_vfe_base_ + 0x91c,0x2007de },\
    {_vfe_base_ + 0x91c,0x2007de },\
    {_vfe_base_ + 0x91c,0x2009d2 },\
    {_vfe_base_ + 0x91c,0x2009d2 },\
    {_vfe_base_ + 0x91c,0x2009d5 },\
    {_vfe_base_ + 0x91c,0x2009d5 },\
    {_vfe_base_ + 0x91c,0x200bc9 },\
    {_vfe_base_ + 0x91c,0x200bc9 },\
    {_vfe_base_ + 0x91c,0x200bcd },\
    {_vfe_base_ + 0x91c,0x200bcd },\
    {_vfe_base_ + 0x91c,0x249dc0 },\
    {_vfe_base_ + 0x91c,0x249dc0 },\
    {_vfe_base_ + 0x91c,0x244dc4 },\
    {_vfe_base_ + 0x91c,0x244dc4 },\


struct camera_hw_reg_array hw_vfe_hw_dmi0_regs[] = {
	vfe_hw_dmi0(VFE_BASE)
};

 #define vfe_hw_dmi1(_vfe_base_)\
    {_vfe_base_ + 0x91c,0xd7a676},\
 	{_vfe_base_ + 0x91c,0xc345e8},\
 	{_vfe_base_ + 0x91c,0xb8659d},\
 	{_vfe_base_ + 0x91c,0xb0655e},\
 	{_vfe_base_ + 0x91c,0xa7e515},\
 	{_vfe_base_ + 0x91c,0xa244ed},\
 	{_vfe_base_ + 0x91c,0xa024da},\
 	{_vfe_base_ + 0x91c,0xa224e8},\
 	{_vfe_base_ + 0x91c,0xa7650f},\
 	{_vfe_base_ + 0x91c,0xaf254e},\
 	{_vfe_base_ + 0x91c,0xb5e586},\
 	{_vfe_base_ + 0x91c,0xc0a5d6},\
 	{_vfe_base_ + 0x91c,0xd2e667},\
 	{_vfe_base_ + 0x91c,0xcbe61b},\
 	{_vfe_base_ + 0x91c,0xb50589},\
 	{_vfe_base_ + 0x91c,0xac4545},\
 	{_vfe_base_ + 0x91c,0xa464fa},\
 	{_vfe_base_ + 0x91c,0x9c44b5},\
 	{_vfe_base_ + 0x91c,0x96648f},\
 	{_vfe_base_ + 0x91c,0x94447f},\
 	{_vfe_base_ + 0x91c,0x964488},\
 	{_vfe_base_ + 0x91c,0x9bc4a8},\
 	{_vfe_base_ + 0x91c,0xa424ec},\
 	{_vfe_base_ + 0x91c,0xac253b},\
 	{_vfe_base_ + 0x91c,0xb46583},\
 	{_vfe_base_ + 0x91c,0xc74609},\
 	{_vfe_base_ + 0x91c,0xc105d0},\
 	{_vfe_base_ + 0x91c,0xae2559},\
 	{_vfe_base_ + 0x91c,0xa48504},\
 	{_vfe_base_ + 0x91c,0x9b44a1},\
 	{_vfe_base_ + 0x91c,0x92c45d},\
 	{_vfe_base_ + 0x91c,0x8d2444},\
 	{_vfe_base_ + 0x91c,0x8ac439},\
 	{_vfe_base_ + 0x91c,0x8cc43e},\
 	{_vfe_base_ + 0x91c,0x926451},\
 	{_vfe_base_ + 0x91c,0x9b0491},\
 	{_vfe_base_ + 0x91c,0xa484f5},\
 	{_vfe_base_ + 0x91c,0xade554},\
 	{_vfe_base_ + 0x91c,0xbce5c2},\
 	{_vfe_base_ + 0x91c,0xb8e5a1},\
 	{_vfe_base_ + 0x91c,0xa9853d},\
 	{_vfe_base_ + 0x91c,0x9f64d9},\
 	{_vfe_base_ + 0x91c,0x94c46d},\
 	{_vfe_base_ + 0x91c,0x8c6437},\
 	{_vfe_base_ + 0x91c,0x868429},\
 	{_vfe_base_ + 0x91c,0x84a429},\
 	{_vfe_base_ + 0x91c,0x86441f},\
 	{_vfe_base_ + 0x91c,0x8be423},\
 	{_vfe_base_ + 0x91c,0x946456},\
 	{_vfe_base_ + 0x91c,0x9ea4c0},\
 	{_vfe_base_ + 0x91c,0xa8452d},\
 	{_vfe_base_ + 0x91c,0xb4c592},\
 	{_vfe_base_ + 0x91c,0xb5c599},\
 	{_vfe_base_ + 0x91c,0xa7e536},\
 	{_vfe_base_ + 0x91c,0x9ce4c7},\
 	{_vfe_base_ + 0x91c,0x92045c},\
 	{_vfe_base_ + 0x91c,0x89242c},\
 	{_vfe_base_ + 0x91c,0x83a428},\
 	{_vfe_base_ + 0x91c,0x81c414},\
 	{_vfe_base_ + 0x91c,0x83641c},\
 	{_vfe_base_ + 0x91c,0x88c416},\
 	{_vfe_base_ + 0x91c,0x91043e},\
 	{_vfe_base_ + 0x91c,0x9b84a7},\
 	{_vfe_base_ + 0x91c,0xa5c51e},\
 	{_vfe_base_ + 0x91c,0xb1a579},\
 	{_vfe_base_ + 0x91c,0xb4e591},\
 	{_vfe_base_ + 0x91c,0xa84538},\
 	{_vfe_base_ + 0x91c,0x9d64c6},\
 	{_vfe_base_ + 0x91c,0x92645b},\
 	{_vfe_base_ + 0x91c,0x89842a},\
 	{_vfe_base_ + 0x91c,0x83e425},\
 	{_vfe_base_ + 0x91c,0x824415},\
 	{_vfe_base_ + 0x91c,0x83c41a},\
 	{_vfe_base_ + 0x91c,0x890416},\
 	{_vfe_base_ + 0x91c,0x91843f},\
 	{_vfe_base_ + 0x91c,0x9be4a9},\
 	{_vfe_base_ + 0x91c,0xa5c518},\
 	{_vfe_base_ + 0x91c,0xb1257c},\
 	{_vfe_base_ + 0x91c,0xb8a59b},\
 	{_vfe_base_ + 0x91c,0xaa4537},\
 	{_vfe_base_ + 0x91c,0xa0e4d9},\
 	{_vfe_base_ + 0x91c,0x96646f},\
 	{_vfe_base_ + 0x91c,0x8da435},\
 	{_vfe_base_ + 0x91c,0x87a425},\
 	{_vfe_base_ + 0x91c,0x858422},\
 	{_vfe_base_ + 0x91c,0x87841e},\
 	{_vfe_base_ + 0x91c,0x8d2424},\
 	{_vfe_base_ + 0x91c,0x958459},\
 	{_vfe_base_ + 0x91c,0x9f64c1},\
 	{_vfe_base_ + 0x91c,0xa82524},\
 	{_vfe_base_ + 0x91c,0xb3a581},\
 	{_vfe_base_ + 0x91c,0xc025bc},\
 	{_vfe_base_ + 0x91c,0xae4546},\
 	{_vfe_base_ + 0x91c,0xa6c500},\
 	{_vfe_base_ + 0x91c,0x9da4a4},\
 	{_vfe_base_ + 0x91c,0x948460},\
 	{_vfe_base_ + 0x91c,0x8ee444},\
 	{_vfe_base_ + 0x91c,0x8cc43b},\
 	{_vfe_base_ + 0x91c,0x8ea43f},\
 	{_vfe_base_ + 0x91c,0x944455},\
 	{_vfe_base_ + 0x91c,0x9ca494},\
 	{_vfe_base_ + 0x91c,0xa5e4f1},\
 	{_vfe_base_ + 0x91c,0xace53d},\
 	{_vfe_base_ + 0x91c,0xbc25ac},\
 	{_vfe_base_ + 0x91c,0xc965fd},\
 	{_vfe_base_ + 0x91c,0xb3e569},\
 	{_vfe_base_ + 0x91c,0xac052f},\
 	{_vfe_base_ + 0x91c,0xa6c4fd},\
 	{_vfe_base_ + 0x91c,0x9e64c0},\
 	{_vfe_base_ + 0x91c,0x98e49a},\
 	{_vfe_base_ + 0x91c,0x97048d},\
 	{_vfe_base_ + 0x91c,0x98e498},\
 	{_vfe_base_ + 0x91c,0x9e64b6},\
 	{_vfe_base_ + 0x91c,0xa624f3},\
 	{_vfe_base_ + 0x91c,0xab8529},\
 	{_vfe_base_ + 0x91c,0xb2e564},\
 	{_vfe_base_ + 0x91c,0xc6a5f6},\
 	{_vfe_base_ + 0x91c,0xd2464c},\
 	{_vfe_base_ + 0x91c,0xbfa5b5},\
 	{_vfe_base_ + 0x91c,0xb4a56c},\
 	{_vfe_base_ + 0x91c,0xae6544},\
 	{_vfe_base_ + 0x91c,0xa9051d},\
 	{_vfe_base_ + 0x91c,0xa4c503},\
 	{_vfe_base_ + 0x91c,0xa2e4f3},\
 	{_vfe_base_ + 0x91c,0xa4a4fd},\
 	{_vfe_base_ + 0x91c,0xa8a515},\
 	{_vfe_base_ + 0x91c,0xadc538},\
 	{_vfe_base_ + 0x91c,0xb3a560},\
 	{_vfe_base_ + 0x91c,0xbe25ad},\
 	{_vfe_base_ + 0x91c,0xcf8639},\
 	{_vfe_base_ + 0x91c,0xd7c697},\
 	{_vfe_base_ + 0x91c,0xc50604},\
 	{_vfe_base_ + 0x91c,0xb9e5b3},\
 	{_vfe_base_ + 0x91c,0xb20574},\
 	{_vfe_base_ + 0x91c,0xa9853a},\
 	{_vfe_base_ + 0x91c,0xa44515},\
 	{_vfe_base_ + 0x91c,0xa1e50c},\
 	{_vfe_base_ + 0x91c,0xa4051b},\
 	{_vfe_base_ + 0x91c,0xa90544},\
 	{_vfe_base_ + 0x91c,0xb16581},\
 	{_vfe_base_ + 0x91c,0xb825b4},\
 	{_vfe_base_ + 0x91c,0xc20600},\
 	{_vfe_base_ + 0x91c,0xd56691},\
 	{_vfe_base_ + 0x91c,0xcd064a},\
 	{_vfe_base_ + 0x91c,0xb605a4},\
 	{_vfe_base_ + 0x91c,0xadc562},\
 	{_vfe_base_ + 0x91c,0xa6652a},\
 	{_vfe_base_ + 0x91c,0x9de4ea},\
 	{_vfe_base_ + 0x91c,0x9824c3},\
 	{_vfe_base_ + 0x91c,0x9664b6},\
 	{_vfe_base_ + 0x91c,0x9824c6},\
 	{_vfe_base_ + 0x91c,0x9da4ee},\
 	{_vfe_base_ + 0x91c,0xa60530},\
 	{_vfe_base_ + 0x91c,0xade568},\
 	{_vfe_base_ + 0x91c,0xb645a4},\
 	{_vfe_base_ + 0x91c,0xc8e625},\
 	{_vfe_base_ + 0x91c,0xc165f9},\
 	{_vfe_base_ + 0x91c,0xaee56f},\
 	{_vfe_base_ + 0x91c,0xa5c52b},\
 	{_vfe_base_ + 0x91c,0x9cc4ea},\
 	{_vfe_base_ + 0x91c,0x9424a7},\
 	{_vfe_base_ + 0x91c,0x8e847e},\
 	{_vfe_base_ + 0x91c,0x8c646e},\
 	{_vfe_base_ + 0x91c,0x8e647d},\
 	{_vfe_base_ + 0x91c,0x93e4a8},\
 	{_vfe_base_ + 0x91c,0x9ca4e9},\
 	{_vfe_base_ + 0x91c,0xa6652e},\
 	{_vfe_base_ + 0x91c,0xaf656c},\
 	{_vfe_base_ + 0x91c,0xbea5d2},\
 	{_vfe_base_ + 0x91c,0xb8e5c0},\
 	{_vfe_base_ + 0x91c,0xaa054e},\
 	{_vfe_base_ + 0x91c,0xa0250b},\
 	{_vfe_base_ + 0x91c,0x95e4b9},\
 	{_vfe_base_ + 0x91c,0x8d6478},\
 	{_vfe_base_ + 0x91c,0x87a44c},\
 	{_vfe_base_ + 0x91c,0x85c43f},\
 	{_vfe_base_ + 0x91c,0x87844c},\
 	{_vfe_base_ + 0x91c,0x8d2477},\
 	{_vfe_base_ + 0x91c,0x95c4b9},\
 	{_vfe_base_ + 0x91c,0xa024ff},\
 	{_vfe_base_ + 0x91c,0xa9c544},\
 	{_vfe_base_ + 0x91c,0xb5e593},\
 	{_vfe_base_ + 0x91c,0xb565a9},\
 	{_vfe_base_ + 0x91c,0xa7c541},\
 	{_vfe_base_ + 0x91c,0x9d44f3},\
 	{_vfe_base_ + 0x91c,0x9284a6},\
 	{_vfe_base_ + 0x91c,0x89c45a},\
 	{_vfe_base_ + 0x91c,0x842431},\
 	{_vfe_base_ + 0x91c,0x82641a},\
 	{_vfe_base_ + 0x91c,0x840432},\
 	{_vfe_base_ + 0x91c,0x89645e},\
 	{_vfe_base_ + 0x91c,0x91c49e},\
 	{_vfe_base_ + 0x91c,0x9c44e8},\
 	{_vfe_base_ + 0x91c,0xa6852f},\
 	{_vfe_base_ + 0x91c,0xb2057f},\
 	{_vfe_base_ + 0x91c,0xb465a1},\
 	{_vfe_base_ + 0x91c,0xa7a543},\
 	{_vfe_base_ + 0x91c,0x9d64f4},\
 	{_vfe_base_ + 0x91c,0x9284a6},\
 	{_vfe_base_ + 0x91c,0x89a45f},\
 	{_vfe_base_ + 0x91c,0x83e42d},\
 	{_vfe_base_ + 0x91c,0x824414},\
 	{_vfe_base_ + 0x91c,0x83e42f},\
 	{_vfe_base_ + 0x91c,0x89245e},\
 	{_vfe_base_ + 0x91c,0x91c49f},\
 	{_vfe_base_ + 0x91c,0x9c24e5},\
 	{_vfe_base_ + 0x91c,0xa6052d},\
 	{_vfe_base_ + 0x91c,0xb14576},\
 	{_vfe_base_ + 0x91c,0xb7c5b3},\
 	{_vfe_base_ + 0x91c,0xa9c54d},\
 	{_vfe_base_ + 0x91c,0xa0e50b},\
 	{_vfe_base_ + 0x91c,0x9664bf},\
 	{_vfe_base_ + 0x91c,0x8d847c},\
 	{_vfe_base_ + 0x91c,0x878450},\
 	{_vfe_base_ + 0x91c,0x85a43f},\
 	{_vfe_base_ + 0x91c,0x87844f},\
 	{_vfe_base_ + 0x91c,0x8d047a},\
 	{_vfe_base_ + 0x91c,0x9584b9},\
 	{_vfe_base_ + 0x91c,0x9fe4ff},\
 	{_vfe_base_ + 0x91c,0xa8253c},\
 	{_vfe_base_ + 0x91c,0xb42593},\
 	{_vfe_base_ + 0x91c,0xbfa5ea},\
 	{_vfe_base_ + 0x91c,0xadc567},\
 	{_vfe_base_ + 0x91c,0xa66531},\
 	{_vfe_base_ + 0x91c,0x9d64ef},\
 	{_vfe_base_ + 0x91c,0x9464ae},\
 	{_vfe_base_ + 0x91c,0x8e8487},\
 	{_vfe_base_ + 0x91c,0x8ca478},\
 	{_vfe_base_ + 0x91c,0x8e8484},\
 	{_vfe_base_ + 0x91c,0x9424ab},\
 	{_vfe_base_ + 0x91c,0x9ca4e7},\
 	{_vfe_base_ + 0x91c,0xa5e526},\
 	{_vfe_base_ + 0x91c,0xad055b},\
 	{_vfe_base_ + 0x91c,0xbbe5ca},\
 	{_vfe_base_ + 0x91c,0xc88624},\
 	{_vfe_base_ + 0x91c,0xb3658d},\
 	{_vfe_base_ + 0x91c,0xaba551},\
 	{_vfe_base_ + 0x91c,0xa6652b},\
 	{_vfe_base_ + 0x91c,0x9e04ee},\
 	{_vfe_base_ + 0x91c,0x9864ca},\
 	{_vfe_base_ + 0x91c,0x96c4c0},\
 	{_vfe_base_ + 0x91c,0x9884ca},\
 	{_vfe_base_ + 0x91c,0x9de4ec},\
 	{_vfe_base_ + 0x91c,0xa5e525},\
 	{_vfe_base_ + 0x91c,0xab454a},\
 	{_vfe_base_ + 0x91c,0xb28588},\
 	{_vfe_base_ + 0x91c,0xc64614},\
 	{_vfe_base_ + 0x91c,0xd02656},\
 	{_vfe_base_ + 0x91c,0xbf25d9},\
 	{_vfe_base_ + 0x91c,0xb3a584},\
 	{_vfe_base_ + 0x91c,0xad4558},\
 	{_vfe_base_ + 0x91c,0xa8a531},\
 	{_vfe_base_ + 0x91c,0xa46518},\
 	{_vfe_base_ + 0x91c,0xa26509},\
 	{_vfe_base_ + 0x91c,0xa48515},\
 	{_vfe_base_ + 0x91c,0xa7e52d},\
 	{_vfe_base_ + 0x91c,0xad654f},\
 	{_vfe_base_ + 0x91c,0xb3257d},\
 	{_vfe_base_ + 0x91c,0xbde5cf},\
 	{_vfe_base_ + 0x91c,0xcf0654},\


struct camera_hw_reg_array hw_vfe_hw_dmi1_regs[] = {
	vfe_hw_dmi1(VFE_BASE)
};


 #define vfe_hw_dmi2(_vfe_base_,reg_offset)\
    {_vfe_base_ + reg_offset,0x13281cb},\
    {_vfe_base_ + reg_offset + 0x4,0x508079},\
    {_vfe_base_ + reg_offset,0x15b01cb},\
    {_vfe_base_ + reg_offset + 0x4,0x508079},\
    {_vfe_base_ + reg_offset,0x18381cb},\
    {_vfe_base_ + reg_offset + 0x4,0x508079},\
    {_vfe_base_ + reg_offset,0x1ac01cb},\
    {_vfe_base_ + reg_offset + 0x4,0x508079},\
    {_vfe_base_ + reg_offset,0x1d481cb},\
    {_vfe_base_ + reg_offset + 0x4,0x508079},\
    {_vfe_base_ + reg_offset,0x1328208},\
	{_vfe_base_ + reg_offset + 0x4,0x508079},\
    {_vfe_base_ + reg_offset,0x15b0208},\
    {_vfe_base_ + reg_offset + 0x4,0x508079},\
    {_vfe_base_ + reg_offset,0x1838208},\
    {_vfe_base_ + reg_offset + 0x4,0x508079},\
    {_vfe_base_ + reg_offset,0x1ac0208},\
    {_vfe_base_ + reg_offset + 0x4,0x508079},\
    {_vfe_base_ + reg_offset,0x1d48208},\
    {_vfe_base_ + reg_offset + 0x4,0x508079},\
    {_vfe_base_ + reg_offset,0x1328245},\
    {_vfe_base_ + reg_offset + 0x4,0x508079},\
    {_vfe_base_ + reg_offset,0x15b0245},\
    {_vfe_base_ + reg_offset + 0x4,0x508079},\
    {_vfe_base_ + reg_offset,0x1838245},\
    {_vfe_base_ + reg_offset + 0x4,0x508079},\
    {_vfe_base_ + reg_offset,0x1ac0245},\
    {_vfe_base_ + reg_offset + 0x4,0x508079},\
    {_vfe_base_ + reg_offset,0x1d48245},\
    {_vfe_base_ + reg_offset + 0x4,0x508079},\
    {_vfe_base_ + reg_offset,0x1328282},\
    {_vfe_base_ + reg_offset + 0x4,0x508079},\
    {_vfe_base_ + reg_offset,0x15b0282},\
    {_vfe_base_ + reg_offset + 0x4,0x508079},\
    {_vfe_base_ + reg_offset,0x1838282},\
    {_vfe_base_ + reg_offset + 0x4,0x508079},\
    {_vfe_base_ + reg_offset,0x1ac0282},\
    {_vfe_base_ + reg_offset + 0x4,0x508079},\
    {_vfe_base_ + reg_offset,0x1d48282},\
    {_vfe_base_ + reg_offset + 0x4,0x508079},\
    {_vfe_base_ + reg_offset,0x13282bf},\
    {_vfe_base_ + reg_offset + 0x4,0x508079},\
    {_vfe_base_ + reg_offset,0x15b02bf},\
    {_vfe_base_ + reg_offset + 0x4,0x508079},\
    {_vfe_base_ + reg_offset,0x18382bf},\
    {_vfe_base_ + reg_offset + 0x4,0x508079},\
    {_vfe_base_ + reg_offset,0x1ac02bf},\
    {_vfe_base_ + reg_offset + 0x4,0x508079},\
    {_vfe_base_ + reg_offset,0x1d482bf},\
    {_vfe_base_ + reg_offset + 0x4,0x508079},\
    {_vfe_base_ + reg_offset,0x1d482bf},\
    {_vfe_base_ + reg_offset + 0x4,0x508079}

struct camera_hw_reg_array hw_vfe_hw_dmi2_regs[] = {
	vfe_hw_dmi2(VFE_BASE,0x918)
};

#define vfe_hw_dmi3(_vfe_base_,reg_offset) \
    {_vfe_base_ + reg_offset + 0x4,0xb02},\
    {_vfe_base_ + reg_offset + 0x4,0xd0d},\
    {_vfe_base_ + reg_offset + 0x4,0xd1a},\
    {_vfe_base_ + reg_offset + 0x4,0xc27},\
    {_vfe_base_ + reg_offset + 0x4,0xb33},\
    {_vfe_base_ + reg_offset + 0x4,0x93e},\
    {_vfe_base_ + reg_offset + 0x4,0x947},\
    {_vfe_base_ + reg_offset + 0x4,0x750},\
    {_vfe_base_ + reg_offset + 0x4,0x857},\
    {_vfe_base_ + reg_offset + 0x4,0x65f},\
    {_vfe_base_ + reg_offset + 0x4,0x765},\
    {_vfe_base_ + reg_offset + 0x4,0x66c},\
    {_vfe_base_ + reg_offset + 0x4,0x572},\
    {_vfe_base_ + reg_offset + 0x4,0x677},\
    {_vfe_base_ + reg_offset + 0x4,0x67d},\
    {_vfe_base_ + reg_offset + 0x4,0x483},\
    {_vfe_base_ + reg_offset + 0x4,0x687},\
    {_vfe_base_ + reg_offset + 0x4,0x48d},\
    {_vfe_base_ + reg_offset + 0x4,0x491},\
    {_vfe_base_ + reg_offset + 0x4,0x495},\
    {_vfe_base_ + reg_offset + 0x4,0x499},\
    {_vfe_base_ + reg_offset + 0x4,0x49d},\
    {_vfe_base_ + reg_offset + 0x4,0x4a1},\
    {_vfe_base_ + reg_offset + 0x4,0x3a5},\
    {_vfe_base_ + reg_offset + 0x4,0x3a8},\
    {_vfe_base_ + reg_offset + 0x4,0x4ab},\
    {_vfe_base_ + reg_offset + 0x4,0x3af},\
    {_vfe_base_ + reg_offset + 0x4,0x3b2},\
    {_vfe_base_ + reg_offset + 0x4,0x3b5},\
    {_vfe_base_ + reg_offset + 0x4,0x3b8},\
    {_vfe_base_ + reg_offset + 0x4,0x2bb},\
    {_vfe_base_ + reg_offset + 0x4,0x3bd},\
    {_vfe_base_ + reg_offset + 0x4,0x3c0},\
    {_vfe_base_ + reg_offset + 0x4,0x3c3},\
    {_vfe_base_ + reg_offset + 0x4,0x2c6},\
    {_vfe_base_ + reg_offset + 0x4,0x3c8},\
    {_vfe_base_ + reg_offset + 0x4,0x2cb},\
    {_vfe_base_ + reg_offset + 0x4,0x2cd},\
    {_vfe_base_ + reg_offset + 0x4,0x3cf},\
    {_vfe_base_ + reg_offset + 0x4,0x2d2},\
    {_vfe_base_ + reg_offset + 0x4,0x2d4},\
    {_vfe_base_ + reg_offset + 0x4,0x3d6},\
    {_vfe_base_ + reg_offset + 0x4,0x2d9},\
    {_vfe_base_ + reg_offset + 0x4,0x2db},\
    {_vfe_base_ + reg_offset + 0x4,0x2dd},\
    {_vfe_base_ + reg_offset + 0x4,0x2df},\
    {_vfe_base_ + reg_offset + 0x4,0x2e1},\
    {_vfe_base_ + reg_offset + 0x4,0x1e3},\
    {_vfe_base_ + reg_offset + 0x4,0x2e4},\
    {_vfe_base_ + reg_offset + 0x4,0x2e6},\
    {_vfe_base_ + reg_offset + 0x4,0x2e8},\
    {_vfe_base_ + reg_offset + 0x4,0x2ea},\
    {_vfe_base_ + reg_offset + 0x4,0x1ec},\
    {_vfe_base_ + reg_offset + 0x4,0x2ed},\
    {_vfe_base_ + reg_offset + 0x4,0x2ef},\
    {_vfe_base_ + reg_offset + 0x4,0x1f1},\
    {_vfe_base_ + reg_offset + 0x4,0x2f2},\
    {_vfe_base_ + reg_offset + 0x4,0x2f4},\
    {_vfe_base_ + reg_offset + 0x4,0x1f6},\
    {_vfe_base_ + reg_offset + 0x4,0x2f7},\
    {_vfe_base_ + reg_offset + 0x4,0x2f9},\
    {_vfe_base_ + reg_offset + 0x4,0x1fb},\
    {_vfe_base_ + reg_offset + 0x4,0x2fc},\
    {_vfe_base_ + reg_offset + 0x4,0x2fe,}

struct camera_hw_reg_array hw_vfe_hw_dmi3_regs[] = {
	vfe_hw_dmi3(VFE_BASE,0x918)
};

void camera_hw_reg_set_dmi_0(int len,uint32_t reg_offset){
	int i;
	for(i =0 ;i < len/4; i++){
		msm_camera_io_w(0x0,VFE_BASE + reg_offset);
		msm_camera_io_w(0x0,VFE_BASE + reg_offset+0x4);
	}
}

void vfe_send_hw_cfg(){
	msm_camera_io_w(0x115a6817,VFE_BASE + 0x18);
	mdelay(20);
	msm_camera_io_w(0x10000000,VFE_BASE + 0x688);
    msm_camera_io_w(0x3,VFE_BASE + 0x888);
    msm_camera_io_w(0xb,VFE_BASE + 0x440);

    ///linerization
    vfe_wite_mb(0x101,0x0);
	msm_hw_init(&hw_vfe_hw_dmi0_regs[0],
		sizeof(hw_vfe_hw_dmi0_regs) / sizeof(hw_vfe_hw_dmi0_regs[0]));
	vfe_wite_mb(0x100,0x0);

	msm_hw_init(&hw_vfe_start_11_d_init_regs[0],
		sizeof(hw_vfe_start_11_d_init_regs) / sizeof(hw_vfe_start_11_d_init_regs[0]));

    // mesh_rolloff40
    vfe_wite_mb(0x103,0x0);
	msm_hw_init(&hw_vfe_hw_dmi1_regs[0],
		sizeof(hw_vfe_hw_dmi1_regs) / sizeof(hw_vfe_hw_dmi1_regs[0]));
	vfe_wite_mb(0x100,0x0);

    msm_hw_init(&hw_vfe0_d_init_regs[0],
		sizeof(hw_vfe0_d_init_regs) / sizeof(hw_vfe0_d_init_regs[0]));

    msm_camera_io_w(0x1,VFE_BASE + 0xae4);

    msm_hw_init(&hw_vfe1_d_init_regs[0],
		sizeof(hw_vfe1_d_init_regs) / sizeof(hw_vfe1_d_init_regs[0]));

    msm_camera_io_w(0x4,VFE_BASE + 0x30);

    // vfe_wite_mb(0x117,0x0);
  //  	msm_hw_init(&hw_vfe_hw_dmi2_regs[0],
		// sizeof(hw_vfe_hw_dmi2_regs) / sizeof(hw_vfe_hw_dmi2_regs[0]));
    // vfe_wite_mb(0x100,0x0);

    msm_camera_io_w(0x0,VFE_BASE + 0x30);
    msm_camera_io_w(0x13b40,VFE_BASE + 0xae4);

    msm_hw_init(&hw_vfe2_d_init_regs[0],
		sizeof(hw_vfe2_d_init_regs) / sizeof(hw_vfe2_d_init_regs[0]));

    msm_camera_io_w(0x8000,VFE_BASE + 0x14);

    // vfe_wite_mb(0x111,0x0);
 //    camera_hw_reg_set_dmi_0(1024,0x918);
 //    //vfe_wite_mb(0x0,0x100);
  //   msm_camera_io_w(0x100,VFE_BASE + 0x914);
	// msm_camera_io_w(0x0,VFE_BASE + 0x910);

    msm_camera_io_w(0x30000000,VFE_BASE + 0x8f4);
    msm_camera_io_w(0x4bf65f,VFE_BASE + 0x8f8);

    msm_camera_io_w(0x0,VFE_BASE + 0x14);
    msm_camera_io_w(0x40000,VFE_BASE + 0x14);

   //  vfe_wite_mb(0x107,0x0);
 //    camera_hw_reg_set_dmi_0(1024,0x918);
 //   // vfe_wite_mb(0x0,0x100);
  //   msm_camera_io_w(0x100,VFE_BASE + 0x914);
	// msm_camera_io_w(0x0,VFE_BASE + 0x910);

  //   vfe_wite_mb(0x108,0x0);
 //    camera_hw_reg_set_dmi_0(1024,0x918);
 //    //vfe_wite_mb(0x0,0x100);
   ///  msm_camera_io_w(0x100,VFE_BASE + 0x914);
	/// msm_camera_io_w(0x0,VFE_BASE + 0x910);

    msm_camera_io_w(0x0,VFE_BASE + 0x8bc);
    msm_camera_io_w(0x4c765f,VFE_BASE + 0x8c0);

    msm_camera_io_w(0x0,VFE_BASE + 0x14);

    msm_hw_init(&hw_vfe3_d_init_regs[0],
		sizeof(hw_vfe3_d_init_regs) / sizeof(hw_vfe3_d_init_regs[0]));

    //gamma
    vfe_wite_mb(0x109,0x0);
    msm_hw_init(&hw_vfe_hw_dmi3_regs[0],
		sizeof(hw_vfe_hw_dmi3_regs) / sizeof(hw_vfe_hw_dmi3_regs[0]));
    vfe_wite_mb(0x100,0x0);

    vfe_wite_mb(0x10b,0x0);
    msm_hw_init(&hw_vfe_hw_dmi3_regs[0],
		sizeof(hw_vfe_hw_dmi3_regs) / sizeof(hw_vfe_hw_dmi3_regs[0]));
    vfe_wite_mb(0x100,0x0);

    vfe_wite_mb(0x10d,0x0);
    msm_hw_init(&hw_vfe_hw_dmi3_regs[0],
		sizeof(hw_vfe_hw_dmi3_regs) / sizeof(hw_vfe_hw_dmi3_regs[0]));
    vfe_wite_mb(0x100,0x0);

    msm_camera_io_w(0x0,VFE_BASE + 0x638);

    msm_camera_io_w(0xf0000000,VFE_BASE + 0x66c);

    msm_hw_init(&hw_vfe4_d_init_regs[0],
		sizeof(hw_vfe4_d_init_regs) / sizeof(hw_vfe4_d_init_regs[0]));
}

void vfe_cfg_start(){

	vfe_wite_mb(0x102,0x0);
	msm_hw_init(&hw_vfe_hw_dmi0_regs[0],
		sizeof(hw_vfe_hw_dmi0_regs) / sizeof(hw_vfe_hw_dmi0_regs[0]));
	vfe_wite_mb(0x100,0x0);

	msm_hw_init(&hw_vfe_start_d_init_regs[0],
		sizeof(hw_vfe_start_d_init_regs) / sizeof(hw_vfe_start_d_init_regs[0]));
}

void msm_isp_close_node(struct vfe_device *vfe_dev){
	uint32_t val;

	msm_vfe40_config_irq(vfe_dev, 0, 0x81,MSM_ISP_IRQ_DISABLE);
	val = msm_camera_io_r(VFE_BASE + 0x464);
	msm_camera_io_w_mb(val & ~(1 << 8), VFE_BASE + 0x464);
	msm_camera_io_w_mb(0x0,VFE_BASE + 0x2F4);

	//msm_camera_clk_enable(0);

}

void msm_isp_clock_disable(){

	msm_camera_clk_enable(0);

}
