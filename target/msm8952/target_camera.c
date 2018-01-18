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
#include <dev/fbcon.h>
#include <dev/gpio.h>
#include "ov8865_lib.h"
#include "psci.h"

#define DISPLAY_ID 0
#define CCI_HW_VERSION_ADDR                                         0x00000000
#define CCI_RESET_CMD_ADDR                                          0x00000004
#define CCI_RESET_CMD_RMSK                                          0x0f73f3f7
#define CCI_M0_RESET_RMSK                                                0x3F1
#define CCI_M1_RESET_RMSK                                              0x3F001
#define CCI_QUEUE_START_ADDR                                        0x00000008
#define CCI_SET_CID_SYNC_TIMER_ADDR                                 0x00000010
#define CCI_SET_CID_SYNC_TIMER_OFFSET                               0x00000004
#define CCI_I2C_M0_SCL_CTL_ADDR                                     0x00000100
#define CCI_I2C_M0_SDA_CTL_0_ADDR                                   0x00000104
#define CCI_I2C_M0_SDA_CTL_1_ADDR                                   0x00000108
#define CCI_I2C_M0_SDA_CTL_2_ADDR                                   0x0000010c
#define CCI_I2C_M0_READ_DATA_ADDR                                   0x00000118
#define CCI_I2C_M0_MISC_CTL_ADDR                                    0x00000110
#define CCI_I2C_M0_READ_BUF_LEVEL_ADDR                              0x0000011C
#define CCI_HALT_REQ_ADDR                                           0x00000034
#define CCI_M0_HALT_REQ_RMSK                                               0x1
#define CCI_M1_HALT_REQ_RMSK                                               0x2
#define CCI_I2C_M1_SCL_CTL_ADDR                                     0x00000200
#define CCI_I2C_M1_SDA_CTL_0_ADDR                                   0x00000204
#define CCI_I2C_M1_SDA_CTL_1_ADDR                                   0x00000208
#define CCI_I2C_M1_SDA_CTL_2_ADDR                                   0x0000020c
#define CCI_I2C_M1_MISC_CTL_ADDR                                    0x00000210
#define CCI_I2C_M0_Q0_CUR_WORD_CNT_ADDR                             0x00000304
#define CCI_I2C_M0_Q0_CUR_CMD_ADDR                                  0x00000308
#define CCI_I2C_M0_Q0_REPORT_STATUS_ADDR                            0x0000030c
#define CCI_I2C_M0_Q0_EXEC_WORD_CNT_ADDR                            0x00000300
#define CCI_I2C_M0_Q0_LOAD_DATA_ADDR                                0x00000310
#define CCI_IRQ_MASK_0_ADDR                                         0x00000c04
#define CCI_IRQ_MASK_0_RMSK                                         0x7fff7ff7
#define CCI_IRQ_CLEAR_0_ADDR                                        0x00000c08
#define CCI_IRQ_STATUS_0_ADDR                                       0x00000c0c
#define CCI_IRQ_STATUS_0_I2C_M1_Q0Q1_HALT_ACK_BMSK                   0x4000000
#define CCI_IRQ_STATUS_0_I2C_M0_Q0Q1_HALT_ACK_BMSK                   0x2000000
#define CCI_IRQ_STATUS_0_RST_DONE_ACK_BMSK                           0x1000000
#define CCI_IRQ_STATUS_0_I2C_M1_Q1_REPORT_BMSK                        0x100000
#define CCI_IRQ_STATUS_0_I2C_M1_Q0_REPORT_BMSK                         0x10000
#define CCI_IRQ_STATUS_0_I2C_M1_RD_DONE_BMSK                            0x1000
#define CCI_IRQ_STATUS_0_I2C_M0_Q1_REPORT_BMSK                           0x100
#define CCI_IRQ_STATUS_0_I2C_M0_Q0_REPORT_BMSK                            0x10
#define CCI_IRQ_STATUS_0_I2C_M0_ERROR_BMSK                          0x18000EE6
#define CCI_IRQ_STATUS_0_I2C_M1_ERROR_BMSK                          0x60EE6000
#define CCI_IRQ_STATUS_0_I2C_M0_RD_DONE_BMSK                               0x1
#define CCI_IRQ_GLOBAL_CLEAR_CMD_ADDR                               0x00000c00

#define DEBUG_TOP_REG_START                                                0x0
#define DEBUG_TOP_REG_COUNT                                                 14
#define DEBUG_MASTER_REG_START                                           0x100
#define DEBUG_MASTER_REG_COUNT                                               8
#define DEBUG_MASTER_QUEUE_REG_START                                     0x300
#define DEBUG_MASTER_QUEUE_REG_COUNT                                         6
#define DEBUG_INTR_REG_START                                             0xC00
#define DEBUG_INTR_REG_COUNT                                                 7

#define msm_camera_io_w_mb(d,a) writel(d,a)
#define msm_camera_io_w(d,a)    writel(d,a)
#define msm_camera_io_r(a)      readl(a)
#define msm_camera_io_r_mb(a)   readl(a)

#define CCI_DEV_BASE 0x1b0c000

#define MAX_POLL_COUNT 100

#define CCI_I2C_QUEUE_0_SIZE 64
#define CCI_I2C_Q0_SIZE_128W 128
#define CCI_I2C_QUEUE_1_SIZE 16
#define MSM_CCI_WRITE_DATA_PAYLOAD_SIZE_10 10
#define MSM_CCI_WRITE_DATA_PAYLOAD_SIZE_11 11

/*camera sensor config*/
#define GPIO_RESET_NUM            40 //36(0) 38(1) 40(2)
#define GPIO_STANDBY_NUM          39 //35(0) 50(1) 39(2)
#define GPIO_MCLK_NUM             28 //26(0) 27(1) 28(2)
#define SENSOR_SLAVE_ADDR         0x6c
#define SENSOR_ID_REG_ADDR        0x300b
#define SENSOR_ID                 0x8865

#define NUM_QUEUES 2
#define NUM_MASTER 2

struct msm_camera_i2c_reg_array init_reg_array[] = INIT0_REG_ARRAY;
struct msm_camera_i2c_reg_setting init_settings_array = {
	init_reg_array,
	327,
	MSM_CAMERA_I2C_WORD_ADDR,
	MSM_CAMERA_I2C_BYTE_DATA,
	60,
};

struct msm_camera_i2c_reg_array res0_reg_array[] = RES0_REG_ARRAY;
struct msm_camera_i2c_reg_setting res0_settings_reg_array = {
	res0_reg_array,
	106,
	MSM_CAMERA_I2C_WORD_ADDR,
	MSM_CAMERA_I2C_BYTE_DATA,
	0,
};

struct msm_camera_i2c_reg_array start_reg_array[] = START_REG_ARRAY;
struct msm_camera_i2c_reg_setting start_settings_reg_array = {
	start_reg_array,
	1,
	MSM_CAMERA_I2C_WORD_ADDR,
	MSM_CAMERA_I2C_BYTE_DATA,
	0,
};

struct msm_camera_i2c_reg_array stop_reg_array[] = STOP_REG_ARRAY;
struct msm_camera_i2c_reg_setting stop_settings_reg_array = {
	stop_reg_array,
	1,
	MSM_CAMERA_I2C_WORD_ADDR,
	MSM_CAMERA_I2C_BYTE_DATA,
	0,
};

struct msm_camera_i2c_reg_array group_on_reg_array[] = GROUPON_REG_ARRAY;
struct msm_camera_i2c_reg_setting group_on_settings_reg_array = {
	group_on_reg_array,
	1,
	MSM_CAMERA_I2C_WORD_ADDR,
	MSM_CAMERA_I2C_BYTE_DATA,
	0,
};

struct msm_camera_i2c_reg_array group_off_reg_array[] = GROUPOFF_REG_ARRAY;
struct msm_camera_i2c_reg_setting group_off_settings_reg_array = {
	group_off_reg_array,
	2,
	MSM_CAMERA_I2C_WORD_ADDR,
	MSM_CAMERA_I2C_BYTE_DATA,
	0,
};
struct msm_camera_i2c_reg_array start_on_reg_array[] = START_REG_ARRAY_ON;
struct msm_camera_i2c_reg_setting start_on_settings_reg_array = {
	start_on_reg_array,
	7,
	MSM_CAMERA_I2C_WORD_ADDR,
	MSM_CAMERA_I2C_BYTE_DATA,
	0,
};

struct msm_camera_i2c_reg_array Test_reg_array[] = TEST;
struct msm_camera_i2c_reg_setting test_settings_reg_array = {
	Test_reg_array,
	1,
	MSM_CAMERA_I2C_WORD_ADDR,
	MSM_CAMERA_I2C_BYTE_DATA,
	0,
};

struct msm_camera_cci_master_info{
	uint32_t status;
	uint32_t q_free[NUM_QUEUES];
	uint32_t q_lock[NUM_QUEUES];
	uint8_t reset_pending;
	uint32_t done_pending[NUM_QUEUES];
};
struct msm_camera_cci_master_info cci_master_info[NUM_MASTER];

enum msm_cci_clk_freq{
	I2C_STANDARD_MODE_100KHZ,
	I2C_FAST_MODE_400KHZ,
	I2C_CUSTOM_MODE,
	I2C_FAST_PLUS_MODE_1MHZ,
};

struct msm_cci_clk_params_t {
	uint16_t hw_thigh;
	uint16_t hw_tlow;
	uint16_t hw_tsu_sto;
	uint16_t hw_tsu_sta;
	uint16_t hw_thd_dat;
	uint16_t hw_thd_sta;
	uint16_t hw_tbuf;
	uint8_t hw_scl_stretch_en;
	uint8_t hw_trdhld;
	uint8_t hw_tsp;
	//uint32_t cci_clk_src;
};

/*100khz*/
struct msm_cci_clk_params_t  gcci_clk_params_100khz =
{
	78,
	114,
	28,
	28,
	10,
	77,
	118,
	0,
	6,
	1,
};
/*400khz*/
struct msm_cci_clk_params_t  gcci_clk_params_400khz =
{
	20,
	28,
	21,
	21,
	13,
	18,
	32,
	0,
	6,
	3,
};

static uint32_t ldo6[][11]=
{
	{
		LDOA_RES_TYPE, 6,
		KEY_SOFTWARE_ENABLE, 4, GENERIC_DISABLE,
		KEY_MICRO_VOLT, 4, 0,
		KEY_CURRENT, 4, 0,
	},

	{
		LDOA_RES_TYPE, 6,
		KEY_SOFTWARE_ENABLE, 4, GENERIC_ENABLE,
		KEY_MICRO_VOLT, 4, 1800000,
		KEY_CURRENT, 4, 150,
	},

};


static uint32_t ldo17[][11]=
{
	{
		LDOA_RES_TYPE, 17,
		KEY_SOFTWARE_ENABLE, 4, GENERIC_DISABLE,
		KEY_MICRO_VOLT, 4, 0,
		KEY_CURRENT, 4, 0,
	},
	{
		LDOA_RES_TYPE, 17,
		KEY_SOFTWARE_ENABLE, 4, GENERIC_ENABLE,
		KEY_MICRO_VOLT, 4, 2850000,
		KEY_CURRENT, 4, 40,
	},
};

static uint32_t ldo22[][11] =
{
	{
		LDOA_RES_TYPE, 22,
		KEY_SOFTWARE_ENABLE, 4, GENERIC_DISABLE,
		KEY_MICRO_VOLT, 4, 0,
		KEY_CURRENT, 4, 0,
	},
	{
		LDOA_RES_TYPE, 22,
		KEY_SOFTWARE_ENABLE, 4, GENERIC_ENABLE,
		KEY_MICRO_VOLT, 4, 2850000,
		KEY_CURRENT, 4, 40,
	},
};

static uint32_t ldo23[][11] =
{
	{
		LDOA_RES_TYPE, 23,
		KEY_SOFTWARE_ENABLE, 4, GENERIC_DISABLE,
		KEY_MICRO_VOLT, 4, 0,
		KEY_CURRENT, 4, 0,
	},
	{
		LDOA_RES_TYPE, 23,
		KEY_SOFTWARE_ENABLE, 4, GENERIC_ENABLE,
		KEY_MICRO_VOLT, 4, 1200000,
		KEY_CURRENT, 4, 40,
	},
};

void camera_clocks_enable(int enable){
	clk_get_set_enable("camss_top_ahb_clk_src",0,enable);

	clk_get_set_enable("gcc_camss_ispif_ahb_clk",61540000,enable);
	clk_get_set_enable("cci_clk_src",19200000,enable);
	clk_get_set_enable("gcc_camss_cci_ahb_clk",0,enable);
	clk_get_set_enable("gcc_camss_cci_clk",0,enable);
	clk_get_set_enable("gcc_camss_ahb_clk",0,enable);
	clk_get_set_enable("gcc_camss_top_ahb_clk",0,enable);

    clk_get_set_enable("gcc_smmu_cfg_clk",0,enable);
	clk_get_set_enable("gcc_apss_tcu_clk",0,enable);
}

int msm_cci_poll_irq(int irq_num,int master){
	uint32_t irq;
	uint32_t poll_count = 0;

	irq = msm_camera_io_r_mb(CCI_DEV_BASE+CCI_IRQ_STATUS_0_ADDR);
	mdelay(1);
	if(irq_num == CCI_IRQ_STATUS_0_RST_DONE_ACK_BMSK){  //RESET DONE IRQ
		while(!(irq & CCI_IRQ_STATUS_0_RST_DONE_ACK_BMSK)){
			irq = msm_camera_io_r_mb(CCI_DEV_BASE+CCI_IRQ_STATUS_0_ADDR);
			mdelay(1);
			poll_count++;
			if(poll_count > MAX_POLL_COUNT){
				dprintf(CRITICAL,"msm_cci_poll_irq: reset cci master irq = 0x%x timeout\n",irq);
				return -1;
			}
		}
		msm_camera_io_w_mb(irq,CCI_DEV_BASE+CCI_IRQ_CLEAR_0_ADDR);
		msm_camera_io_w_mb(0x1,CCI_DEV_BASE+CCI_IRQ_GLOBAL_CLEAR_CMD_ADDR);
	}

	if(irq_num == CCI_IRQ_STATUS_0_I2C_M0_RD_DONE_BMSK){
		while(!(irq & CCI_IRQ_STATUS_0_I2C_M0_RD_DONE_BMSK)){
			irq = msm_camera_io_r_mb(CCI_DEV_BASE+CCI_IRQ_STATUS_0_ADDR);
			mdelay(1);
			poll_count++;
			if(poll_count > MAX_POLL_COUNT) {
				dprintf(CRITICAL,"msm_cci_poll_irq: CCI_IRQ_STATUS_0_I2C_M0_RD_DONE_BMSK cci master irq = 0x%x timeout\n",irq);
				return -1;
			}
		}
		msm_camera_io_w_mb(irq,CCI_DEV_BASE+CCI_IRQ_CLEAR_0_ADDR);
		msm_camera_io_w_mb(0x1,CCI_DEV_BASE+CCI_IRQ_GLOBAL_CLEAR_CMD_ADDR);
	}

	if(irq_num == CCI_IRQ_STATUS_0_I2C_M0_Q1_REPORT_BMSK){
		while(!(irq & CCI_IRQ_STATUS_0_I2C_M0_Q1_REPORT_BMSK)){
			irq = msm_camera_io_r_mb(CCI_DEV_BASE+CCI_IRQ_STATUS_0_ADDR);
			mdelay(1);
			poll_count++;
			if(poll_count > MAX_POLL_COUNT){
				dprintf(CRITICAL,"msm_cci_poll_irq: CCI_IRQ_STATUS_0_I2C_M0_Q1_REPORT_BMSK cci master irq = 0x%x timeout\n",irq);
				return -1;
			}
		}
		cci_master_info[master].q_free[1] = 0;
		cci_master_info[master].status = 0;
		cci_master_info[master].done_pending[1] = 0;
	    msm_camera_io_w_mb(irq,CCI_DEV_BASE+CCI_IRQ_CLEAR_0_ADDR);
		msm_camera_io_w_mb(0x1,CCI_DEV_BASE+CCI_IRQ_GLOBAL_CLEAR_CMD_ADDR);
	}

	if(irq_num == CCI_IRQ_STATUS_0_I2C_M0_Q0_REPORT_BMSK){
		while(!(irq & CCI_IRQ_STATUS_0_I2C_M0_Q0_REPORT_BMSK)){
			irq = msm_camera_io_r_mb(CCI_DEV_BASE+CCI_IRQ_STATUS_0_ADDR);
			mdelay(1);
			poll_count++;
			if(poll_count > MAX_POLL_COUNT){
				dprintf(CRITICAL,"msm_cci_poll_irq: CCI_IRQ_STATUS_0_I2C_M0_Q0_REPORT_BMSK cci master irq = 0x%x timeout\n",irq);
				return -1;
			}
			cci_master_info[master].q_free[0] = 0;
			cci_master_info[master].status = 0;
			cci_master_info[master].done_pending[0] = 0;
			msm_camera_io_w_mb(irq,CCI_DEV_BASE+CCI_IRQ_CLEAR_0_ADDR);
			msm_camera_io_w_mb(0x1,CCI_DEV_BASE+CCI_IRQ_GLOBAL_CLEAR_CMD_ADDR);
		}
	}
	return 1;
}

int32_t  msm_cci_set_clk_param(int master,enum msm_cci_clk_freq mode){
	struct msm_cci_clk_params_t *clk_params = &gcci_clk_params_400khz;
	if(mode == I2C_STANDARD_MODE_100KHZ){
		//clk_params = &gcci_clk_params_100khz;
	}
	else if(mode == I2C_FAST_MODE_400KHZ){
		//clk_params = &gcci_clk_params_400khz;
	}

	if(master == 0){
		msm_camera_io_w_mb(clk_params->hw_thigh << 16 | clk_params->hw_tlow,CCI_DEV_BASE + CCI_I2C_M0_SCL_CTL_ADDR);
		msm_camera_io_w_mb(clk_params->hw_tsu_sto << 16 | clk_params->hw_tsu_sta,CCI_DEV_BASE + CCI_I2C_M0_SDA_CTL_0_ADDR);
		msm_camera_io_w_mb(clk_params->hw_thd_dat << 16 | clk_params->hw_thd_sta,CCI_DEV_BASE + CCI_I2C_M0_SDA_CTL_1_ADDR);
		msm_camera_io_w_mb(clk_params->hw_tbuf,CCI_DEV_BASE + CCI_I2C_M0_SDA_CTL_2_ADDR);
		msm_camera_io_w_mb(clk_params->hw_scl_stretch_en << 8 | clk_params->hw_trdhld << 4 | clk_params->hw_tsp,CCI_DEV_BASE + CCI_I2C_M0_MISC_CTL_ADDR);
	}
	else if(master == 1){
		msm_camera_io_w_mb(clk_params->hw_thigh << 16 | clk_params->hw_tlow,CCI_DEV_BASE + CCI_I2C_M1_SCL_CTL_ADDR);
		msm_camera_io_w_mb(clk_params->hw_tsu_sto << 16 | clk_params->hw_tsu_sta,CCI_DEV_BASE + CCI_I2C_M1_SDA_CTL_0_ADDR);
		msm_camera_io_w_mb(clk_params->hw_thd_dat << 16 | clk_params->hw_thd_sta,CCI_DEV_BASE + CCI_I2C_M1_SDA_CTL_1_ADDR);
		msm_camera_io_w_mb(clk_params->hw_tbuf, CCI_DEV_BASE + CCI_I2C_M1_SDA_CTL_2_ADDR);
		msm_camera_io_w_mb(clk_params->hw_scl_stretch_en << 8 | clk_params->hw_trdhld << 4 | clk_params->hw_tsp,CCI_DEV_BASE + CCI_I2C_M1_MISC_CTL_ADDR);
	}
	return 0;
}

static int32_t msm_cci_validate_queue(uint32_t len,int master,int queue){
	int32_t rc = 0;
	uint32_t read_val = 0;
	uint32_t reg_offset = master * 0x200 + queue * 0x100;
	uint32_t max_queue_size;
	int irq_wait;

	if(queue == 1){
		max_queue_size = CCI_I2C_QUEUE_1_SIZE;
		irq_wait = CCI_IRQ_STATUS_0_I2C_M0_Q1_REPORT_BMSK;
	}else{
		max_queue_size = CCI_I2C_QUEUE_0_SIZE;
		irq_wait = CCI_IRQ_STATUS_0_I2C_M0_Q0_REPORT_BMSK;
	}

    read_val = msm_camera_io_r_mb(CCI_DEV_BASE +CCI_I2C_M0_Q0_CUR_WORD_CNT_ADDR + reg_offset);
	mdelay(2);
	if ((read_val + len + 1) > max_queue_size) {
		uint32_t reg_val = 0;
		uint32_t report_val = CCI_I2C_REPORT_CMD | (1 << 8);

		msm_camera_io_w_mb(report_val,CCI_DEV_BASE + CCI_I2C_M0_Q0_LOAD_DATA_ADDR +reg_offset);
		read_val++;
		msm_camera_io_w_mb(read_val,CCI_DEV_BASE +CCI_I2C_M0_Q0_EXEC_WORD_CNT_ADDR + reg_offset);
		reg_val = 1 << ((master * 2) + queue);
		cci_master_info[master].done_pending[queue] = 1;
		msm_camera_io_w_mb(reg_val, CCI_DEV_BASE +CCI_QUEUE_START_ADDR);
		cci_master_info[master].q_free[queue] = 1;
		rc = msm_cci_poll_irq(irq_wait,master);
		if (rc < 0){
			dprintf(CRITICAL,"msm_cci_validate_queue: wait_for_completion_timeout \n");
			return rc;
		}
		cci_master_info[master].status = rc;
	}
	return rc;

}

int32_t msm_cci_write_i2c_queue(int master,int queue,uint32_t val,int32_t length){
	int32_t rc = 0;
	uint32_t reg_offset = master * 0x200 + queue * 0x100;

	rc = msm_cci_validate_queue(length,master, queue);
	if (rc < 0) {
		dprintf(CRITICAL,"msm_cci_write_i2c_queue failed \n");
		return rc;
	}
	msm_camera_io_w_mb(val, CCI_DEV_BASE + CCI_I2C_M0_Q0_LOAD_DATA_ADDR +reg_offset);
	return rc;
}

int32_t msm_cci_i2c_read(uint32_t address,int32_t length,uint32_t *read_val,
	                     unsigned int slave_address,int master,
	                     int queue,int addr_type){
	int32_t rc = 0;
	uint32_t max_queue_size;
	int32_t read_words = 0, exp_words = 0;
	int32_t index = 0, first_byte = 0;
	int i = 0;
	uint32_t val = 0;

	rc = msm_cci_set_clk_param(master,I2C_FAST_MODE_400KHZ);
	if(queue == 1){
		max_queue_size = CCI_I2C_QUEUE_1_SIZE;
	}else{
		max_queue_size = CCI_I2C_QUEUE_0_SIZE;
	}

	rc = msm_cci_validate_queue(max_queue_size,master,queue);
	if(rc < 0){
		dprintf(CRITICAL,"msm_cci_validate_queue failed\n");
		return rc;
	}

	val = CCI_I2C_SET_PARAM_CMD | (slave_address )<< 4 | 3 << 16 | 0 << 18;

	rc = msm_cci_write_i2c_queue(master,queue,val,length);
	if(rc < 0){
		dprintf(CRITICAL,"msm_cci_write_i2c_queue CCI_I2C_SET_PARAM_CMD failed \n");
		goto ERROR;
	}

	val = CCI_I2C_LOCK_CMD;
	rc = msm_cci_write_i2c_queue(master,queue,val,length);
	if(rc < 0){
		dprintf(CRITICAL,"msm_cci_write_i2c_queue CCI_I2C_LOCK_CMD failed \n");
		goto ERROR;
	}

	val = CCI_I2C_WRITE_DISABLE_P_CMD | (addr_type << 4);
	for (i = 0; i < addr_type; i++) {
		val |= ((address >> (i << 3)) & 0xFF)  << ((addr_type - i) << 3);
	}
	rc = msm_cci_write_i2c_queue(master,queue,val,length);
	if(rc < 0){
		dprintf(CRITICAL,"msm_cci_write_i2c_queue CCI_I2C_WRITE_DISABLE_P_CMD failed \n");
		goto ERROR;
	}

	val = CCI_I2C_READ_CMD | (2 << 4);
	rc = msm_cci_write_i2c_queue(master,queue,val,length);
	if(rc < 0){
		dprintf(CRITICAL,"msm_cci_write_i2c_queue CCI_I2C_READ_CMD failed \n");
		goto ERROR;
	}
	val = CCI_I2C_UNLOCK_CMD;
	rc = msm_cci_write_i2c_queue(master,queue,val,length);
	if(rc < 0){
		dprintf(CRITICAL,"msm_cci_write_i2c_queue CCI_I2C_UNLOCK_CMD failed \n");
		goto ERROR;
	}

	val = msm_camera_io_r_mb(CCI_DEV_BASE + CCI_I2C_M0_Q0_CUR_WORD_CNT_ADDR + master * 0x200 + queue * 0x100);
	msm_camera_io_w_mb(val, CCI_DEV_BASE + CCI_I2C_M0_Q0_EXEC_WORD_CNT_ADDR + master * 0x200 + queue * 0x100);

	val = 1 << ((master * 2) + queue);
	msm_camera_io_w_mb(val, CCI_DEV_BASE + CCI_QUEUE_START_ADDR);

	rc = msm_cci_poll_irq(CCI_IRQ_STATUS_0_I2C_M0_RD_DONE_BMSK,master);
	if(rc < 0){
		dprintf(CRITICAL,"msm_cci_i2c_read failed \n");
		goto ERROR;
	}
	read_words = msm_camera_io_r_mb(CCI_DEV_BASE + CCI_I2C_M0_READ_BUF_LEVEL_ADDR + master * 0x100);

	exp_words = ((2 / 4) + 1);
	if (read_words != exp_words){
		dprintf(CRITICAL,"msm_cci_i2c_read:read_words = %d, exp words = %d\n", read_words, exp_words);
		goto ERROR;
	}
	index = 0;
	first_byte = 0;
	char temp_data[2];
	do {
		val = msm_camera_io_r_mb(CCI_DEV_BASE + CCI_I2C_M0_READ_DATA_ADDR + master * 0x100);
		for (i = 0; (i < 4) && (index < 2); i++){
			if (!first_byte){
				first_byte++;
			}else{
				temp_data[index] = (val  >> (i * 8) & 0xFF);
				index++;
			}
		}
	}while (--read_words > 0);
	*read_val = (temp_data[0] << 8) | temp_data[1];
	return 1;

ERROR:
    return rc;
}

int32_t msm_cci_calc_cmd_len(uint32_t cmd_size,struct msm_camera_i2c_reg_array *i2c_cmd,uint32_t *pack,uint32_t payload_size){
	uint8_t i;
	uint32_t len = 0;
	uint8_t data_len = 0, addr_len = 0;
	uint8_t pack_max_len;
	//struct msm_camera_i2c_reg_setting *msg;
	struct msm_camera_i2c_reg_array *cmd = i2c_cmd;
	uint32_t size = cmd_size;

	*pack = 0;
	addr_len = 2;
	data_len = 1;
	len = data_len + addr_len;
	pack_max_len = size < (payload_size-len) ? size : (payload_size-len);

	for (i = 0; i < pack_max_len;){
		if (cmd->delay)
			break;
		if (cmd->reg_addr + 1 == (cmd+1)->reg_addr) {
			len += data_len;
			*pack += data_len;
		} else{
			break;
		}
		i += data_len;
		cmd++;
	}
	if (len > payload_size) {
		dprintf(CRITICAL,"Len error: %d\n", len);
		return -1;
	}
	len += 1;
	len = len/4 + 1;
	return len;
}

void msm_cci_load_report_cmd(int master,int queue)
{
	uint32_t reg_offset = master * 0x200 + queue * 0x100;
	uint32_t read_val = msm_camera_io_r_mb(CCI_DEV_BASE +
		CCI_I2C_M0_Q0_CUR_WORD_CNT_ADDR + reg_offset);
	uint32_t report_val = CCI_I2C_REPORT_CMD | (1 << 8);

	msm_camera_io_w_mb(report_val,
		CCI_DEV_BASE + CCI_I2C_M0_Q0_LOAD_DATA_ADDR +
		reg_offset);
	read_val++;

	msm_camera_io_w_mb(read_val, CCI_DEV_BASE +
		CCI_I2C_M0_Q0_EXEC_WORD_CNT_ADDR + reg_offset);
}

int32_t msm_cci_wait_report_cmd(int master,int queue)
{
	int irq_wait;
	uint32_t reg_val = 0;

	if(queue == 1){
		irq_wait = CCI_IRQ_STATUS_0_I2C_M0_Q1_REPORT_BMSK;
	}else{
		irq_wait = CCI_IRQ_STATUS_0_I2C_M0_Q0_REPORT_BMSK;
	}
	reg_val = 1 << ((master * 2) + queue);
	msm_cci_load_report_cmd(master, queue);
	msm_camera_io_w_mb(reg_val, CCI_DEV_BASE +
		CCI_QUEUE_START_ADDR);
	return msm_cci_poll_irq(irq_wait, queue);
}


int32_t msm_cci_process_full_q(int master,int queue)
{
	int32_t rc = 0;
	rc = msm_cci_wait_report_cmd(master, queue);
	if (rc < 0) {
		dprintf(CRITICAL,"msm_cci_process_full_q failed rc\n");
	}
	return rc;
}

void msm_cci_process_half_q(int master,int queue)
{
	uint32_t reg_val = 1 << ((master * 2) + queue);
	msm_cci_load_report_cmd(master, queue);
	cci_master_info[master].q_free[queue] = 1;
	msm_camera_io_w_mb(reg_val, CCI_DEV_BASE +CCI_QUEUE_START_ADDR);
}

int32_t msm_cci_get_queue_free_size(int master,int queue)
{
	uint32_t read_val = 0;
	uint32_t max_queue_size;
	uint32_t reg_offset = master * 0x200 + queue * 0x100;

	if(queue == 1){
		max_queue_size = CCI_I2C_QUEUE_1_SIZE;
	}else{
		max_queue_size = CCI_I2C_QUEUE_0_SIZE;
	}

	read_val = msm_camera_io_r_mb(CCI_DEV_BASE + CCI_I2C_M0_Q0_CUR_WORD_CNT_ADDR + reg_offset);
	dprintf(CRITICAL,"msm_cci_get_queue_free_size CCI_I2C_M0_Q0_CUR_WORD_CNT_ADDR %d max %d\n", read_val,max_queue_size);
	return (max_queue_size -read_val);
}

int32_t msm_cci_transfer_end(int master,int queue)
{
	int32_t rc = 0;
	uint32_t val = 0;
	uint32_t reg_val = 1 << ((master * 2) + queue);
	uint32_t reg_offset = master * 0x200 + queue * 0x100;
	int irq_wait;
	uint32_t read_val;
	uint32_t report_val;

	if(queue == 1){
		irq_wait = CCI_IRQ_STATUS_0_I2C_M0_Q1_REPORT_BMSK;
	}else{
		irq_wait = CCI_IRQ_STATUS_0_I2C_M0_Q0_REPORT_BMSK;
	}

	val = CCI_I2C_UNLOCK_CMD;
	rc = msm_cci_write_i2c_queue(master,queue,val,2);

	read_val = msm_camera_io_r_mb(CCI_DEV_BASE + CCI_I2C_M0_Q0_CUR_WORD_CNT_ADDR + reg_offset);
	report_val = CCI_I2C_REPORT_CMD | (1 << 8);
	msm_camera_io_w_mb(report_val,CCI_DEV_BASE + CCI_I2C_M0_Q0_LOAD_DATA_ADDR + reg_offset);
	read_val++;
	msm_camera_io_w_mb(read_val, CCI_DEV_BASE + CCI_I2C_M0_Q0_EXEC_WORD_CNT_ADDR + reg_offset);

	reg_val = 1 << ((master * 2) + queue);
	msm_camera_io_w_mb(reg_val, CCI_DEV_BASE + CCI_QUEUE_START_ADDR);
	rc = msm_cci_poll_irq(irq_wait,master);
	if (rc < 0){
		dprintf(CRITICAL,"msm_cci_transfer_end: wait_for_completion_timeout \n");
		return rc;
	}
	return rc;
}

int32_t msm_cci_data_queue(int master,int queue,uint32_t slave_address,uint16_t payload_size,struct msm_camera_i2c_reg_setting i2c_msg){
	int rc = 0;
	uint16_t i = 0, j = 0, k = 0, h = 0, len = 0;
	uint32_t max_queue_size,queue_size = 0;;
	uint32_t read_val = 0;
	uint32_t reg_offset;
	uint32_t val = 0;
	uint16_t reg_addr = 0;
	uint32_t cmd = 0;
	uint8_t data[12];
	struct msm_camera_i2c_reg_array *i2c_cmd = i2c_msg.reg_setting;
	uint16_t cmd_size = i2c_msg.size;

	if(queue == 1){
		max_queue_size = CCI_I2C_QUEUE_1_SIZE;
	}else{
		max_queue_size = CCI_I2C_QUEUE_0_SIZE;
	}

	reg_offset = master * 0x200 + queue * 0x100;

	msm_camera_io_w_mb(0,CCI_DEV_BASE + CCI_SET_CID_SYNC_TIMER_ADDR + 0);

	val = CCI_I2C_SET_PARAM_CMD | slave_address << 4 | 3 << 16 | 0 << 18;
	msm_camera_io_w_mb(val, CCI_DEV_BASE + CCI_I2C_M0_Q0_LOAD_DATA_ADDR + reg_offset);

    cci_master_info[master].q_free[queue] = 0;
	queue_size = max_queue_size/2;
	reg_addr = i2c_cmd->reg_addr;

	val = CCI_I2C_LOCK_CMD;
	rc = msm_cci_write_i2c_queue(master,queue,val,2);
	if(rc < 0){
		dprintf(CRITICAL,"msm_cci_data_queue CCI_I2C_LOCK_CMD failed \n");
		return rc;
	}

	while (cmd_size){
		uint32_t pack = 0;
		len = msm_cci_calc_cmd_len(cmd_size,i2c_cmd,&pack,payload_size);
		if (len <= 0) {
			dprintf(CRITICAL,"msm_cci_calc_cmd_len failed \n");
			return -1;
		}
		read_val = msm_camera_io_r_mb(CCI_DEV_BASE + CCI_I2C_M0_Q0_CUR_WORD_CNT_ADDR + reg_offset);

		if ((read_val + len + 1) > queue_size){
			mdelay(2);
			if ((read_val + len + 1) > max_queue_size){
				mdelay(2);
				rc = msm_cci_process_full_q(master, queue);
				if (rc < 0) {
					dprintf(CRITICAL,"msm_cci_data_queue failed line\n");
					return rc;
				}
				continue;
			}
			msm_cci_process_half_q(master, queue);
		}
		i = 0;
		data[i++] = CCI_I2C_WRITE_CMD;
		reg_addr = i2c_cmd->reg_addr;
		if (i2c_msg.addr_type == MSM_CAMERA_I2C_BYTE_ADDR)
			data[i++] = reg_addr;
		else {
			data[i++] = (reg_addr & 0xFF00) >> 8;
			data[i++] = reg_addr & 0x00FF;
	    }
	    do {
           if (i2c_msg.data_type == MSM_CAMERA_I2C_BYTE_DATA) {
                data[i++] = i2c_cmd->reg_data;
                reg_addr++;
            }else{
                 if(((i + 1) < payload_size) || ((i + 1) == payload_size)){
                    data[i++] = (i2c_cmd->reg_data & 0xFF00) >> 8;
                    data[i++] = i2c_cmd->reg_data & 0x00FF;
                    reg_addr++;
                }else
                    break;
            }
            i2c_cmd++;
            --cmd_size;
	    }while ((pack--) &&(cmd_size > 0) && (i <= payload_size));
	    data[0] |= ((i-1) << 4);
	    len = ((i-1)/4) + 1;
	    read_val = msm_camera_io_r_mb(CCI_DEV_BASE +
			CCI_I2C_M0_Q0_CUR_WORD_CNT_ADDR + reg_offset);
	    for (h = 0, k = 0; h < len; h++) {
            cmd = 0;
            for (j = 0; (j < 4 && k < i); j++){
                cmd |= (data[k++] << (j * 8));
            }
            msm_camera_io_w_mb(cmd, CCI_DEV_BASE + CCI_I2C_M0_Q0_LOAD_DATA_ADDR + master * 0x200 + queue * 0x100);
            read_val += 1;
            msm_camera_io_w_mb(read_val, CCI_DEV_BASE + CCI_I2C_M0_Q0_EXEC_WORD_CNT_ADDR + reg_offset);
	    }
	}
	rc = msm_cci_transfer_end(master, queue);
	return rc;
}

int msm_cci_i2c_write(int master,int queue,uint32_t slave_address,struct msm_camera_i2c_reg_setting i2c_msg){
	int rc = 0;
	uint32_t max_queue_size;

	if(queue == 1){
		max_queue_size = CCI_I2C_QUEUE_1_SIZE;
	}else{
		max_queue_size = CCI_I2C_QUEUE_0_SIZE;
	}

	rc = msm_cci_set_clk_param(master,I2C_FAST_MODE_400KHZ);

	rc = msm_cci_validate_queue(max_queue_size-1,master,queue);
	if(rc < 0){
		dprintf(CRITICAL,"i2c_write:msm_cci_validate_queue failed\n");
		return rc;
	}

	rc = msm_cci_data_queue(master,queue,slave_address,MSM_CCI_WRITE_DATA_PAYLOAD_SIZE_11,i2c_msg);
	if(rc < 0){
		dprintf(CRITICAL,"i2c_write:msm_cci_data_queue failed\n");
	}
	return rc;
}

int msm_cci_init(int master){
	int rc = 0;
	camera_clocks_enable(1);

	msm_camera_io_w_mb(0,CCI_DEV_BASE+CCI_IRQ_MASK_0_ADDR);

	if(master == 0)
		msm_camera_io_w_mb(CCI_M0_RESET_RMSK,CCI_DEV_BASE + CCI_RESET_CMD_ADDR);
	else
		msm_camera_io_w_mb(CCI_M1_RESET_RMSK,CCI_DEV_BASE + CCI_RESET_CMD_ADDR);

	rc = msm_cci_poll_irq(CCI_IRQ_STATUS_0_RST_DONE_ACK_BMSK,0);
	return rc;
}

void msm_cci_release(){
	camera_clocks_enable(0);
}

void camera_cci_config(int master){
	gpio_tlmm_config(29,1,GPIO_OUTPUT,GPIO_PULL_UP,GPIO_2MA,1);
	mdelay(2);
	gpio_tlmm_config(30,1,GPIO_OUTPUT,GPIO_PULL_UP,GPIO_2MA,1);
	mdelay(2);

	gpio_tlmm_config(31,1,GPIO_OUTPUT,GPIO_PULL_UP,GPIO_2MA,1);
	mdelay(2);
	gpio_tlmm_config(32,1,GPIO_OUTPUT,GPIO_PULL_UP,GPIO_2MA,1);
}

void camera_mclk_config(int clk_num,int enable){
	if(clk_num == 1){
		clk_get_set_enable("mclk1_clk_src",24000000,enable);
		clk_get_set_enable("gcc_camss_mclk1_clk",0,enable);
	}else if(clk_num == 0){
		clk_get_set_enable("mclk0_clk_src",24000000,enable);
		clk_get_set_enable("gcc_camss_mclk0_clk",0,enable);
	}else if(clk_num == 2){
		clk_get_set_enable("mclk2_clk_src",24000000,enable);
		clk_get_set_enable("gcc_camss_mclk2_clk",0,enable);
	}
}

void camera_power_up(){
	//mclk
	gpio_tlmm_config(GPIO_MCLK_NUM,1,GPIO_OUTPUT,GPIO_NO_PULL,GPIO_2MA,1); //MCLK CONFIG
	mdelay(2);

	//RESET STANDBY
	gpio_tlmm_config(GPIO_RESET_NUM,0,GPIO_OUTPUT,GPIO_NO_PULL,GPIO_2MA,1); //RESET
	mdelay(2);
	gpio_tlmm_config(GPIO_STANDBY_NUM,0,GPIO_OUTPUT,GPIO_NO_PULL,GPIO_2MA,1); //STANDBY

	gpio_tlmm_config(47,0,GPIO_OUTPUT,GPIO_NO_PULL,GPIO_2MA,1);
	mdelay(2);
	gpio_set_dir(47,0);

    //gpio_tlmm_config(54,0,GPIO_INPUT,GPIO_NO_PULL,GPIO_2MA,1);
	//ldo enable
	rpm_send_data(&ldo6[1][0],36,RPM_REQUEST_TYPE);  //1.8v DOVDD
	rpm_send_data(&ldo17[1][0],36,RPM_REQUEST_TYPE); //2.8V AFDVDD
	rpm_send_data(&ldo22[1][0],36,RPM_REQUEST_TYPE); //2.8V AVDD
	rpm_send_data(&ldo23[1][0],36,RPM_REQUEST_TYPE); //1.2V DVDD

	//mclk set&enable 24000000
	camera_mclk_config(2,1); //mclk 2
	mdelay(1);

	//STANDBY low
	gpio_set_dir(GPIO_STANDBY_NUM,0);
	mdelay(1);
	//RESET low
	gpio_set_dir(GPIO_RESET_NUM,0);
	mdelay(1);

	//STANDBY HIGH
	gpio_set_dir(GPIO_STANDBY_NUM,2);
	mdelay(1);
	//RESET HIGH
	gpio_set_dir(GPIO_RESET_NUM,2);
	mdelay(1);
}

int camera_check_id(){
	int rc = 0;
	uint32_t read_val = 0;
	rc = msm_cci_i2c_read(SENSOR_ID_REG_ADDR,2,&read_val,(SENSOR_SLAVE_ADDR >> 1),0,1,2);
	if(rc < 0){
		dprintf(CRITICAL,"msm_cci_i2c_read  failed\n");
		goto error;
	}
	dprintf(CRITICAL,"msm_cci_i2c_read  read_val = 0x%x\n",read_val);
	if(read_val == SENSOR_ID){
		dprintf(CRITICAL,"check camera id success! sensor is ov8865\n");
	}else{
		dprintf(CRITICAL,"check camera id failed! sensor is not ov8865\n");
		rc = -1;
	}
error:
    return rc;
}

int camera_sensor_start_on(){
	int rc;
	rc = msm_cci_i2c_write(0,0,(SENSOR_SLAVE_ADDR >> 1),start_on_settings_reg_array);
	return rc;
}

int camera_sensor_stop(){
	int rc;
	rc = msm_cci_i2c_write(0,0,(SENSOR_SLAVE_ADDR >> 1),stop_settings_reg_array);
	return rc;
}

int camera_sensor_start(){
	int rc;
	rc = msm_cci_i2c_write(0,0,(SENSOR_SLAVE_ADDR >> 1),start_settings_reg_array);
	return rc;
}

int camera_sensot_reg_init(){
	int rc;
	rc = msm_cci_i2c_write(0,0,(SENSOR_SLAVE_ADDR >> 1),res0_settings_reg_array);
	return rc;
}

int camera_sensor_param_init(){
	int rc;
	rc = msm_cci_i2c_write(0,0,(SENSOR_SLAVE_ADDR >> 1),init_settings_array);
	mdelay(2);
	rc = camera_sensot_reg_init();
	return rc;
}


void camera_clk_disable(){
	clk_get_set_enable("vfe1_clk_src",266670000,0);
	clk_get_set_enable("gcc_camss_vfe1_clk",0,0);
	clk_get_set_enable("gcc_camss_csi_vfe1_clk",0,0);
	clk_get_set_enable("gcc_camss_vfe1_ahb_clk",0,0);
	clk_get_set_enable("gcc_camss_vfe1_axi_clk",0,0);
	clk_get_set_enable("gcc_camss_micro_ahb_clk",0,0);
	clk_get_set_enable("gcc_camss_ispif_ahb_clk",0,0);
	clk_get_set_enable("gcc_camss_top_ahb_clk",0,0);
	clk_get_set_enable("gcc_camss_ispif_ahb_clk",61540000,0);
	clk_get_set_enable("csi1phytimer_clk_src",200000000,0);
	clk_get_set_enable("gcc_camss_csi1phytimer_clk",0,0);
	clk_get_set_enable("camss_top_ahb_clk_src",0,0);
	clk_get_set_enable("gcc_camss_csi1phy_clk",0,0);
	clk_get_set_enable("gcc_camss_ahb_clk",0,0);
	clk_get_set_enable("gcc_camss_csi1_ahb_clk",0,0);
	clk_get_set_enable("csi1_clk_src",200000000,0);
	clk_get_set_enable("gcc_camss_csi1pix_clk",0,0);
	clk_get_set_enable("gcc_camss_csi1rdi_clk",0,0);
	clk_get_set_enable("gcc_camss_csi1_clk",0,0);
	clk_get_set_enable("gpll6_clk_src",0,0);
}

void target_camera_camera_release(struct vfe_device *vfe_dev){
	camera_sensor_stop();
	camera_mclk_config(2,0);

	msm_ispif_release();
	msm_isp_close_node(vfe_dev);
	camera_csid_release();
	camera_csiphy_release();
    msm_cci_release();

    msm_isp_clock_disable();
	camera_csiphy_clock_disable();
	camera_csid_clock_disable();
	msm_ispif_clock_disable();
	clk_get_set_enable("gpll6_clk_src",0,0);

	// rpm_send_data(&ldo6[0][0],36,RPM_REQUEST_TYPE);  //1.8v DOVDD
	// rpm_send_data(&ldo17[0][0],36,RPM_REQUEST_TYPE); //2.8V AFDVDD
	// rpm_send_data(&ldo22[0][0],36,RPM_REQUEST_TYPE); //2.8V AVDD
	// rpm_send_data(&ldo23[0][0],36,RPM_REQUEST_TYPE); //1.2V DVDD
	dprintf(INFO," target_camera_camera_release msm_clock_release exit.....\n");
}

#if SECONDARY_CPU_SUPPORT

void request_fastrvc_pause_working(bool showFlag)
{
	uint32_t reg_value = 0;
	reg_value = readl(FRVC_CAMERA_STATUS_REG);
	reg_value &= 0xFF00FFFF;

	if(showFlag)
		reg_value |= (KERNEL_REQUEST_PAUSE_FRVC << 16);
	else
		reg_value |= (KERNEL_REQUEST_REGAIN_FRVC << 16);

	writel(reg_value,FRVC_CAMERA_STATUS_REG);
	dprintf(INFO,"Paused show FastRVC screen ... (%s) \n",showFlag ? "TRUE" : "FLASE");
}

bool query_fastrvc_working_status()
{
	uint32_t reg_value = 0;
	uint8_t  display_status = 0;

	reg_value = readl(FRVC_CAMERA_STATUS_REG);
    display_status = ( reg_value >> 8 ) & 0xFF;
	return (FRVC_DISPLAY_IS_WORKING & display_status);
}

/*
*******************************************************************************
** Checks if GPIO or equivalent trigger to enable early camera is set to ON
** If this function retuns FALSE, only animated splash may be shown.
** This is also a check to see if early-camera/animated splash can exit
** if gpio == 1, it is ON
** if gpio == 0, it is OFF
*******************************************************************************
 */
bool is_reverse_camera_on()
{
	if(1 == gpio_get(37)){
		gpio_set_dir(47,2);
		return true; 
	}
	gpio_set_dir(47,0);
	return false;	
}

void early_camera_fastrvc_thread(void)
{
	int i = 0;
	int ret = 0;
	uint32_t reg_value = 0;
	unsigned char camera_status = 0;
	unsigned char display_status = 0;
	unsigned char notify_status = 0;
	unsigned char kernel_notify_lk_set_pingpong_status = 0;
	bool camera_init_done = false;

	uint32_t irq0 = 0;
	uint32_t irq1 = 0;
	struct vfe_device vfe_dev;

	dprintf(CRITICAL,"target_early_camera_init Start.\n");

	ret = camera_csid_init();
	if(ret < 0){
		dprintf(CRITICAL,"camera_csid_init failed\n");
		goto thread_exit;
	}
	camera_csiphy_init();
	camera_power_up();
	mdelay(2);
	camera_cci_config(0); //master 0
	mdelay(2);

	ret = msm_cci_init(0); //master 0
	if(ret < 0){
		dprintf(CRITICAL,"msm cci init failed.....\n");
		camera_init_done = false;
		goto msm_cci_init_failed;
	}
	ret = camera_check_id();
	if(ret < 0){
		dprintf(CRITICAL,"camera_check_id failed\n");
		camera_init_done = false;
		goto camera_check_id_failed;
	}
	ret  = camera_sensor_param_init();
	if(ret < 0){
		dprintf(CRITICAL,"camera_sensor_param_init  failed\n");
		camera_init_done = false;
		goto camera_sensor_set_param_failed;
	}
	camera_sensor_start_on();

	vfe_dev.id = 1;
	ret = msm_isp_open_node(&vfe_dev);
	if(ret < 0){
		dprintf(CRITICAL,"camera_start_up:  msm_isp_open_node failed \n");
		camera_init_done = false;
		goto thread_exit;
	}

	msm_isp_vfe_start_stream(&vfe_dev);
	msm_ispif_init();
	msm_ispif_cfg();
	camera_sensor_start_on();
	mdelay(2);
	camera_sensor_start();
	vfe_cfg_start();

	camera_init_done = true;
	writel(0x0,FRVC_CAMERA_STATUS_REG);
	// Signal Kernel early camera is active.
	reg_value |= FRVC_CAMERA_IS_ENABLED;
	writel(reg_value,FRVC_CAMERA_STATUS_REG);

	dprintf(CRITICAL, "Waiting for display init to complete for FastRVC\n");
	while((FALSE == target_display_is_init_done()) && (i < 300)) {
		mdelay(10); // delay for 10ms
		i++;
	}
	dprintf(CRITICAL, "Display init done! Entery FastRVC mode... \n");

	reg_value |= (FRVC_DISPLAY_IS_ENABLED << 8) | (KERNEL_REQUEST_REGAIN_FRVC << 16);
	writel(reg_value,FRVC_CAMERA_STATUS_REG);
	writel(0x00,VFE_PING_ADDR_FROM_KERNEL);
	writel(0x00,VFE_PONG_ADDR_FROM_KERNEL);
    i = 1;
	while(1)
	{
		reg_value      = readl(FRVC_CAMERA_STATUS_REG) & 0xFFFFFFFF;
		camera_status  = reg_value & 0xFF;
		display_status = (reg_value >> 8) & 0xFF;
		notify_status  = (reg_value >> 16) & 0xFF;
		kernel_notify_lk_set_pingpong_status = (reg_value >> 24) & 0xFF;

		if(is_reverse_camera_on()){
			if(!(display_status & FRVC_NOTIFY_ANDROID_SHOW_CAMERA)){
				reg_value &= 0xFFFF00FF;
				display_status |= FRVC_NOTIFY_ANDROID_SHOW_CAMERA;
				reg_value |= (display_status << 8);
				writel(reg_value,FRVC_CAMERA_STATUS_REG);
			}
		}else{
			if(display_status & FRVC_NOTIFY_ANDROID_SHOW_CAMERA){
				reg_value &= 0xFFFF00FF;
				display_status &= 0x0F;
				reg_value |= (display_status << 8);
				writel(reg_value,FRVC_CAMERA_STATUS_REG);
			}
		}

		if (KERNEL_REQUEST_EXIT_FRVC == notify_status) {
			//This value indicates kernel request LK to shutdown immediately
			break;
		}
		else if(KERNEL_REQUEST_STOP_FRVC == notify_status) {
			// If FastRVC not running, quit direct.
			// else, wait FastRVC complete and quit.
			if(false == is_reverse_camera_on()){
				dprintf(CRITICAL, "Early Camera start exit... \n");
				reg_value &= 0xFFFF00FF;
				reg_value |= (FRVC_DISPLAY_IS_ENABLED << 8);
				writel(reg_value,FRVC_CAMERA_STATUS_REG);
				break;
			}else{
				if(display_status != FRVC_NOTIFY_ANDROID_SHOW_CAMERA){
					reg_value &= 0xFFFF00FF;
					reg_value |= (FRVC_NOTIFY_ANDROID_SHOW_CAMERA << 8);
					writel(reg_value,FRVC_CAMERA_STATUS_REG);
				}
				mdelay(10);
				continue;
			}
		}else if(KERNEL_REQUEST_PAUSE_FRVC == notify_status){
			/* If Reverse signal not trigger, do not show the FastRVC */
			if(kernel_notify_lk_set_pingpong_status == PINGPONG_ALREADY_SET){
				if(readl(VFE_PING_ADDR_FROM_KERNEL) != 0x00){
					writel(0x00,VFE_PING_ADDR_FROM_KERNEL);
				}

				if(readl(VFE_PONG_ADDR_FROM_KERNEL) != 0x00){
					writel(0x00,VFE_PONG_ADDR_FROM_KERNEL);
				}
			/* Update Camera and Display status */
				if(FRVC_CAMERA_IS_WORKING == camera_status) {
				/* Notify Kernel that Camera in LK is shutdown */
					reg_value &= 0xFFFFFF00;
					reg_value |= (FRVC_CAMERA_IS_ENABLED);
					writel(reg_value,FRVC_CAMERA_STATUS_REG);
					dprintf(INFO,"FRVC_CAMERA_IS_ENABLED \n");
				}
				mdelay(10);
				continue;
			}
			if(readl(VFE_PING_ADDR_FROM_KERNEL) == 0x00 && readl(VFE_PONG_ADDR_FROM_KERNEL) == 0x00){
				mdelay(10);
				continue;
			}
		}else {
			/* Update Camera and Display status */
			if(FRVC_CAMERA_IS_WORKING != camera_status) {
				/* Notify Kernel that Camera in LK is shutdown */
				reg_value &= 0xFFFFFF00;
				reg_value |= (FRVC_CAMERA_IS_WORKING);
				writel(reg_value,FRVC_CAMERA_STATUS_REG);
			}

			if(!(FRVC_DISPLAY_IS_WORKING & display_status)) {
				// Notify  Kernel that Display is not using now..
				reg_value &= 0xFFFF00FF;
				display_status |= FRVC_DISPLAY_IS_WORKING;
				reg_value |= (display_status << 8);
				writel(reg_value,FRVC_CAMERA_STATUS_REG);
			}
		}
			irq0 = msm_vfe_poll_irq0();
		if(((irq0 >> 29) & 0x3)){
			irq1 = msm_vfe_poll_irq1();
			msm_vfe_irq_mask_cfg(irq0,irq1);
		}
		if(((irq0 >> 25) & 0xF)){
			irq1 = msm_vfe_poll_irq1();
			msm_vfe_irq_mask_cfg(irq0,irq1);
			msm_get_camera_frame(notify_status);
		}
		nv12_to_rgb888(is_reverse_camera_on());
	}

thread_exit:
    reg_value &= 0xFF00FFFF;
    reg_value |= (FRVC_NOTIFY_KERNEL_WAIT_EXIT << 16);
    writel(reg_value,FRVC_CAMERA_STATUS_REG);
    do{
        reg_value = readl(FRVC_CAMERA_STATUS_REG);
        notify_status  = (reg_value >> 16) & 0xFF;
        mdelay(10);
    }while(FRVC_NOTIFY_KERNEL_EXIT_DONE != notify_status);

    if(camera_init_done){
    	target_camera_camera_release(&vfe_dev);
    }

    dprintf(CRITICAL,"target_early_camera_init end !\n");

	reg_value &= 0xFFFF0000;
	reg_value |=(FRVC_DISPLAY_IS_DONE << 8) | FRVC_CAMERA_IS_DONE;
	writel(reg_value,FRVC_CAMERA_STATUS_REG);

	return;

camera_sensor_set_param_failed:
    if(camera_init_done == false)
    	camera_sensor_stop();
camera_check_id_failed:
    if(camera_init_done == false){
    	camera_mclk_config(2,0);
    	msm_cci_release();
    }
msm_cci_init_failed:
    if(camera_init_done == false){
    	camera_csid_release();
    	camera_csiphy_release();
    	camera_csiphy_clock_disable();
    	camera_csid_clock_disable();
    	clk_get_set_enable("gpll6_clk_src",0,0);
    }
	dprintf(INFO,"FastRVC Thread exit Now!\n");
}

#endif
