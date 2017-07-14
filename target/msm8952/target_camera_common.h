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

#ifndef  _TARGET_CAMERA_COMMON_H
#define  _TARGET_CAMERA_COMMON_H

#define MAX_POWER_CONFIG      12
#define CSI_EMBED_DATA        0x12
#define CSI_RESERVED_DATA_0   0x13
#define CSI_YUV422_8          0x1E
#define CSI_RAW8              0x2A
#define CSI_RAW10             0x2B
#define CSI_RAW12             0x2C
#define CSI_DECODE_6BIT         0
#define CSI_DECODE_8BIT         1
#define CSI_DECODE_10BIT        2
#define CSI_DECODE_12BIT        3
#define CSI_DECODE_DPCM_10_6_10 4
#define CSI_DECODE_DPCM_10_8_10 5
#define MAX_CID                 16
#define I2C_SEQ_REG_DATA_MAX    1024
#define I2C_REG_DATA_MAX       (8*1024)

enum msm_cci_i2c_cmd_type {
	CCI_I2C_SET_PARAM_CMD	 = 1,
	CCI_I2C_WAIT_CMD,
	CCI_I2C_WAIT_SYNC_CMD,
	CCI_I2C_WAIT_GPIO_EVENT_CMD,
	CCI_I2C_TRIG_I2C_EVENT_CMD,
	CCI_I2C_LOCK_CMD,
	CCI_I2C_UNLOCK_CMD,
	CCI_I2C_REPORT_CMD,
	CCI_I2C_WRITE_CMD,
	CCI_I2C_READ_CMD,
	CCI_I2C_WRITE_DISABLE_P_CMD,
	CCI_I2C_READ_DISABLE_P_CMD,
	CCI_I2C_WRITE_CMD2,
	CCI_I2C_WRITE_CMD3,
	CCI_I2C_REPEAT_CMD,
	CCI_I2C_INVALID_CMD,
};

enum msm_camera_i2c_reg_addr_type {
	MSM_CAMERA_I2C_BYTE_ADDR = 1,
	MSM_CAMERA_I2C_WORD_ADDR,
	MSM_CAMERA_I2C_3B_ADDR,
};

enum msm_camera_i2c_data_type {
	MSM_CAMERA_I2C_BYTE_DATA = 1,
	MSM_CAMERA_I2C_WORD_DATA,
	MSM_CAMERA_I2C_SET_BYTE_MASK,
	MSM_CAMERA_I2C_UNSET_BYTE_MASK,
	MSM_CAMERA_I2C_SET_WORD_MASK,
	MSM_CAMERA_I2C_UNSET_WORD_MASK,
	MSM_CAMERA_I2C_SET_BYTE_WRITE_MASK_DATA,
};

struct msm_camera_i2c_reg_array {
	uint16_t reg_addr;
	uint16_t reg_data;
	uint16_t delay;
};

struct msm_camera_i2c_reg_setting {
	struct msm_camera_i2c_reg_array *reg_setting;
	uint16_t size;
	enum msm_camera_i2c_reg_addr_type addr_type;
	enum msm_camera_i2c_data_type data_type;
	uint16_t delay;
};

struct msm_camera_csid_vc_cfg {
	unsigned char cid;
	unsigned char dt;
	unsigned char decode_format;
};

struct msm_camera_csid_lut_params {
	unsigned char num_cid;
	struct msm_camera_csid_vc_cfg vc_cfg;
	//struct msm_camera_csid_vc_cfg *vc_cfg[MAX_CID];
};

struct msm_camera_csid_params {
	unsigned char lane_cnt;
	unsigned short lane_assign;
	unsigned char phy_sel;
	unsigned int csi_clk;
	struct msm_camera_csid_lut_params lut_params;
	unsigned char csi_3p_sel;
};

struct msm_camera_csiphy_params {
	unsigned char lane_cnt;
	unsigned char settle_cnt;
	unsigned short lane_mask;
	unsigned char combo_mode;
	unsigned char csid_core;
	unsigned int csiphy_clk;
	unsigned char csi_3phase;
};

#endif
