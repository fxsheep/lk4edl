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

#define EARLY_CAMERA

//#define BRIDGE_REV_1  // For adashub rev 1 of TI 960 Bridge chip.
#define DISPLAY_ID 0 // Use dsi 0

#define MAX_REV_ID 2

struct camera_i2c_reg_array {
	unsigned short reg_addr;
	unsigned short reg_data;
	unsigned int delay;
};

struct i2c_config_data {
	struct camera_i2c_reg_array *i2c_regs;	// Array of i2c registers to be written
	unsigned int	size;					// Number of elements in the array
	unsigned int	i2c_slave_address;		// Slave address to use for the write
	unsigned int	i2c_num_bytes_address;// Number of bytes used for i2c address
	unsigned int	i2c_num_bytes_data;	// Number of bytes used for i2c data
	unsigned int	i2c_revision_id_num;	// Number of revision id's to check
	unsigned int	i2c_revision_id_val[MAX_REV_ID];	// Expected revision id's of the device
	unsigned int	i2c_revision_id_reg;	// Address of the expected revision id of the device
};

int get_cam_data(struct i2c_config_data **cam_data);
int early_camera_init(void);
void target_early_camera_init(void);
void early_camera_flip(void);
int early_camera_on(void);

void early_camera_stop(void);

void set_early_camera_enabled(bool enabled);

#endif
