/* Copyright (c) 2015, The Linux Foundation. All rights reserved.
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

#ifndef __MSM8952_CLOCK_H
#define __MSM8952_CLOCK_H

#include <clock.h>
#include <clock_lib2.h>

#define UART_DM_CLK_RX_TX_BIT_RATE 0xCC

#define REG_MM(off)                     (CLK_CTL_BASE + (off))

#ifdef EARLY_CAMERA_SUPPORT
#define MMSS_CAMSS_CSI0_CBCR            REG_MM(0x30B4)
#define MMSS_CAMSS_CSI0_AHB_CBCR        REG_MM(0x30BC)
#define MMSS_CCI_CMD_RCGR               REG_MM(0x3300)
#define MMSS_CCI_CFG_RCGR               REG_MM(0x3304)
#define MMSS_CCI_M                      REG_MM(0x3308)
#define MMSS_CCI_N                      REG_MM(0x330c)
#define MMSS_CCI_D                      REG_MM(0x3310)
#define MMSS_CAMSS_CCI_CBCR             REG_MM(0x3344)
#define MMSS_CAMSS_CCI_AHB_CBCR         REG_MM(0x3348)
#define MMSS_CAMSS_AHB_CBCR             REG_MM(0x348C)
#define MMSS_CAMSS_TOP_AHB_CBCR         REG_MM(0x3484)
#define MMSS_VFE0_CMD_RCGR              REG_MM(0x3600)
#define MMSS_CAMSS_VFE0_AHB_CBCR        REG_MM(0x3668)
#define MMSS_CAMSS_CSI_VFE0_CBCR        REG_MM(0x3704)
#define MMSS_CAMSS_VFE0_STREAM_CBCR     REG_MM(0x3720)
#define MMSS_CAMSS_VFE_AXI_CBCR         REG_MM(0x36BC)
#define MMSS_MMAGIC_CAMSS_AXI_CBCR      REG_MM(0x3C44)
#define MMSS_VFE0_CFG_RCGR              REG_MM(0x3604)
#define MMSS_CAMSS_VFE0_CBCR            REG_MM(0x36A8)
#define MMSS_CAMSS_VFE0_BCR             REG_MM(0x3660)
#define MMSS_CAMSS_VFE_AHB_CBCR         REG_MM(0x36B8)
#define MMSS_CAMSS_VFE_AXI_CBCR         REG_MM(0x36BC)
#define MMSS_CAMSS_JPEG_AXI_CBCR        REG_MM(0x35B8)
#define MMSS_CAMSS_CPP_AXI_CBCR         REG_MM(0x36C4)
#define MMSS_CAMSS_ISPIF_AHB_CBCR       REG_MM(0x3224)
#define MMSS_CAMSS_CSI2_AHB_CBCR        REG_MM(0x3188)
#define MMSS_CAMSS_CSI2PHYTIMER_CBCR    REG_MM(0x3084)
#define MMSS_SMMU_VFE_AXI_CBCR          REG_MM(0x3C08)
#define MMSS_SMMU_VFE_AHB_CBCR          REG_MM(0x3C04)
#define MMSS_CSI0_CMD_RCGR              REG_MM(0x3090)
#define MMSS_CSI0_CFG_RCGR              REG_MM(0x3094)
#define MMSS_CAMSS_CSI0PHY_CBCR         REG_MM(0x30C4)
#define MMSS_CAMSS_CSI0PIX_CBCR         REG_MM(0x30E4)
#define MMSS_CAMSS_CSI0RDI_CBCR         REG_MM(0x30D4)
#define MMSS_CSI0PHYTIMER_CMD_RCGR      REG_MM(0x3000)
#define MMSS_CSI0PHYTIMER_CFG_RCGR      REG_MM(0x3004)
#define MMSS_CSI2PHYTIMER_CMD_RCGR      REG_MM(0x3060)
#define MMSS_CSI2PHYTIMER_CFG_RCGR      REG_MM(0x3064)
#define MMSS_CAMSS_CSI2PHY_CBCR         REG_MM(0x3194)
#define MMSS_CSI2_CMD_RCGR              REG_MM(0x3160)
#define MMSS_CSI2_CFG_RCGR              REG_MM(0x3164)
#define MMSS_CAMSS_CSI2_CBCR            REG_MM(0x3184)
#define MMSS_CSIPHY2_3P_CMD_RCGR        REG_MM(0x3280)
#define MMSS_CSIPHY2_3P_CFG_RCGR        REG_MM(0x3284)
#define MMSS_CAMSS_CSIPHY2_3P_CBCR      REG_MM(0x3274)
#define MMSS_CAMSS_CSI2RDI_CBCR         REG_MM(0x31A4)
#define MMSS_CAMSS_CSI2PIX_CBCR         REG_MM(0x31B4)
#define MMSS_CAMSS_CPP_VBIF_AHB_CBCR    REG_MM(0x36C8)
#define MMSS_MMSS_MMAGIC_CFG_AHB_CBCR   REG_MM(0x5054)
#define MMSS_MMAGIC_CAMSS_GDSCR         REG_MM(0x3C4C)
#define MMSS_CAMSS_VFE0_GDSCR           REG_MM(0x3664)
#define MMSS_CAMSS_TOP_GDSCR            REG_MM(0x34A0)
#define MMSS_CAMSS_VFE1_GDSCR           REG_MM(0x3674)
#define MMSS_CAMSS_JPEG_GDSCR           REG_MM(0x35A4)
#define MMSS_CAMSS_CPP_GDSCR            REG_MM(0x36D4)
#define MMSS_FD_GDSCR                   REG_MM(0x3B64)

#define VIDEO_GDSCR                     REG_MM(0x1024)
#define MMAGIC_VIDEO_GDSCR              REG_MM(0x119c)
#define MDSS_GDSCR                      REG_MM(0x2304)
#define MMAGIC_MDSS_GDSCR               REG_MM(0x247C)
#define MMAGIC_BIMC_GDSCR               REG_MM(0x529C)
#endif

#define MDP_GDSCR                       REG_MM(0x4D078)
#define GDSC_POWER_ON_BIT               BIT(31)
#define GDSC_POWER_ON_STATUS_BIT        BIT(29)
#define GDSC_EN_FEW_WAIT_MASK           (0x0F << 16)
#define GDSC_EN_FEW_WAIT_256_MASK       BIT(19)

#define VSYNC_CMD_RCGR                  REG_MM(0x4D02C)
#define VSYNC_CFG_RCGR                  REG_MM(0x4D030)
#define MDSS_VSYNC_CBCR                 REG_MM(0x4D090)

#define MDP_CMD_RCGR                    REG_MM(0x4D014)
#define MDP_CFG_RCGR                    REG_MM(0x4D018)
#define MDP_CBCR                        REG_MM(0x4D088)
#define MDP_AHB_CBCR                    REG_MM(0x4D07C)
#define MDP_AXI_CBCR                    REG_MM(0x4D080)

#ifdef EARLY_CAMERA_SUPPORT
#define MMSS_AHB_CMD_RCGR               REG_MM(0x5000)
#define MMSS_AHB_CFG_RCGR               REG_MM(0x5004)

#define MMSS_MMAGIC_AHB_CBCR            REG_MM(0x5024)
#define SMMU_MDP_AHB_CBCR               REG_MM(0x2454)
#define MDSS_AHB_CBCR                   REG_MM(0x2308)
#define MDSS_HDMI_AHB_CBCR              REG_MM(0x230C)
#define MDSS_HDMI_CBCR                  REG_MM(0x2338)
#define MDSS_EXTPCLK_CBCR               REG_MM(0x2324)
#define EXTPCLK_CMD_RCGR                REG_MM(0x2060)
#define EXTPCLK_CFG_RCGR                REG_MM(0x2064)
#define HDMI_CMD_RCGR                   REG_MM(0x2100)
#define HDMI_CFG_RCGR                   REG_MM(0x2104)

#define AXI_CMD_RCGR                    REG_MM(0x5040)
#define AXI_CFG_RCGR                    REG_MM(0x5044)
#define MMSS_S0_AXI_CBCR                REG_MM(0x5064)
#define MMSS_MMAGIC_AXI_CBCR            REG_MM(0x506C)
#define MMAGIC_MDSS_AXI_CBCR            REG_MM(0x2474)
#define MMAGIC_BIMC_AXI_CBCR            REG_MM(0x5294)
#define SMMU_MDP_AXI_CBCR               REG_MM(0x2458)
#define MDSS_AXI_CBCR                   REG_MM(0x2310)
#endif


#define DSI_BYTE0_CMD_RCGR              REG_MM(0x4D044)
#define DSI_BYTE0_CFG_RCGR              REG_MM(0x4D048)
#define DSI_BYTE0_CBCR                  REG_MM(0x4D094)
#define DSI_ESC0_CMD_RCGR               REG_MM(0x4D05C)
#define DSI_ESC0_CFG_RCGR               REG_MM(0x4D060)
#define DSI_ESC0_CBCR                   REG_MM(0x4D098)
#define DSI_PIXEL0_CMD_RCGR             REG_MM(0x4D000)
#define DSI_PIXEL0_CFG_RCGR             REG_MM(0x4D004)
#define DSI_PIXEL0_CBCR                 REG_MM(0x4D084)
#define DSI_PIXEL0_M                    REG_MM(0x4D008)
#define DSI_PIXEL0_N                    REG_MM(0x4D00C)
#define DSI_PIXEL0_D                    REG_MM(0x4D010)

#define DSI_BYTE1_CMD_RCGR              REG_MM(0x4D0B0)
#define DSI_BYTE1_CFG_RCGR              REG_MM(0x4D0B4)
#define DSI_BYTE1_CBCR                  REG_MM(0x4D0A0)
#define DSI_ESC1_CMD_RCGR               REG_MM(0x4D0A8)
#define DSI_ESC1_CFG_RCGR               REG_MM(0x4D0AC)
#define DSI_ESC1_CBCR                   REG_MM(0x4D09C)
#define DSI_PIXEL1_CMD_RCGR             REG_MM(0x4D0B8)
#define DSI_PIXEL1_CFG_RCGR             REG_MM(0x4D0BC)
#define DSI_PIXEL1_CBCR                 REG_MM(0x4D0A4)
#define DSI_PIXEL1_M                    REG_MM(0x4D0C0)
#define DSI_PIXEL1_N                    REG_MM(0x4D0C4)
#define DSI_PIXEL1_D                    REG_MM(0x4D0C8)

#ifdef EARLY_CAMERA_SUPPORT
#define DSI0_PHY_PLL_OUT                BIT(8)
#define DSI1_PHY_PLL_OUT                BIT(9)
#define PIXEL_SRC_DIV_1_5               BIT(1)
#endif

#define MMSS_DSI_CLKS_FLAG_DSI0         BIT(0)
#define MMSS_DSI_CLKS_FLAG_DSI1         BIT(1)

void platform_clock_init(void);

void clock_init_mmc(uint32_t interface);
void clock_config_mmc(uint32_t interface, uint32_t freq);
void clock_config_uart_dm(uint8_t id);
void hsusb_clock_init(void);
void clock_config_ce(uint8_t instance);
void clock_ce_enable(uint8_t instance);
void clock_ce_disable(uint8_t instance);
void mdp_gdsc_ctrl(uint8_t enable);

#ifdef EARLY_CAMERA_SUPPORT
void clock_usb30_init(void);
void clock_reset_usb_phy();
void mmss_dsi_clock_enable(uint32_t cfg_rcgr, uint32_t dual_dsi);
void mmss_dsi_clock_disable(uint32_t dual_dsi);
void mmss_bus_clock_enable(void);
void mmss_bus_clock_disable(void);
#endif

void mdss_bus_clocks_enable(void);
void mdss_bus_clocks_disable(void);
void mdp_clock_enable(void);
void mdp_clock_disable(void);

#ifdef EARLY_CAMERA_SUPPORT
void mmss_gdsc_enable();
void mmss_gdsc_disable();
void video_gdsc_enable();
void video_gdsc_disable();
void camera_clocks_enable(int enable);
void camera_gdsc_enable(int enable);
void clock_config_blsp_i2c(uint8_t blsp_id, uint8_t qup_id);

void hdmi_ahb_core_clk_enable(void);
void hdmi_pixel_clk_enable(uint32_t rate);
void hdmi_pixel_clk_disable(void);
void hdmi_core_ahb_clk_disable(void);
#endif

void gcc_dsi_clocks_enable(uint32_t flags,  bool use_dsi1_pll, uint8_t pclk0_m,
		uint8_t pclk0_n, uint8_t pclk0_d);
void gcc_dsi_clocks_disable(uint32_t flags);
#endif
