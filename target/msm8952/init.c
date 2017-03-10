/* Copyright (c) 2015-2016, The Linux Foundation. All rights reserved.
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
#include <platform/iomap.h>
#include <reg.h>
#include <target.h>
#include <platform.h>
#include <uart_dm.h>
#include <mmc.h>
#include <platform/gpio.h>
#include <dev/keys.h>
#include <spmi_v2.h>
#include <pm8x41.h>
#include <pm8x41_hw.h>
#include <board.h>
#include <baseband.h>
#include <hsusb.h>
#include <scm.h>
#include <platform/gpio.h>
#include <platform/gpio.h>
#include <platform/irqs.h>
#include <platform/clock.h>
#include <platform/timer.h>
#include <crypto5_wrapper.h>
#include <partition_parser.h>
#include <stdlib.h>
#include <rpm-smd.h>
#include <spmi.h>
#include <sdhci_msm.h>
#include <clock.h>
#include <boot_device.h>
#include <secapp_loader.h>
#include <rpmb.h>
#include <smem.h>

#include "target/display.h"

#ifdef EARLY_CAMERA_SUPPORT
#include <msm_panel.h>
#include <psci.h>
#include <target/display.h>
#include <target/target_camera.h>
#define ANIMATED_SPLAH_PARTITION "splash"
#define ANIMATED_SPLASH_BUFFER   0x836a5580
#endif

#if LONG_PRESS_POWER_ON
#include <shutdown_detect.h>
#endif

#if PON_VIB_SUPPORT
#include <vibrator.h>
#endif

#if PON_VIB_SUPPORT
#define VIBRATE_TIME    250
#endif

#define PMIC_ARB_CHANNEL_NUM    0
#define PMIC_ARB_OWNER_ID       0
#define TLMM_VOL_UP_BTN_GPIO    85
#define TLMM_VOL_UP_BTN_GPIO_8956 113
#define TLMM_VOL_UP_BTN_GPIO_8937 91
#define TLMM_VOL_DOWN_BTN_GPIO    128

#define FASTBOOT_MODE           0x77665500
#define RECOVERY_MODE           0x77665502
#define PON_SOFT_RB_SPARE       0x88F

#define CE1_INSTANCE            1
#define CE_EE                   1
#define CE_FIFO_SIZE            64
#define CE_READ_PIPE            3
#define CE_WRITE_PIPE           2
#define CE_READ_PIPE_LOCK_GRP   0
#define CE_WRITE_PIPE_LOCK_GRP  0
#define CE_ARRAY_SIZE           20
#define SUB_TYPE_SKUT           0x0A

struct mmc_device *dev;

static uint32_t mmc_pwrctl_base[] =
	{ MSM_SDC1_BASE, MSM_SDC2_BASE };

static uint32_t mmc_sdhci_base[] =
	{ MSM_SDC1_SDHCI_BASE, MSM_SDC2_SDHCI_BASE };

static uint32_t  mmc_sdc_pwrctl_irq[] =
	{ SDCC1_PWRCTL_IRQ, SDCC2_PWRCTL_IRQ };

#ifdef EARLY_CAMERA_SUPPORT
static int early_camera_enabled = 1;
void (*cpu_on_ep) (void);
/* early domain initialization */
void earlydomain_init();

/* run all early domain services */
void earlydomain_services();

/* early domain cleanup and exit*/
void earlydomain_exit();

/* is charging supported */
bool target_charging_supported();

/* is charging in progress */
bool target_charging_in_progress();
#endif

void target_early_init(void)
{
#if WITH_DEBUG_UART
	uart_dm_init(2, 0, BLSP1_UART1_BASE);
#endif
}

#ifdef EARLY_CAMERA_SUPPORT
bool mmc_read_done = false;
bool target_is_mmc_read_done()
{
	return mmc_read_done;
}
#endif

static void set_sdc_power_ctrl()
{
	/* Drive strength configs for sdc pins */
	struct tlmm_cfgs sdc1_hdrv_cfg[] =
	{
		{ SDC1_CLK_HDRV_CTL_OFF,  TLMM_CUR_VAL_16MA, TLMM_HDRV_MASK, 0},
		{ SDC1_CMD_HDRV_CTL_OFF,  TLMM_CUR_VAL_10MA, TLMM_HDRV_MASK, 0},
		{ SDC1_DATA_HDRV_CTL_OFF, TLMM_CUR_VAL_10MA, TLMM_HDRV_MASK , 0},
	};

	/* Pull configs for sdc pins */
	struct tlmm_cfgs sdc1_pull_cfg[] =
	{
		{ SDC1_CLK_PULL_CTL_OFF,  TLMM_NO_PULL, TLMM_PULL_MASK, 0},
		{ SDC1_CMD_PULL_CTL_OFF,  TLMM_PULL_UP, TLMM_PULL_MASK, 0},
		{ SDC1_DATA_PULL_CTL_OFF, TLMM_PULL_UP, TLMM_PULL_MASK, 0},
	};

	struct tlmm_cfgs sdc1_rclk_cfg[] =
	{
		{ SDC1_RCLK_PULL_CTL_OFF, TLMM_PULL_DOWN, TLMM_PULL_MASK, 0},
	};

	/* Set the drive strength & pull control values */
	tlmm_set_hdrive_ctrl(sdc1_hdrv_cfg, ARRAY_SIZE(sdc1_hdrv_cfg));
	tlmm_set_pull_ctrl(sdc1_pull_cfg, ARRAY_SIZE(sdc1_pull_cfg));
	tlmm_set_pull_ctrl(sdc1_rclk_cfg, ARRAY_SIZE(sdc1_rclk_cfg));
}

void target_sdc_init()
{
	struct mmc_config_data config;

	/* Set drive strength & pull ctrl values */
	set_sdc_power_ctrl();

	/* Try slot 1*/
	config.slot          = 1;
	config.bus_width     = DATA_BUS_WIDTH_8BIT;
	config.max_clk_rate  = MMC_CLK_192MHZ;
	config.sdhc_base     = mmc_sdhci_base[config.slot - 1];
	config.pwrctl_base   = mmc_pwrctl_base[config.slot - 1];
	config.pwr_irq       = mmc_sdc_pwrctl_irq[config.slot - 1];
	config.hs400_support = 1;

	if (!(dev = mmc_init(&config))) {
	/* Try slot 2 */
		config.slot          = 2;
		config.max_clk_rate  = MMC_CLK_200MHZ;
		config.sdhc_base     = mmc_sdhci_base[config.slot - 1];
		config.pwrctl_base   = mmc_pwrctl_base[config.slot - 1];
		config.pwr_irq       = mmc_sdc_pwrctl_irq[config.slot - 1];
		config.hs400_support = 0;

		if (!(dev = mmc_init(&config))) {
			dprintf(CRITICAL, "mmc init failed!");
			ASSERT(0);
		}
	}
}

void *target_mmc_device()
{
	return (void *) dev;
}

/* Return 1 if vol_up pressed */
int target_volume_up()
{
	static uint8_t first_time = 0;
	uint8_t status = 0;
	uint32_t vol_up_gpio;

	if(platform_is_msm8956())
		vol_up_gpio = TLMM_VOL_UP_BTN_GPIO_8956;
	else if(platform_is_msm8937() || platform_is_msm8917())
		vol_up_gpio = TLMM_VOL_UP_BTN_GPIO_8937;
	else
		vol_up_gpio = TLMM_VOL_UP_BTN_GPIO;

	if (!first_time) {
		gpio_tlmm_config(vol_up_gpio, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA, GPIO_ENABLE);

		/* Wait for the gpio config to take effect - debounce time */
		udelay(10000);

		first_time = 1;
	}

	/* Get status of GPIO */
	status = gpio_status(vol_up_gpio);

	/* Active low signal. */
	return !status;
}

/* Return 1 if vol_down pressed */
uint32_t target_volume_down()
{
	static  bool vol_down_key_init = false;

	if ((board_hardware_id() == HW_PLATFORM_QRD) &&
			(board_hardware_subtype() == SUB_TYPE_SKUT)) {
		uint32_t status = 0;

		if (!vol_down_key_init) {
			gpio_tlmm_config(TLMM_VOL_DOWN_BTN_GPIO, 0, GPIO_INPUT, GPIO_PULL_UP,
				 GPIO_2MA, GPIO_ENABLE);
			/* Wait for the gpio config to take effect - debounce time */
			thread_sleep(10);
			vol_down_key_init = true;
		}

		/* Get status of GPIO */
		status = gpio_status(TLMM_VOL_DOWN_BTN_GPIO);

		/* Active low signal. */
		return !status;
	} else {
		/* Volume down button tied in with PMIC RESIN. */
		return pm8x41_resin_status();
	}
}

uint32_t target_is_pwrkey_pon_reason()
{
	uint8_t pon_reason = pm8950_get_pon_reason();
	if (pm8x41_get_is_cold_boot() && ((pon_reason == KPDPWR_N) || (pon_reason == (KPDPWR_N|PON1))))
		return 1;
	else
		return 0;
}

static void target_keystatus()
{
	keys_init();

	if(target_volume_down())
		keys_post_event(KEY_VOLUMEDOWN, 1);

	if(target_volume_up())
		keys_post_event(KEY_VOLUMEUP, 1);
}

/* Configure PMIC and Drop PS_HOLD for shutdown */
void shutdown_device()
{
	dprintf(CRITICAL, "Going down for shutdown.\n");

	/* Configure PMIC for shutdown */
	pm8x41_reset_configure(PON_PSHOLD_SHUTDOWN);

	/* Drop PS_HOLD for MSM */
	writel(0x00, MPM2_MPM_PS_HOLD);

	mdelay(5000);

	dprintf(CRITICAL, "shutdown failed\n");

	ASSERT(0);
}


void target_init(void)
{
#if VERIFIED_BOOT
#if !VBOOT_MOTA
	int ret = 0;
#endif
#endif
	dprintf(INFO, "target_init()\n");

	spmi_init(PMIC_ARB_CHANNEL_NUM, PMIC_ARB_OWNER_ID);

	if(platform_is_msm8937() || platform_is_msm8917())
	{
		uint8_t pmi_rev = 0;
		uint32_t pmi_type = 0;

		pmi_type = board_pmic_target(1) & 0xffff;
		if(pmi_type == PMIC_IS_PMI8950)
		{
			/* read pmic spare register for rev */
			pmi_rev = pmi8950_get_pmi_subtype();
			if(pmi_rev)
				board_pmi_target_set(1,pmi_rev);
		}
	}

	target_keystatus();

	target_sdc_init();
	if (partition_read_table())
	{
		dprintf(CRITICAL, "Error reading the partition table info\n");
		ASSERT(0);
	}

#if LONG_PRESS_POWER_ON
	shutdown_detect();
#endif

#if PON_VIB_SUPPORT
	/* turn on vibrator to indicate that phone is booting up to end user */
	vib_timed_turn_on(VIBRATE_TIME);
#endif

	if (target_use_signed_kernel())
		target_crypto_init_params();

#if VERIFIED_BOOT
#if !VBOOT_MOTA
	clock_ce_enable(CE1_INSTANCE);

	/* Initialize Qseecom */
	ret = qseecom_init();

	if (ret < 0)
	{
		dprintf(CRITICAL, "Failed to initialize qseecom, error: %d\n", ret);
		ASSERT(0);
	}

	/* Start Qseecom */
	ret = qseecom_tz_init();

	if (ret < 0)
	{
		dprintf(CRITICAL, "Failed to start qseecom, error: %d\n", ret);
		ASSERT(0);
	}

	if (rpmb_init() < 0)
	{
		dprintf(CRITICAL, "RPMB init failed\n");
		ASSERT(0);
	}

	/*
	 * Load the sec app for first time
	 */
	if (load_sec_app() < 0)
	{
		dprintf(CRITICAL, "Failed to load App for verified\n");
		ASSERT(0);
	}
#endif
#endif

#if SMD_SUPPORT
	rpm_smd_init();
#endif
}

void target_serialno(unsigned char *buf)
{
	uint32_t serialno;
	if (target_is_emmc_boot()) {
		serialno = mmc_get_psn();
		snprintf((char *)buf, 13, "%x", serialno);
	}
}

unsigned board_machtype(void)
{
	return LINUX_MACHTYPE_UNKNOWN;
}

/* Detect the target type */
void target_detect(struct board_data *board)
{
	/* This is already filled as part of board.c */
}

/* Detect the modem type */
void target_baseband_detect(struct board_data *board)
{
	uint32_t platform;

	platform = board->platform;

	switch(platform) {
	case MSM8952:
	case MSM8956:
	case MSM8976:
	case MSM8937:
	case MSM8940:
	case MSM8917:
	case MSM8920:
	case MSM8217:
	case MSM8617:
		board->baseband = BASEBAND_MSM;
		break;
	case APQ8052:
	case APQ8056:
	case APQ8076:
	case APQ8037:
	case APQ8017:
		board->baseband = BASEBAND_APQ;
		break;
	default:
		dprintf(CRITICAL, "Platform type: %u is not supported\n",platform);
		ASSERT(0);
	};
}

unsigned target_baseband()
{
	return board_baseband();
}

unsigned check_reboot_mode(void)
{
	uint32_t restart_reason = 0;

	/* Read reboot reason and scrub it */
	restart_reason = readl(RESTART_REASON_ADDR);
	writel(0x00, RESTART_REASON_ADDR);

	return restart_reason;
}

unsigned check_hard_reboot_mode(void)
{
	uint8_t hard_restart_reason = 0;
	uint8_t value = 0;

	/* Read reboot reason and scrub it
	  * Bit-5, bit-6 and bit-7 of SOFT_RB_SPARE for hard reset reason
	  */
	value = pm8x41_reg_read(PON_SOFT_RB_SPARE);
	hard_restart_reason = value >> 5;
	pm8x41_reg_write(PON_SOFT_RB_SPARE, value & 0x1f);

	return hard_restart_reason;
}

int set_download_mode(enum reboot_reason mode)
{
	int ret = 0;
	ret = scm_dload_mode(mode);

	pm8x41_clear_pmic_watchdog();

	return ret;
}

int emmc_recovery_init(void)
{
	return _emmc_recovery_init();
}

void reboot_device(unsigned reboot_reason)
{
	uint8_t reset_type = 0;
	uint32_t ret = 0;

	/* Set cookie for dload mode */
	if(set_download_mode(reboot_reason)) {
		dprintf(CRITICAL, "HALT: set_download_mode not supported\n");
		return;
	}

	writel(reboot_reason, RESTART_REASON_ADDR);

	/* For Reboot-bootloader and Dload cases do a warm reset
	 * For Reboot cases do a hard reset
	 */
	if((reboot_reason == FASTBOOT_MODE) || (reboot_reason == NORMAL_DLOAD) ||
		(reboot_reason == EMERGENCY_DLOAD) || (reboot_reason == RECOVERY_MODE))
		reset_type = PON_PSHOLD_WARM_RESET;
	else
		reset_type = PON_PSHOLD_HARD_RESET;

	pm8994_reset_configure(reset_type);

	ret = scm_halt_pmic_arbiter();
	if (ret)
		dprintf(CRITICAL , "Failed to halt pmic arbiter: %d\n", ret);

	/* Drop PS_HOLD for MSM */
	writel(0x00, MPM2_MPM_PS_HOLD);

	mdelay(5000);

	dprintf(CRITICAL, "Rebooting failed\n");
}

#if USER_FORCE_RESET_SUPPORT
/* Return 1 if it is a force resin triggered by user. */
uint32_t is_user_force_reset(void)
{
	uint8_t poff_reason1 = pm8x41_get_pon_poff_reason1();
	uint8_t poff_reason2 = pm8x41_get_pon_poff_reason2();

	dprintf(SPEW, "poff_reason1: %d\n", poff_reason1);
	dprintf(SPEW, "poff_reason2: %d\n", poff_reason2);
	if (pm8x41_get_is_cold_boot() && (poff_reason1 == KPDPWR_AND_RESIN ||
							poff_reason2 == STAGE3))
		return 1;
	else
		return 0;
}
#endif

#define SMBCHG_USB_RT_STS 0x21310
#define USBIN_UV_RT_STS BIT(0)
unsigned target_pause_for_battery_charge(void)
{
	uint8_t pon_reason = pm8x41_get_pon_reason();
	uint8_t is_cold_boot = pm8x41_get_is_cold_boot();
	bool usb_present_sts = !(USBIN_UV_RT_STS &
				pm8x41_reg_read(SMBCHG_USB_RT_STS));
	dprintf(INFO, "%s : pon_reason is:0x%x cold_boot:%d usb_sts:%d\n", __func__,
		pon_reason, is_cold_boot, usb_present_sts);
	/* In case of fastboot reboot,adb reboot or if we see the power key
	* pressed we do not want go into charger mode.
	* fastboot reboot is warm boot with PON hard reset bit not set
	* adb reboot is a cold boot with PON hard reset bit set
	*/
	if (is_cold_boot &&
			(!(pon_reason & HARD_RST)) &&
			(!(pon_reason & KPDPWR_N)) &&
			usb_present_sts)
		return 1;
	else
		return 0;
}

void target_uninit(void)
{
#if PON_VIB_SUPPORT
	turn_off_vib_early();
#endif
	mmc_put_card_to_sleep(dev);
	sdhci_mode_disable(&dev->host);
	if (crypto_initialized())
	{
		crypto_eng_cleanup();
		clock_ce_disable(CE1_INSTANCE);
	}

	if (target_is_ssd_enabled())
		clock_ce_disable(CE1_INSTANCE);

#if VERIFIED_BOOT
#if !VBOOT_MOTA
	if (is_sec_app_loaded())
	{
		if (send_milestone_call_to_tz() < 0)
		{
			dprintf(CRITICAL, "Failed to unload App for rpmb\n");
			ASSERT(0);
		}
	}

	if (rpmb_uninit() < 0)
	{
		dprintf(CRITICAL, "RPMB uninit failed\n");
		ASSERT(0);
	}

	clock_ce_disable(CE1_INSTANCE);
#endif
#endif

#if SMD_SUPPORT
	rpm_smd_uninit();
#endif
}

void target_usb_init(void)
{
	uint32_t val;

	/* Select and enable external configuration with USB PHY */
	ulpi_write(ULPI_MISC_A_VBUSVLDEXTSEL | ULPI_MISC_A_VBUSVLDEXT, ULPI_MISC_A_SET);

	/* Enable sess_vld */
	val = readl(USB_GENCONFIG_2) | GEN2_SESS_VLD_CTRL_EN;
	writel(val, USB_GENCONFIG_2);

	/* Enable external vbus configuration in the LINK */
	val = readl(USB_USBCMD);
	val |= SESS_VLD_CTRL;
	writel(val, USB_USBCMD);
}

void target_usb_stop(void)
{
	/* Disable VBUS mimicing in the controller. */
	ulpi_write(ULPI_MISC_A_VBUSVLDEXTSEL | ULPI_MISC_A_VBUSVLDEXT, ULPI_MISC_A_CLEAR);
}

static uint8_t splash_override;
/* Returns 1 if target supports continuous splash screen. */
int target_cont_splash_screen()
{
	uint8_t splash_screen = 0;
	if (!splash_override) {
		switch (board_hardware_id()) {
		case HW_PLATFORM_MTP:
		case HW_PLATFORM_SURF:
		case HW_PLATFORM_RCM:
		case HW_PLATFORM_QRD:
			splash_screen = 1;
			break;
		default:
			splash_screen = 0;
			break;
		}
		dprintf(SPEW, "Target_cont_splash=%d\n", splash_screen);
	}
	return splash_screen;
}

#ifdef EARLY_CAMERA_SUPPORT
/* Returns 1 if target supports animated splash screen. */
int target_animated_splash_screen()
{
	uint8_t animated_splash = 0;
/*	if(!splash_override && !target_charging_in_progress()) {*/
	if(!splash_override) {
		switch(board_hardware_id()) {
			case HW_PLATFORM_ADP:
			case HW_PLATFORM_DRAGON:
				dprintf(SPEW, "Target_animated_splash=1\n");
				// enable animated splash for ADP and Dragonboard
				animated_splash = 1;
				break;
			default:
				dprintf(SPEW, "Target_animated_splash=0\n");
				animated_splash = 0;
		}
	}
	return animated_splash;
}
#endif

void target_force_cont_splash_disable(uint8_t override)
{
        splash_override = override;
}

uint8_t target_panel_auto_detect_enabled()
{
	uint8_t ret = 0;

	switch(board_hardware_id())
	{
		case HW_PLATFORM_QRD:
			ret = platform_is_msm8956() ? 1 : 0;
			break;
		case HW_PLATFORM_SURF:
		case HW_PLATFORM_MTP:
		default:
			ret = 0;
	}
	return ret;
}

/* Do any target specific intialization needed before entering fastboot mode */
void target_fastboot_init(void)
{
	if (target_is_ssd_enabled()) {
		clock_ce_enable(CE1_INSTANCE);
		target_load_ssd_keystore();
	}
}

void target_load_ssd_keystore(void)
{
	uint64_t ptn;
	int      index;
	uint64_t size;
	uint32_t *buffer = NULL;

	if (!target_is_ssd_enabled())
		return;

	index = partition_get_index("ssd");

	ptn = partition_get_offset(index);
	if (ptn == 0){
		dprintf(CRITICAL, "Error: ssd partition not found\n");
		return;
	}

	size = partition_get_size(index);
	if (size == 0) {
		dprintf(CRITICAL, "Error: invalid ssd partition size\n");
		return;
	}

	buffer = memalign(CACHE_LINE, ROUNDUP(size, CACHE_LINE));
	if (!buffer) {
		dprintf(CRITICAL, "Error: allocating memory for ssd buffer\n");
		return;
	}

	if (mmc_read(ptn, buffer, size)) {
		dprintf(CRITICAL, "Error: cannot read data\n");
		free(buffer);
		return;
	}

	clock_ce_enable(CE1_INSTANCE);
	scm_protect_keystore(buffer, size);
	clock_ce_disable(CE1_INSTANCE);
	free(buffer);
}

crypto_engine_type board_ce_type(void)
{
	return CRYPTO_ENGINE_TYPE_HW;
}

/* Set up params for h/w CE. */
void target_crypto_init_params()
{
	struct crypto_init_params ce_params;

	/* Set up base addresses and instance. */
	ce_params.crypto_instance  = CE1_INSTANCE;
	ce_params.crypto_base      = MSM_CE1_BASE;
	ce_params.bam_base         = MSM_CE1_BAM_BASE;

	/* Set up BAM config. */
	ce_params.bam_ee               = CE_EE;
	ce_params.pipes.read_pipe      = CE_READ_PIPE;
	ce_params.pipes.write_pipe     = CE_WRITE_PIPE;
	ce_params.pipes.read_pipe_grp  = CE_READ_PIPE_LOCK_GRP;
	ce_params.pipes.write_pipe_grp = CE_WRITE_PIPE_LOCK_GRP;

	/* Assign buffer sizes. */
	ce_params.num_ce           = CE_ARRAY_SIZE;
	ce_params.read_fifo_size   = CE_FIFO_SIZE;
	ce_params.write_fifo_size  = CE_FIFO_SIZE;

	/* BAM is initialized by TZ for this platform.
	 * Do not do it again as the initialization address space
	 * is locked.
	 */
	ce_params.do_bam_init      = 0;

	crypto_init_params(&ce_params);
}

uint32_t target_get_pmic()
{
	return PMIC_IS_PMI8950;
}
#if EARLYDOMAIN_SUPPORT

/* calls psci to turn on the secondary core */
void enable_secondary_core()
{
    cpu_on_ep = &cpu_on_asm;

    if (psci_cpu_on(platform_get_secondary_cpu_num(), (paddr_t)cpu_on_ep))
    {
        dprintf(CRITICAL, "Failed to turn on secondary CPU: %x\n", platform_get_secondary_cpu_num());
    }
    dprintf (INFO, "LK continue on cpu0\n");
}

/* handles the early domain */
void earlydomain()
{
    /* init and run early domain services*/
    earlydomain_init();

    /* run all early domain services */
    earlydomain_services();

    /* cleanup and exit early domain services*/
    earlydomain_exit();
}

/* early domain initialization */
void earlydomain_init()
{
    dprintf(INFO, "started early domain on secondary CPU\n");
}

/* early domain cleanup and exit */
void earlydomain_exit()
{
    isb();

    /* clean-up */
    arch_disable_cache(UCACHE);

    arch_disable_mmu();

    dsb();
    isb();

    arch_disable_ints();

    /* turn off secondary cpu */
    psci_cpu_off();
}

enum compression_type {
	COMPRESSION_TYPE_NONE = 0,
	COMPRESSION_TYPE_RLE24,
	COMPRESSION_TYPE_MAX
};

typedef struct animated_img_header {
	unsigned char magic[LOGO_IMG_MAGIC_SIZE]; /* "SPLASH!!" */
	uint32_t display_id;
	uint32_t width;
	uint32_t height;
	uint32_t fps;
	uint32_t num_frames;
	uint32_t type;
	uint32_t blocks;
	uint32_t img_size;
	uint32_t offset;
	uint8_t reserved[512-44];
} animated_img_header;

#define NUM_DISPLAYS 3
void **buffers[NUM_DISPLAYS];
struct animated_img_header g_head[NUM_DISPLAYS];

int animated_splash_screen_mmc()
{
	int index = INVALID_PTN;
	unsigned long long ptn = 0;
	struct fbcon_config *fb_display = NULL;
	struct animated_img_header *header;
	uint32_t blocksize, realsize, readsize;
	void *buffer;
	uint32_t i = 0, j = 0;
	void *tmp;
	void *head;
	int ret = 0;

	index = partition_get_index(ANIMATED_SPLAH_PARTITION);
	if (index == 0) {
		dprintf(CRITICAL, "ERROR: splash Partition table not found\n");
		return -1;
	}
	ptn = partition_get_offset(index);
	if (ptn == 0) {
		dprintf(CRITICAL, "ERROR: splash Partition invalid\n");
		return -1;
	}

	mmc_set_lun(partition_get_lun(index));

	blocksize = mmc_get_device_blocksize();
	if (blocksize == 0) {
		dprintf(CRITICAL, "ERROR:splash Partition invalid blocksize\n");
		return -1;
	}
	// Assume it is always for display ID 0
	fb_display = target_display_get_fb(0);

	if (!fb_display) {
		dprintf(CRITICAL, "ERROR: fb config is not allocated\n");
		return -1;
	}

	buffer = (void *)ANIMATED_SPLASH_BUFFER;
	for (j = 0; j < NUM_DISPLAYS; j++)
	{
		head = (void *)&(g_head[j]);
		if (mmc_read(ptn, (uint32_t *)(head), blocksize)) {
			dprintf(CRITICAL, "ERROR: Cannot read splash image header\n");
			return -1;
		}

		header = (animated_img_header *)head;
		if (memcmp(header->magic, LOGO_IMG_MAGIC, 8)) {
			dprintf(CRITICAL, "Invalid magic number in header %s %d\n",
				header->magic, header->height);
			ret = -1;
			goto end;
		}

		if (header->width == 0 || header->height == 0) {
			dprintf(CRITICAL, "Invalid height and width\n");
			ret = -1;
			goto end;
		}

		buffers[j] = (void **)malloc(header->num_frames*sizeof(void *));
		if (buffers[j] == NULL) {
			dprintf(CRITICAL, "Cant alloc mem for ptrs\n");
			ret = -1;
			goto end;
		}
		if ((header->type == COMPRESSION_TYPE_RLE24) && (header->blocks != 0)) {
			dprintf(CRITICAL, "Compressed data not supported\n");
			ret = 0;
			goto end;
		} else {
			if ((header->width > fb_display->width) ||
				(header->height > fb_display->height)) {
				dprintf(CRITICAL, "Logo config greater than fb config. header->width %u"
					" fb->width = %u header->height = %u fb->height = %u\n",
					header->width, fb_display->width, header->height, fb_display->height);
				ret = -1;
				goto end;
			}
			dprintf(INFO, "width:%d height:%d blocks:%d imgsize:%d num_frames:%d\n", header->width,
			header->height, header->blocks, header->img_size,header->num_frames);
			realsize =  header->blocks * blocksize;
			if ((realsize % blocksize) != 0)
				readsize =  ROUNDUP(realsize, blocksize) - blocksize;
			else
				readsize = realsize;
			if (blocksize == LOGO_IMG_HEADER_SIZE) {
				if (mmc_read((ptn + LOGO_IMG_HEADER_SIZE), (uint32_t *)buffer, readsize)) {
					dprintf(CRITICAL, "ERROR: Cannot read splash image from partition 1\n");
					ret = -1;
					goto end;
				}
			} else {
				if (mmc_read(ptn + blocksize , (uint32_t *)buffer, realsize)) {
					dprintf(CRITICAL, "ERROR: Cannot read splash image from partition 2\n");
					ret = -1;
					goto end;
				}
			}
			tmp = buffer;
			for (i = 0; i < header->num_frames; i++) {
				buffers[j][i] = tmp;
				tmp = tmp + header->img_size;
			}

		}
		buffer = tmp;
		ptn += LOGO_IMG_HEADER_SIZE + readsize;
	}
end:
	return ret;
}

#if EARLYCAMERA_NO_GPIO

inline bool get_reverse_camera_gpio() {
	return TRUE;
}

#else

inline bool get_reverse_camera_gpio() {
	/* if gpio == 1, it is ON
	   if gpio == 0, it is OFF */
	return (1 == gpio_get(103));
}

#endif

/* checks if GPIO or equivalent trigger to enable early camera is set to ON
   If this function retuns FALSE, only animated splash may be shown.
   This is also a check to see if early-camera/animated splash can exit*/
bool is_reverse_camera_on() {
	uint32_t trigger_reg = 0;
	trigger_reg = readl_relaxed((void *)MDSS_SCRATCH_REG_2);
	if ((FALSE == get_reverse_camera_gpio()) ||
		(0xF5F5F5F5 == trigger_reg))
		return FALSE; /* trigger to exit */
	else
		return TRUE;
}

int animated_splash() {
	void *disp_ptr, *layer_ptr;
	uint32_t ret = 0, k = 0, j = 0;
	uint32_t frame_cnt[NUM_DISPLAYS];
	struct target_display_update update[NUM_DISPLAYS];
	struct target_layer layer[NUM_DISPLAYS];
	struct target_display * disp;
	struct fbcon_config *fb;
	uint32_t sleep_time;
	uint32_t disp_cnt = NUM_DISPLAYS;
	uint32_t reg_value;
	bool camera_on = FALSE;
	bool camera_frame_on = false;

	if (!buffers[0]) {
		dprintf(CRITICAL, "Unexpected error in read\n");
		return 0;
	}
	for (j = 0; j < NUM_DISPLAYS; j ++) {
		frame_cnt[j] = 0;
		disp_ptr = target_display_open(j);
		if (disp_ptr == NULL) {
			dprintf(CRITICAL, "Display open failed\n");
			return -1;
		}
		disp = target_get_display_info(disp_ptr);
		if (disp == NULL){
			dprintf(CRITICAL, "Display info failed\n");
			return -1;
		}
		layer_ptr = target_display_acquire_layer(disp_ptr, "as", kFormatRGB888);
		if (layer_ptr == NULL){
			dprintf(CRITICAL, "Layer acquire failed\n");
			return -1;
		}
		fb = target_display_get_fb(j);

		layer[j].layer = layer_ptr;
		layer[j].z_order = 2;
		update[j].disp = disp_ptr;
		update[j].layer_list = &layer[j];
		update[j].num_layers = 1;
		layer[j].fb = fb;
		sleep_time = 1000 / g_head[j].fps;
		layer[j].width = fb->width;
		layer[j].height = fb->height;
		if (g_head[j].width < fb->width) {
			dprintf(SPEW, "Modify width\n");
			layer[j].width = g_head[j].width;
		}
		if (g_head[j].height < fb->height) {
			dprintf(SPEW, "Modify height\n");
			layer[j].height = g_head[j].height;
		}
	}

	while (1) {
		camera_on = is_reverse_camera_on();

		reg_value = readl_relaxed((void *)MDSS_SCRATCH_REG_1);
		if (0xFEFEFEFE == reg_value) {
			//This value indicates kernel request LK to shutdown immediately
			break;
		}
		else if (0xDEADDEAD == reg_value) {
			// This reg value means kernel is started
			// LK should notify kernel by writing 0xDEADBEEF to
			// MDSS_SCRATCH_REG_1 when it is ready to exit

			if (0 == early_camera_enabled)
				break;
			else if ((1 == early_camera_enabled) &&
					(FALSE == camera_on) && (false == camera_frame_on))
				break;
		}

		for (j = 0; j < disp_cnt; j++) {
			if (j == 0 && early_camera_enabled == 1) {
				if (early_camera_on()) {
					if(layer[j].layer) {
						target_release_layer(&layer[j]);
						layer[j].layer = NULL;
					}
					camera_frame_on = true;
					continue;
				} else {
					if(!layer[j].layer) {
						layer_ptr = target_display_acquire_layer(disp_ptr, "as", kFormatRGB888);
						layer[j].layer = layer_ptr;
						layer[j].z_order = 2;
						camera_frame_on = false;
					}
				}
			}
			layer[j].fb->base = buffers[j][frame_cnt[j]];
			layer[j].fb->format = kFormatRGB888;
			layer[j].fb->bpp = 24;
			ret = target_display_update(&update[j],1, j);
			frame_cnt[j]++;
			if (frame_cnt[j] >= g_head[j].num_frames) {
				frame_cnt[j] = 0;
			}
		}

		if(early_camera_enabled == 1)
			// Rely on camera timing to flip.
			early_camera_flip();
		else
			// assume all displays have the same fps
			mdelay_optimal(sleep_time);
		k++;
	}
	if (early_camera_enabled == 1)
		early_camera_stop();
	for (j = 0; j < NUM_DISPLAYS; j++) {
		target_release_layer(&layer[j]);
	}

	return ret;
}

/* early domain services */
void earlydomain_services()
{
	uint32_t ret = 0;
	int i = 0;

	dprintf(CRITICAL, "earlydomain_services: Waiting for display init to complete\n");

	while((FALSE == target_display_is_init_done()) && (i < 100))
	{
		mdelay_optimal(10); // delay for 10ms
		i++;
	}

	dprintf(CRITICAL, "earlydomain_services: Display init done\n");
	// Notify Kernel that LK is running
	writel(0xC001CAFE, MDSS_SCRATCH_REG_1);

	/* starting early domain services */
	if (early_camera_init() == -1) {
		early_camera_enabled = 0;
		dprintf(CRITICAL, "earlydomain_services: Early Camera exit init failed\n");
	} else {
		dprintf(CRITICAL, "earlydomain_services: Early Camera starting\n");
	}

	/*Create Animated splash thread
	if target supports it*/
	if (target_animated_splash_screen())
	{
		ret = animated_splash_screen_mmc();
		mmc_read_done = true;
		dprintf(CRITICAL, "earlydomain_services: mmc read done\n");
		if (ret) {
			dprintf(CRITICAL, "Error in reading memory. Skip Animated splash\n");
		} else {
			animated_splash();
		}
	}

	// Notify Kernel that LK is shutdown
	writel(0xDEADBEEF, MDSS_SCRATCH_REG_1);

  /* starting early domain services */
}

#else

/* stubs for early domain functions */
void enable_secondary_core() {}
void earlydomain() {}
void earlydomain_init() {}
void earlydomain_services() {}
void earlydomain_exit() {}

#endif /* EARLYDOMAIN_SUPPORT */
