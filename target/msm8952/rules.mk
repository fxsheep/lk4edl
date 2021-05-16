LOCAL_DIR := $(GET_LOCAL_DIR)

INCLUDES += -I$(LOCAL_DIR)/include -I$(LK_TOP_DIR)/platform/msm_shared
INCLUDES += -I$(LK_TOP_DIR)/dev/gcdb/display -I$(LK_TOP_DIR)/dev/gcdb/display/include
INCLUDES += -I$(LK_TOP_DIR)/app/aboot

PLATFORM := msm8952

MEMBASE := 0x86500000 # SDRAM
MEMSIZE := 0x00A00000 # 10MB

BASE_ADDR        := 0x80000000
SCRATCH_ADDR     := 0xA0A00000

DEFINES += DISPLAY_SPLASH_SCREEN=1
DEFINES += DISPLAY_TYPE_MIPI=1
DEFINES += DISPLAY_TYPE_DSI6G=1

DEFINES += PMI_CONFIGURED=1

MODULES += \
	dev/keys \
	lib/ptable \
	dev/pmic/pm8x41 \
	dev/qpnp_haptic \
	dev/vib \
	lib/libfdt \
	dev/qpnp_wled \
	dev/pmic/pmi8994 \
	dev/pmic/fgsram \
	dev/gcdb/display

DEFINES += \
	MEMSIZE=$(MEMSIZE) \
	MEMBASE=$(MEMBASE) \
	BASE_ADDR=$(BASE_ADDR) \
	SCRATCH_ADDR=$(SCRATCH_ADDR)


OBJS += \
	$(LOCAL_DIR)/init.o \
	$(LOCAL_DIR)/meminfo.o \

ifneq ($(DISPLAY_USE_CONTINUOUS_SPLASH),1)
OBJS += \
	$(LOCAL_DIR)/target_display.o \
	$(LOCAL_DIR)/oem_panel.o
endif
ifeq ($(ENABLE_SMD_SUPPORT),1)
OBJS += \
	$(LOCAL_DIR)/regulator.o
endif
ifeq ($(ENABLE_MDTP_SUPPORT),1)
OBJS += \
	$(LOCAL_DIR)/mdtp_defs.o
endif
ifeq ($(ENABLE_EARLY_CAMERA_SUPPORT),1)
OBJS += \
       $(LOCAL_DIR)/target_camera.o \
       $(LOCAL_DIR)/target_camera_csiphy.o \
       $(LOCAL_DIR)/target_camera_csid.o \
       $(LOCAL_DIR)/target_camera_ispif.o \
       $(LOCAL_DIR)/target_camera_isp.o
endif
