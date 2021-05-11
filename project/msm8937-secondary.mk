# top level project rules for the msm8937 project
#
LOCAL_DIR := $(GET_LOCAL_DIR)


include $(LOCAL_DIR)/msm8952.mk

# Use maximum verbosity
DEBUG := 9
DEFINES += LK_LOG_BUF_SIZE=16384

# Avoid writing device info
DEFINES += SAFE_MODE=1
# Display as unlocked by default
DEFINES += DEFAULT_UNLOCK=1

# Use continuous splash from primary bootloader for display
#DISPLAY_USE_CONTINUOUS_SPLASH := 1
