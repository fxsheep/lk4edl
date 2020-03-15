#include <debug.h>
#include "fastboot.h"
#include <dev/fbcon.h>

void fastboot_send_string(const void* _data, size_t size) {
	uint32_t i;
	char buf[MAX_RSP_SIZE];
	const uint8_t* data = _data;

	for(i=0; i<size; i+=MAX_RSP_SIZE-5) {
		uint32_t copysize = MIN(size-i, MAX_RSP_SIZE-5);
		memcpy(buf, &data[i], copysize);
		buf[copysize] = 0;
		fastboot_info(buf);
	}
}

void fastboot_send_string_human(const void* _data, size_t size) {
	uint32_t i;
	char buf[MAX_RSP_SIZE];
	size_t pos = 0;
	const char* data = _data;

	if(size==0)
		size=strlen(data);

	for(i=0; i<size; i++) {
		char c = data[i];
		buf[pos++] = c;

		if(pos==sizeof(buf)-1-4 || i==size-1 || c=='\n' || c=='\r') {
			buf[pos] = 0;
			fastboot_info(buf);
			pos = 0;
		}
	}
}

#if WITH_DEBUG_LOG_BUF
static void cmd_oem_lk_log(const char *arg, void *data, unsigned sz)
{
    fastboot_send_string_human(lk_log_getbuf(), lk_log_getsize());
    fastboot_okay("");
}
#endif

static void cmd_oem_fbconfig(const char *arg, void *data, unsigned sz)
{
    struct fbcon_config *config = fbcon_display();
    char buf[1024];

    fastboot_info("fbcon_config:");

    snprintf(buf, sizeof(buf), "\tbase: %p (end: %p)", (void *)config->base, config->base + (config->width * config->height * config->bpp/3));
    fastboot_info(buf);
    snprintf(buf, sizeof(buf), "\twidth: %u", config->width);
    fastboot_info(buf);
    snprintf(buf, sizeof(buf), "\theight: %u", config->height);
    fastboot_info(buf);
    snprintf(buf, sizeof(buf), "\tstride: %u", config->stride);
    fastboot_info(buf);
    snprintf(buf, sizeof(buf), "\tbpp: %u", config->bpp);
    fastboot_info(buf);
    snprintf(buf, sizeof(buf), "\tformat: %u", config->format);
    fastboot_info(buf);
    snprintf(buf, sizeof(buf), "\tupdate_start: %p", config->update_start);
    fastboot_info(buf);
    snprintf(buf, sizeof(buf), "\tupdate_done: %p", config->update_done);
    fastboot_info(buf);

    fastboot_okay("");
}

void fastboot_lk2nd_register_commands(void) {
#if WITH_DEBUG_LOG_BUF
	fastboot_register("oem lk_log", cmd_oem_lk_log);
#endif
        fastboot_register("oem fbconfig", cmd_oem_fbconfig);
}
