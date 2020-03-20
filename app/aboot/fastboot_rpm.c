#include <stdlib.h>
#include <string.h>
#include "fastboot.h"
#include "elf.h"
#include <arch/defines.h>
#include <mmc.h>
#include <partition_parser.h>

void cmd_rpm_read_fw(void) {

	int index = INVALID_PTN;
	unsigned long long ptn = 0;
	uint32_t blocksize, realsize, readsize;
    	Elf32_Ehdr *elf32hdr;
	elf32hdr = memalign(CACHE_LINE, sizeof(elf32hdr));
	if(!elf32hdr){
		dprintf(CRITICAL, "ERROR: failed to allocate memory\n");
		goto fail;
	}

	fastboot_info("Reading rpm fw from mmc...");
	index = partition_get_index("rpm");
	if (index == 0) {
		dprintf(CRITICAL, "ERROR: splash Partition table not found\n");
		goto fail;
	}

	ptn = partition_get_offset(index);
	if (ptn == 0) {
		dprintf(CRITICAL, "ERROR: splash Partition invalid\n");
		goto fail;
	}

	mmc_set_lun(partition_get_lun(index));

	blocksize = mmc_get_device_blocksize();
	if (blocksize == 0) {
		dprintf(CRITICAL, "ERROR:splash Partition invalid blocksize\n");
		goto fail;
	}

	if (mmc_read(ptn, (uint32_t *)elf32hdr, blocksize)) {
	 	dprintf(CRITICAL, "ERROR: Cannot read splash image header\n");
		goto fail;
	}
	
	fastboot_okay("");
fail:

	fastboot_fail("");

}

void fastboot_rpm_register_commands(void) {
        fastboot_register("oem rpm-read-fw", cmd_rpm_read_fw);
}
