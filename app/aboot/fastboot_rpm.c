#include <stdlib.h>
#include <string.h>
#include "fastboot.h"
#include "elf.h"
#include <arch/defines.h>
#include <mmc.h>
#include <partition_parser.h>
#include <platform.h>
#include <target.h>
#include <reboot.h>

void cmd_rpm_read_fw(void) {
	char buf[1024];
	int index = INVALID_PTN;
	unsigned long long ptn = 0;
	uint32_t blocksize, realsize, readsize;
    	Elf32_Ehdr *elf32hdr;
	elf32hdr = target_get_scratch_address();
	if(!elf32hdr){
		dprintf(CRITICAL, "ERROR: failed to allocate memory\n");
		goto fail;
	}

	fastboot_info("Reading rpm fw from mmc...");
	index = partition_get_index("rpm");
	if (index == 0) {
		dprintf(CRITICAL, "ERROR: Partition not found\n");
		goto fail;
	}

	ptn = partition_get_offset(index);
	if (ptn == 0) {
		dprintf(CRITICAL, "ERROR: Invalid partition\n");
		goto fail;
	}

	mmc_set_lun(partition_get_lun(index));

	blocksize = mmc_get_device_blocksize();
	if (blocksize == 0) {
		dprintf(CRITICAL, "ERROR:Invalid blocksize\n");
		goto fail;
	}

	if (mmc_read(ptn, (uint32_t *)elf32hdr, blocksize)) {
	 	dprintf(CRITICAL, "ERROR: Cannot read splash image header\n");
		goto fail;
	}

	if (elf32hdr->e_ident[EI_CLASS] != ELFCLASS32) {
	 	dprintf(CRITICAL, "ERROR: Not an ELF32 image\n");
		goto fail;
	}
        snprintf(buf, sizeof(buf), "\te_entry: 0x%08x", elf32hdr->e_entry);
        fastboot_info(buf);

	fastboot_send_string_human(elf32hdr,4);
	uint32_t *rpm_addr;
	rpm_addr = 0x200000;
	fastboot_send_string_human(rpm_addr,4);
	addr_t *rpm_addr2;
	rpm_addr2 = 0x290000;
        snprintf(buf, sizeof(buf), "\tEntry: 0x%08x", PA((addr_t)rpm_addr2));
        fastboot_info(buf);
	fastboot_send_string_human(rpm_addr2,4);
	fastboot_okay("");
fail:

	fastboot_fail("");

}

void fastboot_rpm_register_commands(void) {
        fastboot_register("oem rpm-read-fw", cmd_rpm_read_fw);
}
