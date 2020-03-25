#include <stdlib.h>
#include <string.h>
#include "fastboot.h"
#include <arch/defines.h>
#include <mmc.h>
#include <partition_parser.h>
#include <platform.h>
#include <target.h>
#include <reboot.h>
#include <lib/elf.h>

void mmu_dacr_off(void) {
        __asm("MOV R0, #0xFFFFFFFF; MCR p15,0,R0,c3,c0,0;");
}


static void process_elf_blob(const void *start, size_t len) {
    void *entrypt;
    elf_handle_t elf;

    status_t st = elf_open_handle_memory(&elf, start, len);
    if (st < 0) {
        dprintf(CRITICAL, "unable to open elf handle\n");
        return;
    }

    st = elf_load(&elf);
    if (st < 0) {
        dprintf(CRITICAL, "elf processing failed, status : %d\n", st);
        goto exit;
    }

    entrypt = (void *)elf.entry;
/***
    if (entrypt < start || entrypt >= (void *)((char *)start + len)) {
        dprintf(CRITICAL, "out of bounds entrypoint for elf : %p\n", entrypt);
        goto exit;
    }
***/

    dprintf(INFO, "elf looks good\n");
    //elf_close_handle(&elf);
    
    void (*elf_start)(void) = (void *)entrypt;
    dprintf(INFO, "elf (%p) running ...\n", entrypt);

    target_uninit();
    platform_uninit();
    
    __asm("LDR R0, =0x08003100;");
    __asm("LDR PC, =0x08006730;");
    //elf_start();
    dprintf(INFO, "elf (%p) finished\n", entrypt);

exit:
    elf_close_handle(&elf);
}

void cmd_boot_edl(void) {
        int *patch1;
	patch1 = 0x0802219C; //boot_hand_control_to_deviceprogrammer_ddr_main
	fastboot_info("Booting to EDL from LK...");
        fastboot_info("Applying some patches first");
	*patch1 = 0x47702000; //BX LR

//PBL shared data patch, Start	
        
	//This disables forced EDL mode flag, avoiding boot_dload_check
	int *pbl2sbl_dload;
        pbl2sbl_dload = 0x08003116;
        *pbl2sbl_dload = 0x0;


//PBL shared data patch, End
        //int *patch3;
        //patch3 = 0x08020014;
        //*patch3 = 0xE7FEE7FE;

	target_uninit();
    	platform_uninit();
	__asm("MOV R0, #0xFFFFFFFF; MCR p15,0,R0,c3,c0,0;");
        __asm("LDR R0, =0x08003100; LDR PC, =0x08008B30;");

}

void cmd_load_sbl1(void) {
        char buf[1024];
        int index = INVALID_PTN;
        unsigned long long ptn = 0;
        uint32_t blocksize, realsize, readsize;
        uint32_t *elf_buffer;
	fastboot_info("Start chainloading SBL1...");
	elf_buffer = target_get_scratch_address();
        if(!elf_buffer){
                dprintf(CRITICAL, "ERROR: failed to allocate memory\n");
                goto fail;
        }

        fastboot_info("Reading sbl1bak from mmc...");
        index = partition_get_index("sbl1bak");
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

	readsize = partition_get_size(index);
        if (readsize == 0) {
                dprintf(CRITICAL, "ERROR:Invalid partition size\n");
                goto fail;
        }

        if (mmc_read(ptn, (uint32_t *)elf_buffer, blocksize)) {
                dprintf(CRITICAL, "ERROR: Cannot read splash image header\n");
                goto fail;
        }
	
	mmu_dacr_off();

        snprintf(buf, sizeof(buf), "\treadsize: %d", readsize);
        fastboot_info(buf);
        fastboot_okay("");

//        target_uninit();
//  	platform_uninit();

	process_elf_blob(elf_buffer, readsize);
	//Never
	return;
fail:

        fastboot_fail("");

}

#if 0
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
#endif

void fastboot_rpm_register_commands(void) {
//        fastboot_register("oem rpm-read-fw", cmd_rpm_read_fw);
        fastboot_register("oem boot-edl",cmd_boot_edl);
	fastboot_register("oem load-sbl1",cmd_load_sbl1);
}
