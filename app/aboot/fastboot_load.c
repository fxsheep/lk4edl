#include <stdlib.h>
#include <string.h>
#include "fastboot.h"
#include <arch/defines.h>
#include <asm.h>
#include <mmc.h>
#include <partition_parser.h>
#include <platform.h>
#include <target.h>
#include <reboot.h>
#include <lib/elf.h>

#define PBL_SIZE (98304)
#define PBL_BASE_ADDR (0x100000)
#define PBL_COPY_ADDR (0x08080000) //0x8068000 DOESN'T WORK


#define PT_GET_TYPE(x) (x & 3)

#define PT_FLD_GET_TYPE(x) (PT_GET_TYPE(x))
#define PT_SLD_GET_TYPE(x) (PT_GET_TYPE(x))

#define PT_FLD_TYPE_FAULT    (0)
#define PT_FLD_TYPE_PT       (1)
#define PT_FLD_TYPE_SECTION  (2)
#define PT_FLD_TYPE_RESERVED (3)


#define PT_SLD_TYPE_UNSUPPORTED   (0)
#define PT_SLD_TYPE_LP            (1)
#define PT_SLD_TYPE_XSP           (2)
#define PT_SLD_TYPE_XSP_WITH_NX   (3)

#define PT_FL_OFFSET(x) ((uint32_t)x >> 20)
#define PT_SL_OFFSET(x) (((uint32_t)x & 0xFF000)>>12)


void mmu_dacr_off(void) {
        __asm("MOV R0, #0xFFFFFFFF; MCR p15,0,R0,c3,c0,0;");
}

/*
 * Returns the first level descriptor (page table entry) of the given virtual address
 */
uint32_t pt_get_first_level_descriptor(uint32_t *addr)
{
    uint32_t *base = (uint32_t *)arm_read_ttbr();
    dprintf(INFO, "arm_read_ttbr:%x\n", base);
    return base[PT_FL_OFFSET(addr)];
}


/*
 * Returns the address of the second level descriptor (page table entry) for the given virtual address
 */
uint32_t *pt_get_second_level_descriptor_ptr(uint32_t *addr)
{
    uint32_t fl = pt_get_first_level_descriptor(addr);
    dprintf(INFO, "pt_get_first_level_descriptor:%x\n", fl);

    //assert PT_FLD_GET_TYPE(fl) == PT_FLD_TYPE_PT;
    uint32_t *base = (uint32_t *)((fl >> 10) << 10);
    return &base[PT_SL_OFFSET(addr)];
}


/*
 * Returns the second level descriptor (page table entry) of the given virtual address
 */
uint32_t pt_get_second_level_descriptor(uint32_t *addr)
{
    return *pt_get_second_level_descriptor_ptr(addr);
}


/*
 * Sets the second level descriptor (page table entry) of the given virtual address with the given value
 */
void pt_set_second_level_descriptor(uint32_t *addr, uint32_t val)
{
    uint32_t *sladdr = pt_get_second_level_descriptor_ptr(addr);
    *sladdr = val;
}


/*
 * Sets the second level descriptor (page table entry) of the given virtual address (va)
 * with the second level descriptor of another virtual address (new_va)
 */

void pt_second_level_xsmallpage_remap(uint32_t *va, uint32_t *new_va) 
{
    uint32_t new_sl = pt_get_second_level_descriptor(new_va);
    dprintf(INFO, "pt_get_second_level_descriptor:%x\n", new_sl);
    pt_set_second_level_descriptor(va, new_sl);
}


void pageremap(void) {
    arm_write_cr1(arm_read_cr1() | 0x0);
    dprintf(INFO, "Remapping pages\n");
    pt_second_level_xsmallpage_remap(PBL_BASE_ADDR, PBL_COPY_ADDR);   
    pt_second_level_xsmallpage_remap(PBL_BASE_ADDR + 0x3000, PBL_COPY_ADDR + 0x3000);
    pt_second_level_xsmallpage_remap(PBL_BASE_ADDR + 0x4000, PBL_COPY_ADDR + 0x4000);
    pt_second_level_xsmallpage_remap(PBL_BASE_ADDR + 0x5000, PBL_COPY_ADDR + 0x5000);
    pt_second_level_xsmallpage_remap(PBL_BASE_ADDR + 0xB000, PBL_COPY_ADDR + 0xB000);
    pt_second_level_xsmallpage_remap(PBL_BASE_ADDR + 0xD000, PBL_COPY_ADDR + 0xD000);
    pt_second_level_xsmallpage_remap(PBL_BASE_ADDR + 0xF000, PBL_COPY_ADDR + 0xF000);
    pt_second_level_xsmallpage_remap(PBL_BASE_ADDR + 0x10000, PBL_COPY_ADDR + 0x10000);
    arm_invalidate_tlb();
    arm_write_cr1(arm_read_cr1() | 0x1);
    return;
}

void patch_pbl(uint32_t *addr, uint32_t value) {
	*addr = value;
	return;
}

void patch_pbl_nop(uint32_t *start, uint32_t *end) {
	int i, n;
	n = (end - start) / 4;
	for(i = 0; i <= n; i++) {
		patch_pbl(start + n * 4, 0xE1A00000);
	}
	return;
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
        fastboot_info("Booting to EDL from LK...");
        target_uninit();
        platform_uninit();
        __asm("MOV R0, #0xFFFFFFFF; MCR p15,0,R0,c3,c0,0;");
        __asm("LDR R0, =0x08003100; LDR PC, =0x08008B30;");

}

void cmd_boot_edl2sbl(void) {
        int *patch1;
	patch1 = 0x0802219C; //boot_hand_control_to_deviceprogrammer_ddr_main
	fastboot_info("Booting to patched EDL from LK...");
        fastboot_info("Applying some patches first");
	*patch1 = 0x47702000; //BX LR

	int *patch;
	patch = 0x08020C2C;
	*patch = 0x49012000;

	patch = 0x08020C30;
	*patch = 0x00006008;

	patch = 0x08020C34;
	*patch = 0x4AB000;

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

void cmd_boot_pbl(void) {
	mmu_dacr_off();
        target_uninit();
        platform_uninit();
        __asm("LDR PC, =0x100000;");

}

void cmd_boot_pbl_patched(void) {
	mmu_dacr_off();
	fastboot_info("Start copying PBL...");
	memcpy(PBL_COPY_ADDR, PBL_BASE_ADDR, PBL_SIZE);
        fastboot_info("Remapping PBL");
	pageremap();
	fastboot_info("Patching PBL");

	//DACR:Set ourselves as manager
//	patch_pbl(0x110008, 0xE3E00000); //IDK WHY THIS DOESN'T WOR, kek
	//Disable MMU reset
	patch_pbl(0x110014, 0xE1A00000);
	//Disable page table init
	patch_pbl_nop(0x110678, 0x1107B8);
	patch_pbl(0x1107B8, 0xE3A05000);
	//pbl_auth patch (to avoid a side effect)
	patch_pbl(0x103478, 0xEA000004);


	//
	//patch sbl1 GUID to DEADBEEF-CBDD-4805-B4F9-F428251C3E98 , original is DEA0BA2C-CBDD-4805-B4F9-F428251C3E98
	patch_pbl(0x10D314, 0xDEADBEEF);
	//
	//

#if 0
	//test
//	patch_pbl(0x10B908, 0xEAFFFFFE);
//        patch_pbl(0x10CDC4, 0xEAFFFFFE);

#if 0
	//test hash_verify
        patch_pbl(0x10F450, 0xE51F0000);
        patch_pbl(0x10F454, 0xE580E000);
        patch_pbl(0x10F458, 0xB0500000);
        patch_pbl(0x10F45C, 0xEAFFFFFE);
#endif

//pbl_elf_loader works.	
//        patch_pbl(0x105488, 0xEAFFFFFE);

//bootable_media_detect works.	
//	patch_pbl(0x10306C, 0xEAFFFFFE);


//auth_hash_seg works. 
//        patch_pbl(0x1034E8, 0xEAFFFFFE);

//prepare_sbl_entrance works.
//	patch_pbl(0x105AF0, 0xEAFFFFFE);

////jump_to_sbl DOESNT WORK
////	patch_pbl(0x104130, 0xEAFFFFFE);

#endif

	//It turned out that 
	//Remapping of 0x103000, 0x10D000 and 0x105000 works while 0x104000 not
	//Fuck I dont wanna mess with bootrom anymore 
	
	//The following is a really working SBL1 in-place patch
	
	//We targets the prepare_sbl_entrance located at 0x00105AF0
	//First modify the return which is an ldmfd, jump to 0x10DA00 instead,
	//where we place our patching code
	//In arm mode
	patch_pbl(0x105C1C, 0xEA001F77); //	B    sub_10DA00	


	patch_pbl(0x10DA00, 0xE59F1008);
        patch_pbl(0x10DA04, 0xE59F2008);
        patch_pbl(0x10DA08, 0xE5812000);
        patch_pbl(0x10DA0C, 0xE8BD81F0);
        patch_pbl(0x10DA10, 0x0801F50E); //patch SBL1 boot_is_auth_enabled
	patch_pbl(0x10DA14, 0x2001E002); // always return 0

	fastboot_info("Booting now");
	fastboot_okay("");
	target_uninit();
        platform_uninit();
        __asm("LDR PC, =0x100000;");

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

        if (mmc_read(ptn, (uint32_t *)elf_buffer, readsize)) {
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
        fastboot_register("oem boot-edl-sbl",cmd_boot_edl2sbl);
	fastboot_register("oem load-sbl1",cmd_load_sbl1);
        fastboot_register("oem boot-pbl",cmd_boot_pbl);
        fastboot_register("oem boot-pbl-patched",cmd_boot_pbl_patched);
}
