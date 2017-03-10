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

#include <sys/types.h>
#include <stdint.h>
#include <asm.h>
#include <debug.h>
#include "psci.h"

#pragma GCC optimize ("O0")

#define __asmeq(x, y)  ".ifnc " x "," y " ; .err ; .endif\n\t"

enum
{
    PSCI_CPU_OFF = 0x84000002,
    PSCI_CPU_ON  = 0x84000003,
} PSC_CPU_FUNC;

static int psci_call(uint32_t fn, uint32_t cpu_id, uint32_t entry_point, uint32_t context)
{
    register uint32_t r0 __asm__ ("r0") = fn;
    register uint32_t r1 __asm__ ("r1") = cpu_id;
    register uint32_t r2 __asm__ ("r2") = entry_point;
    register uint32_t r3 __asm__ ("r3") = context;

    __asm__ volatile(
                __asmeq("%0", "r0")
                __asmeq("%1", "r1")
                __asmeq("%2", "r2")
                __asmeq("%3", "r3")
                "smc   #0\n"
                : "=r" (r0), "=r" (r1), "=r" (r2), "=r" (r3));

    return r0;
}

/* Turn on the given CPU core */
int psci_cpu_on(uint32_t cpu_id, uint32_t phys_addr)
{
    uint32_t func_id = PSCI_CPU_ON;
    int err = 0;

    if ((err = psci_call(func_id, cpu_id, phys_addr, 0)))
        dprintf(CRITICAL, "Tuning on the CPU failed: %x\n", cpu_id);

    return err;
}

/*
 * Turn off the CPU core calling this function.
 * Note: If this call succeeds, it is not expected to return
 * */
int psci_cpu_off()
{
    uint32_t func_id = PSCI_CPU_OFF;
    int err = 0;

    if ((err = psci_call(func_id, 0, 0, 0)))
        dprintf(CRITICAL, "Tuning off the CPU failed: \n");

    return err;
}
