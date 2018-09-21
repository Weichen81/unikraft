/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Authors: Wei Chen <Wei.Chen@arm.com>
 *          Jianyong Wu <Jianyong.Wu@arm.com>
 *
 * Copyright (c) 2018, Arm Ltd. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * THIS HEADER MAY NOT BE EXTRACTED OR MODIFIED IN ANY WAY.
 */
#include <string.h>
#include <libfdt.h>
#include <uk/assert.h>
#include <uk/essentials.h>
#include <uk/print.h>
#include <arm/cpu.h>

static void *rtc_base_addr;
uint32_t rtc_boot_seconds;

/* Define offset of RTC registers */
#define RTC_REG_DR	0
#define RTC_REG_MR	0x4
#define RTC_REG_LR	0x8
#define RTC_REG_CR	0xc
#define RTC_REG_IMSC	0x10
#define RTC_REG_RIS	0x14
#define RTC_REG_MIS	0x18
#define RTC_REG_ICR	0x1c

#define RTC_REG(r)	(rtc_base_addr + (r))

static char *rtc_device_list[] = {
	"arm,pl031",
	"arm,primecell",
};

uint32_t rtc_read(void)
{
	return ioreg_read32(RTC_REG(RTC_REG_DR));
}

/*
 * set rtc match register comparing with counter
 * value to generat a interrupt
 */
void rtc_set_match(uint32_t alam)
{
	ioreg_write32(RTC_REG(RTC_REG_MR), alam);
}

void rtc_update(uint32_t val)
{
	ioreg_write32(RTC_REG(RTC_REG_LR), val);
}

void rtc_enable(void)
{
	ioreg_write32(RTC_REG(RTC_REG_CR), 1);
}

/* return rtc status, 1 denotes enable and 0 denotes disable */
uint32_t rtc_get_status(void)
{
	uint32_t val;

	val = ioreg_read32(RTC_REG(RTC_REG_CR));
	val &= 0x1;
	return val;
}

/* mask alam */
void rtc_mask_intr(void)
{
	ioreg_write32(RTC_REG(RTC_REG_IMSC), 1);
}

/* clear alam mask */
void rtc_unmask_intr(void)
{
	ioreg_write32(RTC_REG(RTC_REG_IMSC), 0);
}

/* return the raw state of rtc interrupt before masking*/
uint32_t rtc_get_intr_raw_state(void)
{
	return ioreg_read32(RTC_REG(RTC_REG_RIS));
}

/* return interrupt state after interrupt masking */
uint32_t rtc_get_intr_state(void)
{
	return ioreg_read32(RTC_REG(RTC_REG_MIS));
}

void rtc_clear_intr(void)
{
	ioreg_write32(RTC_REG(RTC_REG_ICR), 1);
}

int _dtb_init_rtc(void *dtb)
{
	uint32_t idx;
	int fdt_rtc, naddr, nsize, prop_len, prop_min_len;
	const uint64_t *regs;

	uk_printd(DLVL_INFO, "Probing RTC...\n");
	for (idx = 0;
		idx < sizeof(rtc_device_list) / sizeof(rtc_device_list[0]);
		idx++) {
		fdt_rtc = fdt_node_offset_by_compatible(dtb, -1,
				rtc_device_list[idx]);
                if (fdt_rtc >= 0)
                        break;
	}

	if (fdt_rtc < 0)
		UK_CRASH("No valid RTC device find\n");

	naddr = fdt_address_cells(dtb, fdt_rtc);
	if (naddr < 0 || naddr >= FDT_MAX_NCELLS)
		UK_CRASH("Could not find proper address cells!\n");

	nsize = fdt_size_cells(dtb, fdt_rtc);
	if (nsize < 0 || nsize >= FDT_MAX_NCELLS)
		UK_CRASH("Could not find proper size cells!\n");

        regs = fdt_getprop(dtb, fdt_rtc, "reg", &prop_len);

	/*
	 * The property must contain at least the start address and size.
	 */
	prop_min_len = (int)sizeof(fdt32_t) * (naddr + nsize);
	if (regs == NULL || prop_len < prop_min_len)
		UK_CRASH("Bad 'reg' property: %p %d\n", regs, prop_len);

	rtc_base_addr = (void *)fdt64_to_cpu(regs[0]);

	/* Record the boot seconds */
	rtc_boot_seconds = rtc_read();

	uk_printd(DLVL_INFO, "Found RTC on: %p\n", rtc_base_addr);
}
