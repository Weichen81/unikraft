/* SPDX-License-Identifier: ISC */
/*
 * Authors: Wei Chen <Wei.Chen@arm.com>
 * Authors: Jianyong Wu <Jianyong.Wu@arm.com>
 *
 * Copyright (c) 2018 Arm Ltd.
 *
 * Permission to use, copy, modify, and/or distribute this software
 * for any purpose with or without fee is hereby granted, provided
 * that the above copyright notice and this permission notice appear
 * in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
 * WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE
 * AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR
 * CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS
 * OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */
#include <libfdt.h>
#include <kvm/console.h>
#include <uk/assert.h>
#include <kvm-arm/mm.h>
#include <arm/cpu.h>
#include <arm/fdt.h>
#include <uk/arch/limits.h>

void *_libkvmplat_dtb;

/*
 * uk_dtb_find_device find device offset in dtb by
 * searching device list
 */
int uk_dtb_find_device(const char *device_list[],
	uint32_t size)
{
	uint32_t idx;
	int device = -1;

	for (idx = 0;
		idx < size / sizeof(device_list[0]);
		idx++) {
		device = fdt_node_offset_by_compatible(_libkvmplat_dtb, -1,
		device_list[idx]);
		if (device >= 0)
		break;
	}

	return device;
}

/*
 * uk_dtb_read_reg read specified device address and
 * size in reg region from dtb
 */
uint64_t uk_dtb_read_reg(int device, uint32_t index,
	uint64_t *size)
{
	int  prop_len, prop_min_len;
	uint32_t naddr, nsize, term_size;
	uint64_t addr;
	fdt32_t *regs;

	UK_ASSERT(device != -1);
	naddr = fdt_address_cells(_libkvmplat_dtb, device);
	UK_ASSERT(naddr < FDT_MAX_NCELLS);

	nsize = fdt_size_cells(_libkvmplat_dtb, device);
	UK_ASSERT(nsize < FDT_MAX_NCELLS);

	regs = fdt_getprop(_libkvmplat_dtb, device, "reg", &prop_len);
	prop_min_len = (int)sizeof(fdt32_t) * (naddr + nsize);
	UK_ASSERT(regs != NULL && prop_len >= prop_min_len);

	term_size = nsize + naddr;
	index = index * term_size;
	if(sizeof(fdt32_t) * (index + 1) > prop_len)
		UK_CRASH("too large index\n");

	addr = uk_dtb_read_number(regs + index, naddr);

	*size = uk_dtb_read_number(regs + index + naddr, nsize);

	return addr;
}

/* this reads a number from a given dtb cell address */
uint64_t uk_dtb_read_number(fdt32_t *regs, uint32_t size)
{
	uint64_t number = 0;
	UK_ASSERT(size < 3 && size > 0);

	for(uint32_t i = 0; i < size; i++)
	{
		number <<= 32;
		number |= fdt32_to_cpu(*regs);
		regs++;
	}

	return number;
}
