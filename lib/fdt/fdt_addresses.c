/*
 * libfdt - Flat Device Tree manipulation
 * Copyright (C) 2014 David Gibson <david@gibson.dropbear.id.au>
 *
 * libfdt is dual licensed: you can use it either under the terms of
 * the GPL, or the BSD license, at your option.
 *
 *  a) This library is free software; you can redistribute it and/or
 *     modify it under the terms of the GNU General Public License as
 *     published by the Free Software Foundation; either version 2 of the
 *     License, or (at your option) any later version.
 *
 *     This library is distributed in the hope that it will be useful,
 *     but WITHOUT ANY WARRANTY; without even the implied warranty of
 *     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *     GNU General Public License for more details.
 *
 *     You should have received a copy of the GNU General Public
 *     License along with this library; if not, write to the Free
 *     Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston,
 *     MA 02110-1301 USA
 *
 * Alternatively,
 *
 *  b) Redistribution and use in source and binary forms, with or
 *     without modification, are permitted provided that the following
 *     conditions are met:
 *
 *     1. Redistributions of source code must retain the above
 *        copyright notice, this list of conditions and the following
 *        disclaimer.
 *     2. Redistributions in binary form must reproduce the above
 *        copyright notice, this list of conditions and the following
 *        disclaimer in the documentation and/or other materials
 *        provided with the distribution.
 *
 *     THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *     CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *     INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *     MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *     DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 *     CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *     SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 *     NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *     LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 *     HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *     CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 *     OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 *     EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "libfdt_env.h"

#include <fdt.h>
#include <libfdt.h>
#include "libfdt_internal.h"

void *_libkvmplat_dtb;

int fdt_address_cells(const void *fdt, int nodeoffset)
{
	return fdt_get_cells(fdt, "#address-cells", nodeoffset);
}

int fdt_size_cells(const void *fdt, int nodeoffset)
{
	return fdt_get_cells(fdt, "#size-cells", nodeoffset);
}

/*
 * uk_dtb_find_device find device offset in dtb by
 * searching device list
 */
int fdt_lookup_device(const char *device_list[],
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

/* this reads a number from a given dtb cell address */
uint64_t fdt_read_number(fdt32_t *regs, uint32_t size)
{
        uint64_t number = 0;
        if (size >= 3 || size <= 0)
		return FDT_ERR_BADOFFSET;

        for(uint32_t i = 0; i < size; i++)
        {
                number <<= 32;
                number |= fdt32_to_cpu(*regs);
                regs++;
        }

        return number;
}

/*
 * uk_dtb_read_reg read specified device address and
 * size in reg region from dtb
 */
uint64_t fdt_read_reg(int device, uint32_t index,
        uint64_t *size)
{
        int  prop_len, prop_min_len;
        uint32_t naddr, nsize, term_size;
        uint64_t addr;
        fdt32_t *regs;

        if (device == -1)
		return FDT_ERR_BADOFFSET;

        naddr = fdt_address_cells(_libkvmplat_dtb, device);
        if (naddr >= FDT_MAX_NCELLS)
		return FDT_ERR_INTERNAL;

        nsize = fdt_size_cells(_libkvmplat_dtb, device);
        if (nsize >= FDT_MAX_NCELLS)
		return FDT_ERR_INTERNAL;

        regs = fdt_getprop(_libkvmplat_dtb, device, "reg", &prop_len);
        prop_min_len = (int)sizeof(fdt32_t) * (naddr + nsize);
        if (regs == NULL || prop_len < prop_min_len)
		return FDT_ERR_NOTFOUND;

        term_size = nsize + naddr;
        index = index * term_size;
        if(sizeof(fdt32_t) * (index + 1) > prop_len)
                return FDT_ERR_BADOFFSET;

        addr = fdt_read_number(regs + index, naddr);

        *size = fdt_read_number(regs + index + naddr, nsize);

        return addr;
}
