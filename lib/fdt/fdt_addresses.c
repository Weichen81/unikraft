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

int fdt_address_cells(const void *fdt, int nodeoffset)
{
	return fdt_get_cells(fdt, "#address-cells", nodeoffset);
}

int fdt_size_cells(const void *fdt, int nodeoffset)
{
	return fdt_get_cells(fdt, "#size-cells", nodeoffset);
}

static uint64_t fdt_reg_read_number(const fdt32_t *regs, uint32_t size)
{
	uint64_t number = 0;

	if (size >= 3 || size <= 0)
		return FDT_ERR_BADNCELLS;

	for(uint32_t i = 0; i < size; i++) {
		number <<= 32;
		number |= fdt32_to_cpu(*regs);
		regs++;
	}

	return number;
}

int fdt_get_address(const void *fdt, int nodeoffset, int index,
			uint64_t *addr, uint64_t *size)
{
	int len, prop_addr, prop_size;
	int naddr, nsize, term_size;
	const void *regs;

	naddr = fdt_address_cells(fdt, nodeoffset);
	if (naddr < 0 || naddr >= FDT_MAX_NCELLS)
		return FDT_ERR_BADNCELLS;

	nsize = fdt_size_cells(fdt, nodeoffset);
	if (nsize < 0 || nsize >= FDT_MAX_NCELLS)
		return FDT_ERR_BADNCELLS;

	/* Get reg content */
	regs = fdt_getprop(fdt, nodeoffset, "reg", &len);
	if (regs == NULL)
		return FDT_ERR_NOTFOUND;

	term_size = (int)sizeof(fdt32_t) * (nsize + naddr);
	prop_addr = term_size * index;
	prop_size = prop_addr + (int)sizeof(fdt32_t) * naddr;

	/* The reg content must cover the reg term[index] at least */
	if (len < (prop_addr + term_size))
		return FDT_ERR_NOSPACE;

	*addr = fdt_reg_read_number(regs + prop_addr, naddr);
	*size = fdt_reg_read_number(regs + prop_size, nsize);

	return 0;
}
