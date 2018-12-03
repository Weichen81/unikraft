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

int fdt_getprop_u32_by_offset(const void *fdt, int offset,
		const char *name, uint32_t *out)
{
	const fdt32_t *prop;
	int prop_len;

	prop = fdt_getprop(fdt, offset, name, &prop_len);
	if (!prop)
		return prop_len;

	if (prop_len >= sizeof(fdt32_t)) {
		*out = fdt32_to_cpu(prop[0]);
		return 0;
	}

	return FDT_ERR_NOTFOUND;
}

int fdt_find_irq_parent_offset(const void *fdt, int offset)
{
	uint32_t irq_parent;

	do {
		/* Find the interrupt-parent phandle */
		if (!fdt_getprop_u32_by_offset(fdt, offset,
				"interrupt-parent", &irq_parent))
			break;

		/* Try to find in parent node */
		offset = fdt_parent_offset(fdt, offset);
	} while (offset >= 0);

	if (offset < 0)
		return offset;

	/* Get interrupt parent node by phandle */
	return fdt_node_offset_by_phandle(fdt, irq_parent);
}


int fdt_interrupt_cells(const void *fdt, int offset)
{
	int intc_offset, len, val;
	const fdt32_t *prop;

	intc_offset = fdt_find_irq_parent_offset(fdt ,offset);
	if (intc_offset < 0)
		return intc_offset;

	prop = fdt_getprop(fdt, intc_offset, "#interrupt-cells", &len);
	if (!prop)
		return len;

	val = fdt32_to_cpu(*prop);
	if ((val <= 0) || (val > FDT_MAX_NCELLS))
		return -FDT_ERR_BADNCELLS;

	return val;
}
