/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Authors: Simon Kuenzer <simon.kuenzer@neclab.eu>
 *
 *
 * Copyright (c) 2017, NEC Europe Ltd., NEC Corporation. All rights reserved.
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
#include <sections.h>

#include <common/gnttab.h>
#if (defined __X86_32__) || (defined __X86_64__)
#include <xen-x86/setup.h>
#elif (defined __ARM_32__) || (defined __ARM_64__)
#include <xen-arm/setup.h>
#endif

#include <uk/assert.h>

int ukplat_memregion_count(void)
{
	return (int) _libxenplat_mrd_num + 5;
}

int ukplat_memregion_get(int i, struct ukplat_memregion_desc *m)
{

	UK_ASSERT(m);

	switch(i) {
	case 0: /* text */
		m->base  = (void *) __TEXT;
		m->len   = (size_t) __ETEXT - (size_t) __TEXT;
		m->flags = (UKPLAT_MEMRF_RESERVED
			    | UKPLAT_MEMRF_READABLE);
#if CONFIG_UKPLAT_MEMRNAME
		m->name  = "text";
#endif
		break;
	case 1:	/* ro data */
		m->base  = (void *) __RODATA;
		m->len   = (size_t) __ERODATA - (size_t) __RODATA;
		m->flags = (UKPLAT_MEMRF_RESERVED
			       | UKPLAT_MEMRF_READABLE);
#if CONFIG_UKPLAT_MEMRNAME
		m->name  = "rodata";
#endif
		break;
	case 2: /* ctors */
		m->base  = (void *) __CTORS;
		m->len   = (size_t) __ECTORS - (size_t) __CTORS;
		m->flags = (UKPLAT_MEMRF_RESERVED
			    | UKPLAT_MEMRF_READABLE);
#if CONFIG_UKPLAT_MEMRNAME
		m->name  = "ctors";
#endif
		break;
	case 3: /* data */
		m->base  = (void *) __DATA;
		m->len   = (size_t) __EDATA - (size_t) __DATA;
		m->flags = (UKPLAT_MEMRF_RESERVED
			    | UKPLAT_MEMRF_READABLE
			    | UKPLAT_MEMRF_WRITABLE);
#if CONFIG_UKPLAT_MEMRNAME
		m->name  = "data";
#endif
		break;
	case 4: /* bss */
		m->base  = (void *) __BSS_START;
		m->len   = (size_t) __END - (size_t) __BSS_START;
		m->flags = (UKPLAT_MEMRF_RESERVED
			    | UKPLAT_MEMRF_READABLE
			    | UKPLAT_MEMRF_WRITABLE);
#if CONFIG_UKPLAT_MEMRNAME
		m->name  = "bss";
#endif
		break;
	default:
		if (i < 0 || i >= ukplat_memregion_count()) {
			m->base  = __NULL;
			m->len   = 0;
			m->flags = 0x0;
#if CONFIG_UKPLAT_MEMRNAME
			m->name  = __NULL;
#endif
			return -1;
		} else {
			memcpy(m, &_libxenplat_mrd[i - 5], sizeof(*m));
		}
		break;
	}

	return 0;
}

int _ukplat_mem_mappings_init(void)
{
	gnttab_init();
	return 0;
}
