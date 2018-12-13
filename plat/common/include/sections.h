/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Authors: Wei Chen <wei.chen@arm.com>
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

#ifndef __PLAT_CMN_SECTIONS_H__
#define __PLAT_CMN_SECTIONS_H__

/*
 * Following global variables are defined in image link scripts, and some
 * variables are optional and may be unavailable on some architectures
 * or configurations.
 */

/* _dtb: start of device tree */
extern char _dtb[];

/* [_text, _etext]: contains .text.* sections */
extern char _text[], _etext[];

/* [_rodata, _erodata]: contains .rodata.* sections */
extern char _rodata[], _erodata[];

/* [_data, _edata]: contains .data.* sections */
extern char _data[], _edata[];

/* [_ctors, _ectors]: contains constructor tables (read-only) */
extern char _ctors[], _ectors[];

/* __bss_start: start of BSS sections */
extern char __bss_start[];

/* _end: end of kernel image */
extern char _end[];


/* The C spec states that comparing 2 pointers belonging to distinct object is undefined:
 * https://wiki.sei.cmu.edu/confluence/display/c/ARR36-C.+Do+not+subtract+or+compare+two+pointers+that+do+not+refer+to+the+same+array
 *
 * In this case, we define following macro to access above symbols. This will
 * help us to avoid subtracting or comparing these symbols as distinct pointers.
 *
 * This macro hide the original variable for a variable address, it will make
 * GCC couldn't recognize varirable's original type, and do further assumptions
 * for it.
 */
#define HIDE_VAR_FOR_GCC(var, ofs)	\
({					\
	unsigned long __var;		\
	__var = (unsigned long) (var);	\
	(typeof(var))(__var + (ofs));	\
})

#define __uk_image_symbol(addr)	HIDE_VAR_FOR_GCC((unsigned long)(addr), 0)

#define __DTB		__uk_image_symbol(_dtb)
#define __TEXT		__uk_image_symbol(_text)
#define __ETEXT		__uk_image_symbol(_etext)
#define __RODATA	__uk_image_symbol(_rodata)
#define __ERODATA	__uk_image_symbol(_erodata)
#define __DATA		__uk_image_symbol(_data)
#define __EDATA		__uk_image_symbol(_edata)
#define __CTORS		__uk_image_symbol(_ctors)
#define __ECTORS	__uk_image_symbol(_ectors)
#define __BSS_START	__uk_image_symbol(__bss_start)
#define __END		__uk_image_symbol(_end)

#endif /* __PLAT_CMN_SECTIONS_H__ */
