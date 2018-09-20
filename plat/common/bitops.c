/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Code base on:
 * https://github.com/freebsd/freebsd/tree/master/sys/libkern/
 * fls.c/flsl.c/flsll.c/ffs.c/ffsl.c
 *
 * Authors: Wei Chen <wei.chen@arm.com>
 *
 * Copyright (c) 2018, Arm Ltd., All rights reserved.
 * Copyright (c) 1990, 1993
 *	The Regents of the University of California.  All rights reserved.
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
#define _FLSx(x)			\
({					\
	int lsb;			\
	if (x == 0)			\
		return 0;		\
	for (lsb = 1; x != 1; lsb++)	\
		x = x >> 1;	\
	lsb;				\
})

#define _FFSx(x)			\
({					\
	int fsb;			\
	if (x == 0)			\
		return 0;		\
	for (fsb = 1; !(x & 1); fsb++)	\
		x = x >> 1;		\
	fsb;				\
})

/**
 * ukarch_fls - find last (highest) set bit in word for all architectures.
 * @word: The word to search
 *
 * Undefined if no bit exists, so code should check against 0 first.
 */
unsigned int ukarch_fls(unsigned int word)
{
	return _FLSx(word);
}

/**
 * ukarch_flsl - find last (highest) set bit in word for all architectures.
 * @word: The word to search
 *
 * Undefined if no bit exists, so code should check against 0 first.
 */
unsigned int ukarch_flsl(unsigned long word)
{
	return _FLSx(word);
}

/**
 * ukarch_ffs - find first (lowest) set bit in word for all architectures.
 * @word: The word to search
 *
 * Undefined if no bit exists, so code should check against 0 first.
 */
unsigned long ukarch_ffs(unsigned int word)
{
	return _FFSx(word);
}


/**
 * ukarch_ffsl - find first (lowest) set bit in word for all architectures.
 * @word: The word to search
 *
 * Undefined if no bit exists, so code should check against 0 first.
 */
unsigned long ukarch_ffsl(unsigned long word)
{
	return _FFSx(word);
}
