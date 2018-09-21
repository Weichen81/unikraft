/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Port from Mini-OS: include/arm/os.h
 */
/*
 * Authors: Karim Allah Ahmed <karim.allah.ahmed@gmail.com>
 *          Thomas Leonard <talex5@gmail.com>
 *          Wei Chen <Wei.Chen@arm.com>
 *
 * Copyright (c) 2014 Karim Allah Ahmed
 * Copyright (c) 2014 Thomas Leonard
 * Copyright (c) 2018, Arm Ltd., All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#ifndef __UKARCH_ATOMIC_H__
#error Do not include this header directly
#endif

/**
 * ukarch_ffsl - find first (lowest) set bit in word.
 * @word: The word to search
 *
 * Undefined if no bit exists, so code should check against 0 first.
 */
static inline unsigned long ukarch_ffsl(unsigned long word)
{
	int clz;

	/* xxxxx10000 = word
	 * xxxxx01111 = word - 1
	 * 0000011111 = word ^ (word - 1)
	 *      4     = 63 - clz(word ^ (word - 1))
	 */

	__asm__("sub x0, %[word], #1\n"
		"eor x0, x0, %[word]\n"
		"clz %[clz], x0\n"
		:
		/* Outputs: */
		[clz] "=r"(clz)
		:
		/* Inputs: */
		[word] "r"(word)
		:
		/* Clobbers: */
		"x0");

	return 63 - clz;
}

/**
 * ukarch_fls - find last (highest) set bit in word.
 * @word: The word to search
 *
 * Undefined if no bit exists, so code should check against 0 first.
 */
unsigned int ukarch_fls(unsigned int word);

/**
 * ukarch_flsl - find last (highest) set bit in word.
 * @word: The word to search
 *
 * Undefined if no bit exists, so code should check against 0 first.
 */
unsigned long ukarch_flsl(unsigned long word);

/**
 * ukarch_ffs - find first (lowest) set bit in word.
 * @word: The word to search
 *
 * Undefined if no bit exists, so code should check against 0 first.
 */
unsigned long ukarch_ffs(unsigned int word);

/**
 * ukarch_ffsl - find first (lowest) set bit in word.
 * @word: The word to search
 *
 * Undefined if no bit exists, so code should check against 0 first.
 */
unsigned long ukarch_ffsl(unsigned long word);
