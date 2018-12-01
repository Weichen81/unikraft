/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Authors: Wei Chen <Wei.Chen@arm.com>
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
#include <stdlib.h>
#include <libfdt.h>
#include <uk/assert.h>
#include <uk/plat/time.h>
#include <uk/plat/lcpu.h>
#include <uk/plat/irq.h>
#include <uk/bitops.h>
#include <cpu.h>

/* Bits definition of cntv_ctl_el0 register */
#define GT_TIMER_ENABLE		0x01
#define GT_TIMER_MASK_IRQ	0x02
#define GT_TIMER_IRQ_STATUS	0x04

static uint64_t boot_ticks;
static uint32_t counter_freq;
extern void *_libkvmplat_dtb;


/* Shift factor for converting ticks to ns */
static uint8_t counter_shift_to_ns;

/* Shift factor for converting ns to ticks */
static uint8_t counter_shift_to_tick;

/* Multiplier for converting counter ticks to nsecs */
static uint32_t ns_per_tick;

/* Multiplier for converting nsecs to counter ticks */
static uint32_t tick_per_ns;

/*
 * The maximum time range in seconds which can be converted by multiplier
 * and shift factors. This will guarantee the converted value not to exceed
 * 64-bit unsigned integer. Increase the time range will reduce the accuracy
 * of conversion, because we will get smaller multiplier and shift factors.
 * In this case, we selected 3600s as the time range.
 */
#define __MAX_CONVERT_SECS	3600UL

/*
 * Minimum delta to sleep using generic timer.
 */
static uint32_t counter_mini_delta;

/* How many nanoseconds per second */
#define NSEC_PER_SEC ukarch_time_sec_to_nsec(1)

static inline uint64_t ticks_to_ns(uint64_t ticks)
{
	return (ns_per_tick * ticks) >> counter_shift_to_ns;
}

static inline uint64_t ns_to_ticks(uint64_t ns)
{
	return (tick_per_ns * ns) >> counter_shift_to_tick;
}

/*
 * Calculate multiplier/shift factors for scaled math.
 */
static void calculate_mult_shift(uint32_t *pmult, uint8_t *pshift,
		uint64_t source, uint64_t target)
{
	uint64_t tmp;
	uint32_t mult;
	uint8_t shift = 32;

	/*
	 * Get the maximum shift factor (max_shift) for the given
	 * conversion range.
	 */
	tmp = (__MAX_CONVERT_SECS * (uint64_t)source) >> shift;
	while (tmp) {
		tmp >>=1;
		shift--;
	}

	/*
	 * Calculate shift factor (S) and scaling multiplier (M).
	 *
	 * (S) needs to be the largest shift factor (<= max_shift) where
	 * the result of the M calculation below fits into uint32_t
	 * without truncation.
	 *
	 * multiplier = (target << shift) / source
	 */
	mult = 0;
	do {
		tmp = ((uint64_t)target << shift) / source;
		if ((tmp & 0xFFFFFFFF00000000L) == 0L)
			mult = (uint32_t)tmp;
		else
			shift--;
	} while (shift > 0 && mult == 0L);

	*pmult = mult;
	*pshift = shift;
}

static inline void generic_timer_enable(void)
{
	SYSREG_WRITE32(cntv_ctl_el0, GT_TIMER_ENABLE);
}

static inline void generic_timer_mask_irq(void)
{
	SYSREG_WRITE32(cntv_ctl_el0,
		SYSREG_READ32(cntv_ctl_el0) | GT_TIMER_MASK_IRQ);
}

static inline void generic_timer_unmask_irq(void)
{
	SYSREG_WRITE32(cntv_ctl_el0,
		SYSREG_READ32(cntv_ctl_el0) & (~GT_TIMER_MASK_IRQ));
}

static inline void generic_timer_update_compare(uint64_t new_val)
{
	SYSREG_WRITE64(cntv_cval_el0, new_val);
}

static int generic_timer_get_fdt_offset(void)
{
	int fdt_timer;

	/* Try to find arm,armv8-timer first */
	fdt_timer = fdt_node_offset_by_compatible(_libkvmplat_dtb,
						-1, "arm,armv8-timer");
	/* If failed, try to find arm,armv7-timer */
	if (fdt_timer < 0)
		return fdt_node_offset_by_compatible(_libkvmplat_dtb,
						-1, "arm,armv7-timer");

	return fdt_timer;
}

static uint32_t generic_timer_get_frequency(int fdt_timer)
{
	int len;
	const uint64_t *fdt_freq;

	/*
	 * On a few platforms the frequency is not configured correctly
	 * by the firmware. A property in the DT (clock-frequency) has
	 * been introduced to workaround those firmware.
	 */
	fdt_freq = fdt_getprop(_libkvmplat_dtb,
			fdt_timer, "clock-frequency", &len);
	if (!fdt_freq || (len <= 0)) {
		uk_pr_info("No clock-frequency found, reading from register directly.\n");
		goto endnofreq;
	}

	return fdt32_to_cpu(fdt_freq[0]);

endnofreq:
	/* No workaround, get from register directly */
	return SYSREG_READ32(cntfrq_el0);
}

static uint32_t generic_timer_get_interrupts(int fdt_timer)
{
	int nintr, len;
	const fdt32_t *regs;

	nintr = fdt_interrupt_cells(_libkvmplat_dtb, fdt_timer);
	if (nintr < 0 || nintr >= FDT_MAX_NCELLS)
		UK_CRASH("Could not find proper interrupt cells!\n");

	/* Otherwise, read from device tree */
	regs = fdt_getprop(_libkvmplat_dtb,
			fdt_timer, "interrupts", &len);
	/*
	 * Interrupt list for secure, non-secure, virtual and
	 * hypervisor timers, in that order.
	 */
	if (regs == NULL || len < (int)sizeof(fdt32_t) * nintr)
		UK_CRASH("Bad 'interrupts' property: %p %d\n", regs, nintr);

	uk_pr_info("mini len:%d, nintr: %d\n", sizeof(fdt32_t) * (nintr * 4), nintr);
	uk_pr_info("len:%d interrupts: %08x %08x %08x\n", len, fdt32_to_cpu(regs[0]), fdt32_to_cpu(regs[1]), fdt32_to_cpu(regs[2] ));
	uk_pr_info("len:%d interrupts: %08x %08x %08x\n", len, fdt32_to_cpu(regs[3]), fdt32_to_cpu(regs[4]), fdt32_to_cpu(regs[5]));
	uk_pr_info("len:%d interrupts: %08x %08x %08x\n", len, fdt32_to_cpu(regs[6]), fdt32_to_cpu(regs[7]), fdt32_to_cpu(regs[8]));
	uk_pr_info("len:%d interrupts: %08x %08x %08x\n", len, fdt32_to_cpu(regs[9]), fdt32_to_cpu(regs[10]), fdt32_to_cpu(regs[11]));
	return fdt32_to_cpu(regs[1]);
}

#ifdef CONFIG_ARM64_ERRATUM_858921
/*
 * The errata #858921 describes that Cortex-A73 (r0p0 - r0p2) counter
 * read can return a wrong value when the counter crosses a 32bit boundary.
 * But newer Cortex-A73 are not affected.
 *
 * The workaround involves performing the read twice, compare bit[32] of
 * the two read values. If bit[32] is different, keep the first value,
 * otherwise keep the second value.
 */
static uint64_t generic_timer_get_ticks(void)
{
    uint64_t val_1st, val_2nd;

    val_1st = SYSREG_READ64(cntvct_el0);
    val_2nd = SYSREG_READ64(cntvct_el0);
    return (((val_1st ^ val_2nd) >> 32) & 1) ? val_1st : val_2nd;
}
#else
static inline uint64_t generic_timer_get_ticks(void)
{
	return SYSREG_READ64(cntvct_el0);
}
#endif

/*
 * monotonic_clock(): returns # of nanoseconds passed since
 * generic_timer_time_init()
 */
static __nsec generic_timer_monotonic(void)
{
	return (__nsec)ticks_to_ns(generic_timer_get_ticks() - boot_ticks);
}

/*
 * Return epoch offset (wall time offset to monotonic clock start).
 */
static uint64_t generic_timer_epochoffset(void)
{
	return 0;
}

/*
 * Returns early if any interrupts are serviced, or if the requested delay is
 * too short. Must be called with interrupts disabled, will enable interrupts
 * "atomically" during idle loop.
 *
 * This function must be called only from the scheduler. It will screw
 * your system if you do otherwise. And, there is no reason you
 * actually want to use it anywhere else. THIS IS NOT A YIELD or any
 * kind of mutex_lock. It will simply halt the cpu, not allowing any
 * other thread to execute.
 */
static void generic_timer_cpu_block(uint64_t until_ns)
{
	uint64_t now_ns, delta_ns;
	uint64_t now_ticks, delta_ticks;

	UK_ASSERT(ukplat_lcpu_irqs_disabled());

	/* Record current ticks */
	now_ticks = generic_timer_get_ticks();
	now_ns = ticks_to_ns(now_ticks - boot_ticks);

	/*
	 * Compute delta in counter ticks. Return if it is less than minimum
	 * safe amount of ticks. Essentially this will cause us to spin until
	 * the timeout.
	 */
	delta_ns = until_ns - now_ns;
	delta_ticks = ns_to_ticks(delta_ns);
	if (delta_ticks < counter_mini_delta) {
		/*
		 * Since we are "spinning", quickly enable interrupts in
		 * the hopes that we might get new work and can do something
		 * else than spin.
		 */
		ukplat_lcpu_enable_irq();
		nop();
		ukplat_lcpu_disable_irq();

		return;
	}

	/* Calculate the next match ticks for compare counter */
	generic_timer_update_compare(now_ticks + delta_ticks);

	/* Unmask the IRQ for next match interrupt */
	generic_timer_unmask_irq();

	/*
	 * Wait for any interrupt. If we got an interrupt then just
	 * return into the scheduler (this func is called _ONLY_ from
	 * a scheduler, see the note above) which will check if there
	 * is work to do and call us again here if not.
	 *
	 * TODO: It would be more efficient for longer sleeps to be
	 * able to distinguish if the interrupt was the timer interrupt
	 * and no other, but this will do for now.
	 */
	ukplat_lcpu_halt_irq();
}

static int generic_timer_init(void)
{
	int fdt_timer;

	/*
	 * Monotonic time begins at boot_ticks (first read of counter
	 * before calibration).
	 */
	boot_ticks = generic_timer_get_ticks();

	/* Save timer fdt offset to get frequency and interrupts */
	fdt_timer = generic_timer_get_fdt_offset();
	if (fdt_timer < 0)
		UK_CRASH("Failed device tree node for generic timer\n");

	/* Get counter frequency from DTB or register */
	counter_freq = generic_timer_get_frequency(fdt_timer);

	/*
	 * Calculate the shift factor and scaling multiplier for
	 * cpnverting ticks to ns.
	 */
	calculate_mult_shift(&ns_per_tick, &counter_shift_to_ns,
				counter_freq, NSEC_PER_SEC);

	/* We disallow zero ns_per_tick */
	UK_BUGON(!ns_per_tick);

	/*
	 * Calculate the shift factor and scaling multiplier for
	 * cpnverting ns to ticks.
	 */
	calculate_mult_shift(&tick_per_ns, &counter_shift_to_tick,
				NSEC_PER_SEC, counter_freq);

	/* We disallow zero ns_per_tick */
	UK_BUGON(!tick_per_ns);

	/*
	 * Set minimal counter delta, programming seems to have an overhead
	 * of 3-4us, but play it safe here.
	 */
	counter_mini_delta = ns_to_ticks(4000);

	uk_pr_info("counter_freq:0x%016x ns_per_tick:0x%08x counter_shift_to_ns: %d tick_per_ns:0x%08x counter_shift_to_tick:%d counter_mini_delta: 0x%08x\n",
		counter_freq, ns_per_tick, counter_shift_to_ns,
		tick_per_ns, counter_shift_to_tick, counter_mini_delta);

	generic_timer_get_interrupts(fdt_timer);

	return 0;
}

static int generic_timer_irq_handler(void *arg __unused)
{
	/*
	 * We just mask the IRQ here, the scheduler will call
	 * generic_timer_cpu_block, and then unmask the IRQ.
	 */
	generic_timer_mask_irq();

	uk_pr_info("...timer_handler...\n");

	/* Yes, we handled the irq. */
	return 1;
}

unsigned long sched_have_pending_events;

void time_block_until(__snsec until)
{
	while ((__snsec) ukplat_monotonic_clock() < until) {
		generic_timer_cpu_block(until);
		if (__uk_test_and_clear_bit(0, &sched_have_pending_events))
			break;
	}
}

/* return ns since time_init() */
__nsec ukplat_monotonic_clock(void)
{
	return generic_timer_monotonic();
}

/* return wall time in nsecs */
__nsec ukplat_clock_wall(void)
{
	return generic_timer_monotonic() + generic_timer_epochoffset();
}

/* must be called before interrupts are enabled */
void ukplat_time_init(void)
{
	int rc;

	/* Register timer IRQ */
	rc = ukplat_irq_register(27, generic_timer_irq_handler, NULL);
	if (rc < 0)
		UK_CRASH("Failed to register timer interrupt handler\n");

	rc = generic_timer_init();
	if (rc < 0)
		UK_CRASH("Failed to initialize platform time\n");

	/*
	 * Mask IRQ before scheduler start working. Otherwise we will get
	 * unexpected timer interrupts when system is booting.
	 */
	generic_timer_mask_irq();

	/* Enable timer */
	generic_timer_enable();
}
