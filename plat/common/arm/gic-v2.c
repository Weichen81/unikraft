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
#include <uk/essentials.h>
#include <uk/print.h>
#include <uk/assert.h>
#include <uk/bitops.h>
#include <uk/asm.h>
#include <irq.h>
#include <arm/cpu.h>
#include <arm/gic-v2.h>

static void *gic_dist_addr, *gic_cpuif_addr;
static uint64_t gic_dist_size, gic_cpuif_size;

#define GIC_DIST_REG(r)	(gic_dist_addr + (r))
#define GIC_CPU_REG(r)	(gic_cpuif_addr + (r))

extern void *_libkvmplat_dtb;
static char *gic_device_list[] = {
	"arm,cortex-a15-gic",
	"arm,cortex-a7-gic",
	"arm,cortex-a9-gic",
	"arm,gic-400",
	"arm,eb11mp-gic",
	"arm,pl390",
	"arm,arm1176jzf-devchip-gic",
	"arm,arm11mp-gic",
	"arm,tc11mp-gic",
	"brcm,brahma-b15-gic",
	"nvidia,tegra210-agic",
	"qcom,msm-8660-qgic",
	"qcom,msm-qgic2",
};

/* inline functions to access GICC & GICD registers */
static inline void write_gicd8(uint64_t offset, uint8_t val)
{
	ioreg_write8(GIC_DIST_REG(offset), val);
}

static inline void write_gicd32(uint64_t offset, uint32_t val)
{
	ioreg_write32(GIC_DIST_REG(offset), val);
}

static inline uint32_t read_gicd32(uint64_t offset)
{
	return ioreg_read32(GIC_DIST_REG(offset));
}

static inline void write_gicc32(uint64_t offset, uint32_t val)
{
        ioreg_write32(GIC_CPU_REG(offset), val);
}

static inline uint32_t read_gicc32(uint64_t offset)
{
	return ioreg_read32(GIC_CPU_REG(offset));
}

/*
 * Functions of GIC CPU interface
 */

/* Enable GIC cpu interface */
static void gic_enable_cpuif(void)
{
	/* just set bit 0 to 1 to enable cpu interface */
	write_gicc32(GICC_CTLR, read_gicc32(GICC_CTLR) | GICC_CTLR_ENABLE);
}

/* Disable GIC cpu interface */
static void gic_disable_cpuif(void)
{
	/* only clear bit 0 to 0 to disable cpu interface */
	write_gicc32(GICC_CTLR, read_gicc32(GICC_CTLR) & (~GICC_CTLR_ENABLE));
}

/* Set priority threshold for processor */
static void gic_set_threshold_priority(uint32_t threshold_prio)
{
	/* GICC_PMR allocate 1 byte for each irq */
	if (threshold_prio > GICC_PMR_PRIO_MAX)
		UK_CRASH("Possible priority from 0 to 255, input: %d\n",
			threshold_prio);
	write_gicc32(GICC_PMR, threshold_prio);
}

/*
 * Acknowledging irq equals reading GICC_IAR also
 * get the intrrupt ID as the side effect.
 */
uint32_t gic_ack_irq(void)
{
	return read_gicc32(GICC_IAR);
}

/*
 * write to GICC_EOIR to inform cpu interface completation
 * of interrupt processing. If GICC_CTLR.EOImode sets to 1
 * this func just gets priority drop.
 */
void gic_eoi_irq(uint32_t irq)
{
	write_gicc32(GICC_EOIR, irq);
}

/* Functions of GIC Distributor */

/*
 * @sgintid denotes the sgi ID;
 * @targetfilter : this term is TargetListFilter
 * 0 denotes forwarding interrupt to cpu specified in the
 * target list; 1 denotes forwarding interrupt to cpu execpt the
 * processor that request the intrrupt; 2 denotes forwarding the
 * interrupt only to the cpu that requtest the interrupt.
 * @targetlist is bitmask, which bit 1 denotes forwarding to and only low 8
 * bit is in use.
 */
static void gic_sgi_gen(uint32_t sgintid, uint8_t targetfilter,
			uint8_t targetlist)
{
	uint32_t val;

	/* Only INTID 0-15 allocated to sgi */
	if (sgintid > GICD_SGI_MAX_INITID)
		UK_CRASH("Only INTID 0-15 allocated to sgi\n");

	/* Set SGI tagetfileter field */
	val = (targetfilter & GICD_SGI_FILTER_MASK) << GICD_SGI_FILTER_SHIFT;

	/* Set SGI targetlist field */
	val |= (targetlist & GICD_SGI_TARGET_MASK) << GICD_SGI_TARGET_SHIFT;

	/* Set SGI INITID field */
	val |= sgintid;

	/* Generate SGI */
	write_gicd32(GICD_SGIR, val);
}

/*
 * Forward the SIG to the CPU interfaces specified in the
 * targetlist. Targetlist is a 8-bit bitmap for 0~7 CPU.
 */
void gic_sgi_gen_to_list(uint32_t sgintid, uint8_t targetlist)
{
	gic_sgi_gen(sgintid, GICD_SGI_FILTER_TO_LIST, targetlist);
}

/*
 * Forward the SGI to all CPU interfaces except that of the
 * processor that requested the interrupt.
 */
void gic_sgi_gen_to_others(uint32_t sgintid)
{
	gic_sgi_gen(sgintid, GICD_SGI_FILTER_TO_OTHERS, 0);
}

/*
 * Forward the SGI only to the CPU interface of the processor
 * that requested the interrupt.
 */
void gic_sgi_gen_to_self(uint32_t sgintid)
{
	gic_sgi_gen(sgintid, GICD_SGI_FILTER_TO_SELF, 0);
}

/*
 * set target cpu for irq in distributor,
 * @target: bitmask value, bit 1 indicates target to
 * corresponding cpu interface
 */
void gic_set_irq_target(uint32_t irq, uint8_t target)
{
	write_gicd8(GICD_ITARGETSR(irq), target);
}

/* set priority for irq in distributor */
void gic_set_irq_prio(uint32_t irq, uint8_t priority)
{
	write_gicd8(GICD_IPRIORITYR(irq), priority);
}

/*
 * Enable an irq in distributor, each irq occupies one bit
 * to configure in corresponding registor
 */
void gic_enable_irq(uint32_t irq)
{
	write_gicd32(GICD_ISENABLER(irq),
		read_gicd32(GICD_ISENABLER(irq)) |
		UK_BIT(irq % GICD_I_PER_ISENABLERn));
}

/*
 * Disable an irq in distributor, one bit reserved for an irq
 * to configure in corresponding register
 */
void gic_disable_irq(uint32_t irq)
{
	write_gicd32(GICD_ICENABLER(irq),
		read_gicd32(GICD_ICENABLER(irq)) |
		UK_BIT(irq % GICD_I_PER_ICENABLERn));
}

/* Enable distributor */
static void gic_enable_dist(void)
{
	/* just set bit 0 to 1 to enable distributor */
	write_gicd32(GICD_CTLR, read_gicd32(GICD_CTLR) | GICD_CTLR_ENABLE);
}

/* disable distributor */
static void gic_disable_dist(void)
{
	/* just clear bit 0 to 0 to enable distributor */
	write_gicd32(GICD_CTLR, read_gicd32(GICD_CTLR) & (~GICD_CTLR_ENABLE));
}

/*
 * set pending state for an irq in distributor, one bit
 * reserved for an irq to configure in corresponding register
 */
void gic_set_irq_pending(uint32_t irq)
{
	write_gicd32(GICD_ISPENDR(irq),
		read_gicd32(GICD_ISPENDR(irq)) |
		UK_BIT(irq % GICD_I_PER_ISPENDRn));
}

/*
 * clear pending state for an irq in distributor, one bit
 * reserved for an irq to configure in corresponding register
 */
void gic_clear_irq_pending(uint32_t irq)
{
	write_gicd32(GICD_ICPENDR(irq),
		read_gicd32(GICD_ICPENDR(irq)) |
		UK_BIT(irq % GICD_I_PER_ICPENDRn));
}

/*
 * inspect that if an irq is in pending state, every bit
 * holds the value for the corresponding irq
 */
int gic_is_irq_pending(uint32_t irq)
{
	if (read_gicd32(GICD_ICPENDR(irq)) &
		UK_BIT(irq % GICD_I_PER_ICPENDRn))
		return 1;
	return 0;
}

/* set active state for an irq in distributor */
void gic_set_irq_active(uint32_t irq)
{
	write_gicd32(GICD_ISACTIVER(irq),
		read_gicd32(GICD_ISACTIVER(irq)) |
		UK_BIT(irq % GICD_I_PER_ISACTIVERn));
}

/* clear active state for an irq in distributor */
void gic_clear_irq_active(uint32_t irq)
{
	write_gicd32(GICD_ICACTIVER(irq),
		read_gicd32(GICD_ICACTIVER(irq)) |
		UK_BIT(irq % GICD_I_PER_ICACTIVERn));
}

/*
 * inspect that if an irq is in active state,
 * every bit holds the value for an irq
 */
int gic_is_irq_active(uint32_t irq)
{
	if (read_gicd32(GICD_ISACTIVER(irq)) &
		UK_BIT(irq % GICD_I_PER_ISACTIVERn))
		return 1;
	return 0;
}

/* Config intrrupt trigger type and polarity */
void gic_set_irq_type(uint32_t irq, int trigger, int polarity)
{
	uint32_t val, mask, oldmask;

	if ((trigger >= UK_IRQ_TRIGGER_MAX) ||
		(polarity >= UK_IRQ_POLARITY_MAX))
		return;

	val = read_gicd32(GICD_ICFGR(irq));
	mask = oldmask = (val >> ((irq % GICD_I_PER_ICFGRn) * 2)) &
			GICD_ICFGR_MASK;

	if (trigger == UK_IRQ_TRIGGER_LEVEL) {
		mask &= ~GICD_ICFGR_TRIG_MASK;
		mask |= GICD_ICFGR_TRIG_LVL;
	} else if (trigger == UK_IRQ_TRIGGER_EDGE) {
		mask &= ~GICD_ICFGR_TRIG_MASK;
		mask |= GICD_ICFGR_TRIG_EDGE;
	}

	if (polarity == UK_IRQ_POLARITY_LOW) {
		mask &= ~GICD_ICFGR_POL_MASK;
		mask |= GICD_ICFGR_POL_LOW;
	} else if (polarity == UK_IRQ_POLARITY_HIGH) {
		mask &= ~GICD_ICFGR_POL_MASK;
		mask |= GICD_ICFGR_POL_HIGH;
	}

	/* Check if nothing changed */
	if (mask == oldmask)
		return;

	/* Update new interrupt type */
	val &= (~(GICD_ICFGR_MASK << (irq % GICD_I_PER_ICFGRn) * 2));
	val |= (mask << (irq % GICD_I_PER_ICFGRn) * 2);
	write_gicd32(GICD_ICFGR(irq), val);
}

static void gic_init_dist(void)
{
	uint32_t val, cpuif_number, irq_number;
	uint32_t i;

	/* Turn down distributor */
	gic_disable_dist();

	/* Get GIC CPU interface */
	val = read_gicd32(GICD_TYPER);
	cpuif_number = GICD_TYPER_CPUI_NUM(val);
	if (cpuif_number > GIC_MAX_CPUIF)
		cpuif_number = GIC_MAX_CPUIF;
	uk_printd(DLVL_INFO, "GICv2 Max CPU interface:%d\n", cpuif_number);

	/* Get the maximum number of interrupts that the GIC supports */
	irq_number = GICD_TYPER_LINE_NUM(val);
	if (irq_number > GIC_MAX_IRQ)
		irq_number = GIC_MAX_IRQ;
	uk_printd(DLVL_INFO, "GICv2 Max interrupt lines:%d\n", irq_number);
	/*
	 * Set all SPI interrupts targets to all CPU.
	 */
	for (i = GIC_SPI_BASE; i < irq_number; i += GICD_I_PER_ITARGETSRn)
		write_gicd32(GICD_ITARGETSR(i), GICD_ITARGETSR_DEF);

	/*
	 * Set all SPI interrupts type to be polarity low level triggered
	 */
	for (i = GIC_SPI_BASE; i < irq_number; i += GICD_I_PER_ICFGRn)
		write_gicd32(GICD_ICFGR(i), GICD_ICFGR_DEF_TYPE);

	/*
	 * Set all interrupts priority to a default value.
	 */
	for (i = 0; i < irq_number; i += GICD_I_PER_IPRIORITYn)
		write_gicd32(GICD_IPRIORITYR(i), GICD_IPRIORITY_DEF);

	/*
	 * Deactivate and disable all SPIs.
	 */
	for (i = GIC_SPI_BASE; i < irq_number; i += GICD_I_PER_ICACTIVERn) {
		write_gicd32(GICD_ICACTIVER(i), GICD_DEF_ICACTIVERn);
		write_gicd32(GICD_ICENABLER(i), GICD_DEF_ICENABLERn);
	}

	/* turn on distributor */
	gic_enable_dist();
}

static void gic_init_cpuif(void)
{
	/* set priority mask to the lowest priority to let all irq visible to cpu interface */
	gic_set_threshold_priority(GICC_PMR_PRIO_MAX);

	/* enable cpu interface */
	gic_enable_cpuif();
}

int _dtb_init_gic(void *dtb)
{
	uint32_t idx;
	int fdt_gic, naddr, nsize, prop_len, prop_min_len;
	const uint64_t *regs;

	uk_printd(DLVL_INFO, "Probing GICv2...\n");
	/* Currently, we only support 1 GIC per system */
	for (idx = 0;
		idx < sizeof(gic_device_list) / sizeof(gic_device_list[0]);
		idx++) {
		fdt_gic = fdt_node_offset_by_compatible(dtb, -1,
				gic_device_list[idx]);
                if (fdt_gic >= 0)
                        break;
	}

	if (fdt_gic < 0)
		UK_CRASH("No valid GIC device find\n");

	naddr = fdt_address_cells(dtb, fdt_gic);
	if (naddr < 0 || naddr >= FDT_MAX_NCELLS)
		UK_CRASH("Could not find proper address cells!\n");

	nsize = fdt_size_cells(dtb, fdt_gic);
	if (nsize < 0 || nsize >= FDT_MAX_NCELLS)
		UK_CRASH("Could not find proper size cells!\n");

        regs = fdt_getprop(dtb, fdt_gic, "reg", &prop_len);

	/*
	 * The property must contain at least the start address and size
	 * of distributor and cpu interface
	 */
	prop_min_len = (int)sizeof(fdt32_t) * (naddr + nsize) * 2;
	if (regs == NULL || prop_len < prop_min_len)
		UK_CRASH("Bad 'reg' property: %p %d\n", regs, prop_len);

	/*
	 * From:
	 * https://git.kernel.org/pub/scm/linux/kernel/git/stable/linux.git/tree/Documentation/devicetree/bindings/interrupt-controller/arm,gic.txt
	 * We know that the first region is the GIC distributor register
	 * base and size. The 2nd region is the GIC cpu interface register
	 * base and size.
	 */
	gic_dist_addr = (void *)fdt64_to_cpu(regs[0]);
	gic_dist_size = fdt64_to_cpu(regs[1]);
	gic_cpuif_addr = (void *)fdt64_to_cpu(regs[2]);
	gic_cpuif_size = fdt64_to_cpu(regs[3]);

	uk_printd(DLVL_INFO, "Found GICv2 on:\n");
	uk_printd(DLVL_INFO, "\tCPU interface address: %p\n", gic_cpuif_addr);
	uk_printd(DLVL_INFO, "\tDistributor address: %p\n", gic_dist_addr);


	/* Initialize GICv2 distributor */
	gic_init_dist();

	/* Initialize GICv2 CPU interface */
	gic_init_cpuif();

	return 0;
}
