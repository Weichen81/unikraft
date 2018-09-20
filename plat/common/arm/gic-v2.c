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

/* Max CPU interface for GICv2 */
#define GIC_MAX_CPUIF		8

/* SPI interrupt definitions */
#define GIC_SPI_TYPE		0
#define GIC_SPI_BASE		32

/* PPI interrupt definitions */
#define GIC_PPI_TYPE		1
#define GIC_PPI_BASE		16

/* SGI interrupt definitions */
#define GIC_SGI_TYPE		2
#define GIC_SGI_BASE		0

/* Max support interrupt number for GICv2 */
#define GIC_MAX_IRQ		__MAX_IRQ

static uint64_t gic_dist_addr, gic_cpuif_addr;
static uint64_t gic_dist_size, gic_cpuif_size;

#define GIC_DIST_REG(r)	((void *)(gic_dist_addr + (r)))
#define GIC_CPU_REG(r)	((void *)(gic_cpuif_addr + (r)))

static const char *gic_device_list[] = {
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
	write_gicc32(GICC_CTLR, GICC_CTLR_ENABLE);
}

/* Set priority threshold for processor */
static void gic_set_threshold_priority(uint32_t threshold_prio)
{
	/* GICC_PMR allocate 1 byte for each irq */
	UK_ASSERT(threshold_prio <= GICC_PMR_PRIO_MAX);
	write_gicc32(GICC_PMR, threshold_prio);
}

/*
 * Acknowledging irq equals reading GICC_IAR also
 * get the interrupt ID as the side effect.
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
	UK_ASSERT(sgintid <= GICD_SGI_MAX_INITID);

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
 * TODO: this will not work untill SMP is supported
 */
void gic_sgi_gen_to_list(uint32_t sgintid, uint8_t targetlist)
{
	gic_sgi_gen(sgintid, GICD_SGI_FILTER_TO_LIST, targetlist);
}

/*
 * Forward the SGI to all CPU interfaces except that of the
 * processor that requested the interrupt.
 * TODO: this will not work untill SMP is supported
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
		UK_BIT(irq % GICD_I_PER_ISENABLERn));
}

/*
 * Disable an irq in distributor, one bit reserved for an irq
 * to configure in corresponding register
 */
void gic_disable_irq(uint32_t irq)
{
	write_gicd32(GICD_ICENABLER(irq),
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

int gic_irq_translate(int type, int hw_irq)
{
	int irq;

        switch (type) {
	case GIC_SPI_TYPE:
		irq = hw_irq + GIC_SPI_BASE;
		if (irq >= GIC_SPI_BASE && irq < GIC_SPI_BASE)
			return irq;
		break;
	case GIC_PPI_TYPE:
		irq = hw_irq + GIC_PPI_BASE;
		if (irq >= GIC_PPI_BASE && irq < GIC_SPI_BASE)
			return irq;
		break;
	case GIC_SGI_TYPE:
		irq = hw_irq + GIC_SGI_BASE;
		if (irq >= GIC_SGI_BASE && irq < GIC_PPI_BASE)
			return irq;
		break;
	default:
		uk_pr_warn("Invalid IRQ type [%d]\n", type);
        }

        return -EINVAL;
}

void gic_handle_irq(void)
{
	uint32_t stat, irq;

	do {
		stat = gic_ack_irq();
		irq = stat & GICC_IAR_INTID_MASK;

		uk_printd(DLVL_CRIT, "Unikraft: EL1 IRQ#%d trap caught\n", irq);

		/*
		 * TODO: Hanle IPI&SGI interrupts here
		 */
		if (irq < GIC_MAX_IRQ) {
			isb();
			_ukplat_irq_handle((unsigned long)irq);
			gic_eoi_irq(stat);
			continue;
		}

		break;
	} while (1);
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
	uk_pr_info("GICv2 Max CPU interface:%d\n", cpuif_number);

	/* Get the maximum number of interrupts that the GIC supports */
	irq_number = GICD_TYPER_LINE_NUM(val);
	if (irq_number > GIC_MAX_IRQ)
		irq_number = GIC_MAX_IRQ;
	uk_pr_info("GICv2 Max interrupt lines:%d\n", irq_number);
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
	 * Set all SPI priority to a default value.
	 */
	for (i = GIC_SPI_BASE; i < irq_number; i += GICD_I_PER_IPRIORITYn)
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
	uint32_t i;
	/* set priority mask to the lowest priority to let all irq visible to cpu interface */
	gic_set_threshold_priority(GICC_PMR_PRIO_MAX);

	/* set PPI and SGI to polarity low level triggered */
	for (i = 0; i < GIC_SPI_BASE; i += GICD_I_PER_ICFGRn)
		write_gicd32(GICD_ICFGR(i), GICD_ICFGR_DEF_TYPE);

	/* set PPI and SGI to a default value */
	for (i = 0; i < GIC_SPI_BASE; i += GICD_I_PER_IPRIORITYn)
		write_gicd32(GICD_IPRIORITYR(i), GICD_IPRIORITY_DEF);

        /*
	 * Deactivate and disable all PPIs.
	 */
        for (i = 0; i < GICD_SGI_MAX_INITID; i += GICD_I_PER_ICACTIVERn) {
		write_gicd32(GICD_ICACTIVER(i), GICD_DEF_ICACTIVERn);
		write_gicd32(GICD_ICENABLER(i), GICD_DEF_PPI_ICENABLERn);
        }

	/* Deactivate and enable all SGIs */
	for (i = 0; i < GIC_PPI_BASE; i += GICD_I_PER_ICACTIVERn)
	{
		write_gicd32(GICD_ICACTIVER(i), GICD_DEF_ICACTIVERn);
		write_gicd32(GICD_ISENABLER(i), GICD_DEF_SGI_ISENABLERn);
	}

	/* enable cpu interface */
	gic_enable_cpuif();
}

int _dtb_init_gic(const void *fdt)
{
	int fdt_gic, ret;

	uk_pr_info("Probing GICv2...\n");

	/* Currently, we only support 1 GIC per system */
	fdt_gic = fdt_node_offset_by_compatible_list(fdt, -1,
				gic_device_list, sizeof(gic_device_list));
	if (fdt_gic < 0)
		UK_CRASH("Could not find GICv2 Interrupt Controller!\n");

	/* Get device address and size at regs region */
	ret = fdt_get_address(fdt, fdt_gic, 0,
			&gic_dist_addr, &gic_dist_size);
	if (ret < 0)
		UK_CRASH("Could not find GICv2 distributor region!\n");

	ret = fdt_get_address(fdt, fdt_gic, 1,
			&gic_cpuif_addr, &gic_cpuif_size);
	if (ret < 0)
		UK_CRASH("Could not find GICv2 cpuif region!\n");

	uk_pr_info("Found GICv2 on:\n");
	uk_pr_info("\tDistributor  : 0x%lx - 0x%lx\n",
		gic_dist_addr, gic_dist_addr + gic_dist_size - 1);
	uk_pr_info("\tCPU interface: 0x%lx - 0x%lx\n",
		gic_cpuif_addr, gic_cpuif_addr + gic_cpuif_size - 1);


	/* Initialize GICv2 distributor */
	gic_init_dist();

	/* Initialize GICv2 CPU interface */
	gic_init_cpuif();

	return 0;
}

int ukplat_get_irq_from_dtb(const void *fdt, int nodeoffset, int index)
{
	const fdt32_t *prop;
	int type, hwirq, size;

	prop = fdt_get_interrupt(fdt, nodeoffset, index, &size);
	if (!prop)
		return -EINVAL;

	type = fdt32_to_cpu(prop[0]);
	hwirq = fdt32_to_cpu(prop[1]);

	return gic_irq_translate(type , hwirq);
}
