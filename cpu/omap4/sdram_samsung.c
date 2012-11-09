/*
 * (C) Copyright 2004-2009
 * Texas Instruments, <www.ti.com>
 * Aneesh V	<aneesh@ti.com>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */
/*
 * All OMAP boards of a given revision todate use the same memory
 * configuration. So keeping this here instead of keeping in board
 * directory. If a new board has different memory part/configuration
 * in future the weakly linked alias for ddr_init() can be over-ridden
 */
#include <common.h>
#include <asm/io.h>

/* Timing regs for Samsung K3PE0E000M */

const struct ddr_regs ddr_regs_samsung_2x4G_400_mhz = {
	.tim1		= 0x10eb0662,
	.tim2		= 0x20370dd2,
	.tim3		= 0x00b1c33f,
	.phy_ctrl_1	= 0x849FF408,
	.ref_ctrl	= 0x00000618,
	.config_init	= 0x80000eba,
	.config_final	= 0x80001aba,
	.zq_config	= 0xd00b3215,
	.mr1		= 0x83,
	.mr2		= 0x4
};

/*
 * Set Read Latency value to 0xB and DLL delay to 0x37
 * according to OMAP4470 LPDDR interface configuration
 * update for 466 MHz
 */
const struct ddr_regs ddr_regs_samsung_2x4G_466_mhz = {
#ifdef CORE_233MHZ
	.tim1		= 0x130D376B,
	.tim2		= 0x3041105A,
	.tim3		= 0x00D4E3CF,
	.ref_ctrl	= 0x00000719,
	.phy_ctrl_1	= 0x849FF37B,
	.config_final	= 0x80001eba,
	.mr2		= 0x5,
#else
	.tim1           = 0x130F376B,
	.tim2           = 0x3041105A,
	.tim3           = 0x00F543CF,
	.ref_ctrl	= 0x0000071B,
	.phy_ctrl_1	= 0x849FF408,
	.config_final	= 0x80001aba,
	.mr2		= 0x4,
#endif
	.config_init	= 0x80000eba,
	.zq_config	= 0xd00b3215,
	.mr1		= 0x83,
};

/* Timing regs for Samsung K3PE7E700M */

const struct ddr_regs ddr_regs_samsung4G_400_mhz = {
	/* tRRD changed from 10ns to 12.5ns because of the tFAW requirement*/
	.tim1		= 0x10eb0662,
	.tim2		= 0x20370dd2,
	.tim3		= 0x00b1c33f,
	.phy_ctrl_1	= 0x849FF408,
	.ref_ctrl	= 0x00000618,
	.config_init	= 0x80000eb2,
	.config_final	= 0x80001ab2,
	.zq_config	= 0x500b3215,
	.mr1		= 0x83,
	.mr2		= 0x4
};

const struct ddr_regs ddr_regs_samsung4G_466_mhz = {
#ifdef CORE_233MHZ
	.tim1		= 0x130D376B,
	.tim2		= 0x3041105A,
	.tim3		= 0x00D4E3CF,
	.ref_ctrl	= 0x00000719,
	.phy_ctrl_1	= 0x849FF37B,
	.config_final	= 0x80001eb2,
	.mr2		= 0x5,
#else
	.tim1           = 0x130F376B,
	.tim2           = 0x3041105A,
	.tim3           = 0x00F543CF,
	.ref_ctrl	= 0x0000071B,
	.phy_ctrl_1	= 0x849FF408,
	.config_final	= 0x80001ab2,
	.mr2		= 0x4,
#endif
	.config_init	= 0x80000eb2,
	.zq_config	= 0x500b3215,
	.mr1		= 0x83,
};

const struct ddr_regs ddr_regs_samsung2G_400_mhz = {
        /* tRRD changed from 10ns to 12.5ns because of the tFAW requirement*/
        .tim1           = 0x10eb0662,
        .tim2           = 0x20370dd2,
        .tim3           = 0x00b1c33f,
        .phy_ctrl_1     = 0x849FF408,
        .ref_ctrl       = 0x00000618,
        .config_init    = 0x80000eb1,
        .config_final   = 0x80001ab1,
        .zq_config      = 0x500b3215,
        .mr1            = 0x83,
        .mr2            = 0x4
};

/* ddr_init() - initializes ddr */
void __ddr_init_samsung(void)
{
	u32 rev, ddr_size;
	const struct ddr_regs *ddr_regs = 0;

	rev = omap_revision();
        ddr_size = ( get_hwid() & 0x18 ) >> 3;
        switch (ddr_size) {
		case    DDR_SIZE_512MB:
			ddr_regs = &ddr_regs_samsung2G_400_mhz;
                        break;
                case    DDR_SIZE_1GB:
#ifdef CORE_233MHZ
                        ddr_regs = &ddr_regs_samsung4G_466_mhz;
#else
                        ddr_regs = &ddr_regs_samsung4G_400_mhz;
#endif
			break;
		case	DDR_SIZE_2GB:
		default:
#ifdef CORE_233MHZ
			ddr_regs= &ddr_regs_samsung_2x4G_466_mhz;
#else
			ddr_regs= &ddr_regs_samsung_2x4G_400_mhz;
#endif
        }

	/* TRAP for catching accesses to the umapped memory */
	__raw_writel(0x80720100, DMM_BASE + DMM_LISA_MAP_0);

	__raw_writel(0x00000000, DMM_BASE + DMM_LISA_MAP_2);
	/* TRAP for catching accesses to the memory actually used by TILER */
	__raw_writel(0xFF020100, DMM_BASE + DMM_LISA_MAP_1);

	if (rev == OMAP4430_ES1_0 || ddr_size == DDR_SIZE_512MB)
		/* original DMM configuration
		 * - 512 MB, 128 byte interleaved, EMIF1&2, SDRC_ADDRSPC=0 */
		__raw_writel(0x80540300, DMM_BASE + DMM_LISA_MAP_3);
	else if (rev < OMAP4460_ES1_0)
		/* original DMM configuration
		 * - 1024 MB, 128 byte interleaved, EMIF1&2, SDRC_ADDRSPC=0 */
		__raw_writel(0x80640300, DMM_BASE + DMM_LISA_MAP_3);
	else {
                /* OMAP4460 and higher: original DMM configuration
                 * - 1024 MB, 128 byte interleaved, EMIF1&2, SDRC_ADDRSPC=0 */
                if (ddr_size == DDR_SIZE_2GB)
                        __raw_writel(0x80740300, DMM_BASE + DMM_LISA_MAP_3);
                else
                        __raw_writel(0x80640300, DMM_BASE + DMM_LISA_MAP_3);

                __raw_writel(0x80720100, MA_BASE + DMM_LISA_MAP_0);
                __raw_writel(0xFF020100, MA_BASE + DMM_LISA_MAP_1);
                __raw_writel(0x00000000, MA_BASE + DMM_LISA_MAP_2);
                if (ddr_size == DDR_SIZE_2GB)
                        __raw_writel(0x80740300, MA_BASE + DMM_LISA_MAP_3);
                else
                        __raw_writel(0x80640300, MA_BASE + DMM_LISA_MAP_3);
	}

	/* same memory part on both EMIFs */
	do_ddr_init(ddr_regs, ddr_regs);

	/* Pull Dn enabled for "Weak driver control" on LPDDR
	 * Interface.
	 */
	if (rev >= OMAP4460_ES1_0) {
		__raw_writel(0x9c9c9c9c, CONTROL_LPDDR2IO1_0);
		__raw_writel(0x9c9c9c9c, CONTROL_LPDDR2IO1_1);
		__raw_writel(0x9c989c00, CONTROL_LPDDR2IO1_2);
		__raw_writel(0xa0888c03, CONTROL_LPDDR2IO1_3);
		__raw_writel(0x9c9c9c9c, CONTROL_LPDDR2IO2_0);
		__raw_writel(0x9c9c9c9c, CONTROL_LPDDR2IO2_1);
		__raw_writel(0x9c989c00, CONTROL_LPDDR2IO2_2);
		__raw_writel(0xa0888c03, CONTROL_LPDDR2IO2_3);
	}

#ifdef CORE_233MHZ
	if (rev >= OMAP4470_ES1_0) {
		__raw_writel(0x9c3c9c00, CONTROL_LPDDR2IO1_2);
		__raw_writel(0x9c3c9c00, CONTROL_LPDDR2IO2_2);
	}
#endif

}

void ddr_init_samsung(void)
	__attribute__((weak, alias("__ddr_init_samsung")));
