/*
 * (C) Copyright 2004-2009
 * Texas Instruments, <www.ti.com>
 * Richard Woodruff <r-woodruff2@ti.com>
 *
 * Based on board/omap4430sdp.c
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
#include <common.h>
#include <asm/arch/cpu.h>
#include <asm/io.h>
#include <asm/arch/bits.h>
#include <asm/arch/mux.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch/sys_info.h>
#include <asm/arch/clocks.h>
#include <asm/arch/mem.h>
#include <i2c.h>
#include <asm/mach-types.h>
#if (CONFIG_COMMANDS & CFG_CMD_NAND) && defined(CFG_NAND_LEGACY)
#include <linux/mtd/nand_legacy.h>
#endif
#include <omap4_hs.h>
#define CONFIG_OMAP4_SDC		1
#define XLOADER_SV 0

/*****************************************
 * Routine: board_init
 * Description: Early hardware init.
 *****************************************/
int board_init(void)
{
	unsigned int sw_ver = XLOADER_SV;

	pmic_set_vpp();

	SEC_ENTRY_Std_Ppa_Call( PPA_SERV_HAL_BN_INIT,1,&sw_ver);

	return 0;
}


#if 1
#define M0_SAFE M0
#define M1_SAFE M1
#define M2_SAFE M2
#define M4_SAFE M4
#define M7_SAFE M7
#define M3_SAFE M3
#define M5_SAFE M5
#define M6_SAFE M6
#else
#define M0_SAFE M7
#define M1_SAFE M7
#define M2_SAFE M7
#define M4_SAFE M7
#define M7_SAFE M7
#define M3_SAFE M7
#define M5_SAFE M7
#define M6_SAFE M7
#endif

/*
 * IEN  - Input Enable
 * IDIS - Input Disable
 * PTD  - Pull type Down
 * PTU  - Pull type Up
 * DIS  - Pull type selection is inactive
 * EN   - Pull type selection is active
 * M0   - Mode 0
 * The commented string gives the final mux configuration for that pin
 */

#define MUX_DEFAULT_OMAP4() \
	MV(CP(GPMC_AD0) , ( PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M1))  /* sdmmc2_dat0 */ \
	MV(CP(GPMC_AD1) , ( PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M1))  /* sdmmc2_dat1 */ \
	MV(CP(GPMC_AD2) , ( PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M1))  /* sdmmc2_dat2 */ \
	MV(CP(GPMC_AD3) , ( PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M1))  /* sdmmc2_dat3 */ \
	MV(CP(GPMC_AD4) , ( PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M1))  /* sdmmc2_dat4 */ \
	MV(CP(GPMC_AD5) , ( PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M1))  /* sdmmc2_dat5 */ \
	MV(CP(GPMC_AD6) , ( PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M1))  /* sdmmc2_dat6 */ \
	MV(CP(GPMC_AD7) , ( PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M1))  /* sdmmc2_dat7 */ \
	MV(CP(GPMC_AD8) , ( IEN | M3))  				 /* gpio_32 - KEY-HOME */ \
	MV(CP(GPMC_AD9) , ( IEN | M3))  				 /* gpio_33 - HW-ID0 */ \
	MV(CP(GPMC_AD10) , ( IEN | M3))  				 /* gpio_34 - HW-ID1 */ \
	MV(CP(GPMC_AD11) , ( IEN | M3))  				 /* gpio_35 - HW-ID2 */ \
	MV(CP(GPMC_AD12) , ( M3))  					 /* gpio_36 - LCD-PWR-EN */ \
	MV(CP(GPMC_AD13) , ( IEN | M3))  				 /* gpio_37 - TP-nINT */ \
	MV(CP(GPMC_AD14) , ( M3))  					 /* gpio_38 - BL-PWR-nEN */ \
	MV(CP(GPMC_AD15) , ( OFF_EN | OFF_IN | M3))  			 /* gpio_39 - TP-RESET */ \
	MV(CP(GPMC_A16) , ( IEN | M3))  				 /* gpio_40 - HW-ID3 */ \
	MV(CP(GPMC_A17) , ( IEN | M3))  				 /* gpio_41 - HW-ID4 */ \
	MV(CP(GPMC_A18) , ( PTU | IEN | M3)) 				 /* gpio_42 - GG-BAT-LOW */ \
	MV(CP(GPMC_A19) , ( M7))					 /* Not Used - gpmc_a19 */ \
	MV(CP(GPMC_A20) , ( M7))					 /* Not Used - IC-CHG-LEVEL */ \
	MV(CP(GPMC_A21) , ( M7))  					 /* Not Used - gpio_45 */ \
	MV(CP(GPMC_A22) , ( M7))  		 			 /* Not Used - gpio_46 */ \
	MV(CP(GPMC_A23) , ( M7))  					 /* Not Used - gpio_47 */ \
	MV(CP(GPMC_A24) , ( M7))  					 /* Not Used - gpio_48 */ \
	MV(CP(GPMC_A25) , ( IEN | M3))  				 /* gpio_49 - HW-ID5 */ \
	MV(CP(GPMC_NCS0) , ( IEN | M3))  				 /* gpio_50 - HW-ID6 */ \
	MV(CP(GPMC_NCS1) , ( IEN | M3))  				 /* gpio_51 - HW-ID7 */ \
	MV(CP(GPMC_NCS2) , ( OFF_EN | OFF_OUT_PTD | M3))  		 /* gpio_52 - MMC2-RST */ \
	MV(CP(GPMC_NCS3) , ( M7))			  	 	 /* Not Used */ \
	MV(CP(GPMC_NWP) , ( M7))  					 /* Not Used */ \
	MV(CP(GPMC_CLK) , ( M7))				  	 /* Not Used */ \
	MV(CP(GPMC_NADV_ALE) , ( M7))  					 /* Not Used */ \
	MV(CP(GPMC_NOE) , ( PTU | IEN | OFF_EN | OFF_OUT_PTD | M1))  	 /* sdmmc2_clk */ \
	MV(CP(GPMC_NWE) , ( PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M1))  /* sdmmc2_cmd */ \
	MV(CP(GPMC_NBE0_CLE) , ( M7))  					 /* Not Used - gpio_59 */ \
	MV(CP(GPMC_NBE1) , ( OFF_EN | OFF_OUT_PTD | M3))  		 /* gpio_60 - HDMI_CT_CP_HPD */ \
	MV(CP(GPMC_WAIT0) , ( M7))  					 /* Not Used - gpio_61 */ \
	MV(CP(GPMC_WAIT1) , ( M7)) 					 /* Not Used - gpio_62 */ \
	MV(CP(GPMC_WAIT2) , ( IEN | M3))  				 /* gpio_100 - LINOUT_DTC */ \
	MV(CP(GPMC_NCS4) , ( M7))  				 	 /* Not Used - AUD-PWR-EN */ \
	MV(CP(GPMC_NCS5) , ( M7)) 					 /* Not Used - gpio_102 */ \
	MV(CP(GPMC_NCS6) , ( M7))  					 /* Not Used - gpio_103 */ \
	MV(CP(GPMC_NCS7) , ( M7))  				 	 /* Not Used */ \
	MV(CP(HDMI_HPD) , ( IEN | M0))  				 /* hdmi_hpd */ \
	MV(CP(HDMI_CEC) , ( M0))  					 /* hdmi_cec */ \
	MV(CP(HDMI_DDC_SCL) , (M7))	  				 /* hdmi_ddc_scl - Remuxed in Kernel */ \
	MV(CP(HDMI_DDC_SDA) , (M7))  					 /* hdmi_ddc_sda - Remuxed in Kernel */ \
	MV(CP(CSI21_DX0) , ( M7))  					 /* Not used - csi21_dx0 */ \
	MV(CP(CSI21_DY0) , ( M7))  					 /* Not used - csi21_dy0 */ \
	MV(CP(CSI21_DX1) , ( M7))  					 /* Not used - csi21_dx1 */ \
	MV(CP(CSI21_DY1) , ( M7))  					 /* Not used - csi21_dy1 */ \
	MV(CP(CSI21_DX2) , ( M7))  					 /* Not used - csi21_dx2 */ \
	MV(CP(CSI21_DY2) , ( M7))  					 /* Not used - csi21_dy2 */ \
	MV(CP(CSI21_DX3) , ( M7))  					 /* Not used - csi21_dx3 */ \
	MV(CP(CSI21_DY3) , ( M7))  					 /* Not used - csi21_dy3 */ \
	MV(CP(CSI21_DX4) , ( M7))  					 /* Not used - csi21_dx4 */ \
	MV(CP(CSI21_DY4) , ( M7))  					 /* Not used - csi21_dy4 */ \
	MV(CP(CSI22_DX0) , ( M7))  					 /* Not used - csi22_dx0 */ \
	MV(CP(CSI22_DY0) , ( M7))  					 /* Not used - csi22_dy0 */ \
	MV(CP(CSI22_DX1) , ( M7))  					 /* Not used - csi22_dx1 */ \
	MV(CP(CSI22_DY1) , ( M7))  					 /* Not used - csi22_dy1 */ \
	MV(CP(CAM_SHUTTER) , ( OFF_EN | OFF_OUT_PTD | M3))  	  	 /* gpio_81 - HDMI_LS_OE */ \
	MV(CP(CAM_STROBE) , ( M3))				  	 /* gpio_82 - CPU-DBG-LED */ \
	MV(CP(CAM_GLOBALRESET) , ( M3))  				 /* gpio_83 - BT-ENABLE */ \
	MV(CP(USBB1_ULPITLL_CLK) , ( M7)) 				 /* Not Used - hsi1_cawake */ \
	MV(CP(USBB1_ULPITLL_STP) , ( M3))				 /* gpio_85 - CHG-LEVEL */ \
	MV(CP(USBB1_ULPITLL_DIR) , ( M7))  				 /* Not Used - hsi1_caflag */ \
	MV(CP(USBB1_ULPITLL_NXT) , ( M7))  				 /* Not Used - hsi1_acready */ \
	MV(CP(USBB1_ULPITLL_DAT0) , ( M7))  				 /* Not Used - hsi1_acwake */ \
	MV(CP(USBB1_ULPITLL_DAT1) , ( M7))  				 /* Not Used - hsi1_acdata */ \
	MV(CP(USBB1_ULPITLL_DAT2) , ( M7))  				 /* Not Used - hsi1_acflag */ \
	MV(CP(USBB1_ULPITLL_DAT3) , ( M7))  				 /* Not Used - hsi1_caready */ \
	MV(CP(USBB1_ULPITLL_DAT4) , ( IEN | OFF_EN | OFF_PD | OFF_IN | M2))	/* abe_mcbsp3_dr */ \
	MV(CP(USBB1_ULPITLL_DAT5) , ( OFF_EN | OFF_OUT_PTD | M2))		/* abe_mcbsp3_dx */ \
	MV(CP(USBB1_ULPITLL_DAT6) , ( IEN | OFF_EN | OFF_PD | OFF_IN | M2))	/* abe_mcbsp3_clkx */ \
	MV(CP(USBB1_ULPITLL_DAT7) , ( IEN | OFF_EN | OFF_PD | OFF_IN | M2))	/* abe_mcbsp3_fsx */ \
	MV(CP(USBB1_HSIC_DATA) , ( M7))						/* Not Used - usbb1_hsic_data */ \
	MV(CP(USBB1_HSIC_STROBE) , ( M7))					/* Not Used - usbb1_hsic_strobe */ \
	MV(CP(USBC1_ICUSB_DP) , ( M7))  					/* Not Used - usbc1_icusb_dp */ \
	MV(CP(USBC1_ICUSB_DM) , ( M7))  					/* Not Used - usbc1_icusb_dm */ \
	MV(CP(SDMMC1_CLK) , ( PTU | OFF_EN | OFF_OUT_PTD | M0))			/* sdmmc1_clk */ \
	MV(CP(SDMMC1_CMD) , ( PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M0))	/* sdmmc1_cmd */ \
	MV(CP(SDMMC1_DAT0) , ( PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M0))	/* sdmmc1_dat0 */ \
	MV(CP(SDMMC1_DAT1) , ( PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M0))	/* sdmmc1_dat1 */ \
	MV(CP(SDMMC1_DAT2) , ( PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M0))	/* sdmmc1_dat2 */ \
	MV(CP(SDMMC1_DAT3) , ( PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M0))	/* sdmmc1_dat3 */ \
	MV(CP(SDMMC1_DAT4) , ( M7))						/* Not Used - sdmmc1_dat4 */ \
	MV(CP(SDMMC1_DAT5) , ( M7))						/* Not Used - sdmmc1_dat5 */ \
	MV(CP(SDMMC1_DAT6) , ( M7))						/* Not Used - sdmmc1_dat6 */ \
	MV(CP(SDMMC1_DAT7) , ( M7))						/* Not Used - sdmmc1_dat7 */ \
	MV(CP(ABE_MCBSP2_CLKX) , ( M7))						/* Not Used - gpio_110 */ \
	MV(CP(ABE_MCBSP2_DR) , ( M7))						/* Not Used - gpio_111 */ \
	MV(CP(ABE_MCBSP2_DX) , ( M3))						/* gpio_112 - AUD-PWRON-EN */ \
	MV(CP(ABE_MCBSP2_FSX) , ( M3))						/* gpio_113 - GG-CE */ \
	MV(CP(ABE_MCBSP1_CLKX) , ( M3))     					/* gpio_114 - WLAN-PWR-EN*/ \
	MV(CP(ABE_MCBSP1_DR) , ( M3))						/* gpio_115 - WLAN-EN */ \
	MV(CP(ABE_MCBSP1_DX) , ( PTU | IEN | OFF_EN | OFF_PD | M1))  		/* sdmmc3_dat2 - WLAN-SDIO-DAT2 */ \
	MV(CP(ABE_MCBSP1_FSX) , ( PTU | IEN | OFF_EN | OFF_PD | M1))      	/* sdmmc3_dat3 - WLAN-SDIO-DAT3 */ \
	MV(CP(ABE_PDM_UL_DATA) , ( PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M0))  /* abe_pdm_ul_data */ \
	MV(CP(ABE_PDM_DL_DATA) , ( PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M0))  /* abe_pdm_dl_data */ \
	MV(CP(ABE_PDM_FRAME) , ( PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M0))    /* abe_pdm_frame */ \
	MV(CP(ABE_PDM_LB_CLK) , ( PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M0))   /* abe_pdm_lb_clk */ \
	MV(CP(ABE_CLKS) , ( PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M0))		/* abe_clks */ \
	MV(CP(ABE_DMIC_CLK1) , ( IEN | M5))  					/* uart4_cts - BT-UART4-CTS */ \
	MV(CP(ABE_DMIC_DIN1) , ( M5))	 					/* uart4_rts - BT-UART4-RTS */ \
	MV(CP(ABE_DMIC_DIN2) , ( PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M0))	/* abe_dmic_din2 */ \
	MV(CP(ABE_DMIC_DIN3) , ( OFF_EN | OFF_OUT_PTD | M2))			/* abe_dmic_clk2 */ \
	MV(CP(UART2_CTS) , ( PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M1))	/* sdmmc3_clk  - WLAN-SDIO-CLK */ \
	MV(CP(UART2_RTS) , ( PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M1))  	/* sdmmc3_cmd  - WLAN-SDIO-CMD */ \
	MV(CP(UART2_RX) ,  ( PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) 	/* sdmmc3_dat0 - WLAN-SDIO-DAT0 */ \
	MV(CP(UART2_TX) ,  ( PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M1))  	/* sdmmc3_dat1 - WLAN-SDIO-DAT1 */ \
	MV(CP(HDQ_SIO) , ( M7))  						/* Not Used - gpio_127 */ \
	MV(CP(I2C1_SCL) , ( PTU | IEN | M0))  					/* i2c1_scl */ \
	MV(CP(I2C1_SDA) , ( PTU | IEN | M0)) 					/* i2c1_sda */ \
	MV(CP(I2C2_SCL) , ( IEN | M1))  					/* uart1_rx */ \
	MV(CP(I2C2_SDA) , ( IEN | M1))  					/* uart1_tx */ \
	MV(CP(I2C3_SCL) , ( PTU | IEN | M0))  					/* i2c3_scl */ \
	MV(CP(I2C3_SDA) , ( PTU | IEN | M0))  					/* i2c3_sda */ \
	MV(CP(I2C4_SCL) , ( M7))  						/* Not Used */ \
	MV(CP(I2C4_SDA) , ( M7))  						/* Not Used */ \
	MV(CP(MCSPI1_CLK) , ( OFF_EN | OFF_OUT_PTD | M0))			/* mcspi1_clk */ \
	MV(CP(MCSPI1_SOMI) , ( PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M0))	/* mcspi1_somi */ \
	MV(CP(MCSPI1_SIMO) , ( OFF_EN | OFF_PD | OFF_IN | M0))			/* mcspi1_simo */ \
	MV(CP(MCSPI1_CS0) , ( OFF_EN | OFF_PD | OFF_IN | M0))			/* mcspi1_cs0 */ \
	MV(CP(MCSPI1_CS1) , ( PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M1))	/* uart1_rx - OMAP-PEN-RX1 */ \
	MV(CP(MCSPI1_CS2) , ( M7))						/* Not Used - gpio_139 */ \
	MV(CP(MCSPI1_CS3) , ( M7))						/* Not Used - gpio_140 */ \
	MV(CP(UART3_CTS_RCTX) , ( OFF_EN | OFF_PD | OFF_IN | M1))		/* uart1_tx - OMAP-PEN-TX1 */ \
	MV(CP(UART3_RTS_SD) , ( IEN | M3)) 					/* gpio_142 - CHG-nINT */ \
	MV(CP(UART3_RX_IRRX) , ( PTU | IEN | M0))				/* uart3_rx - UART3_RX-DBG */ \
	MV(CP(UART3_TX_IRTX) , ( M0))  						/* uart3_tx - UART3_TX-DBG */ \
	MV(CP(SDMMC5_CLK) , ( M3))						/* gpio_145 - LCD_CM_EN */ \
	MV(CP(SDMMC5_CMD) , ( M7))						/* Not Used - gpio_146 */ \
	MV(CP(SDMMC5_DAT0) , ( M7))						/* Not Used - gpio_147 */ \
	MV(CP(SDMMC5_DAT1) , ( PTU | IEN | M3))					/* gpio_148 - WLAN-nINT */ \
	MV(CP(SDMMC5_DAT2) , ( M7))						/* Not used - LCD_CABC_EN */ \
	MV(CP(SDMMC5_DAT3) , ( M7))						/* Not used - sdmmc5_dat3 */ \
	MV(CP(MCSPI4_CLK) , ( M7))						/* Not Used - mcspi4_clk */ \
	MV(CP(MCSPI4_SIMO) , ( IEN | M3))  					/* gpio_152 - MOT-nINT */ \
	MV(CP(MCSPI4_SOMI) , ( M7))				  		/* Not Used - (LCD-CABC-nEN) Remuxed in kernel for EVT1a */ \
	MV(CP(MCSPI4_CS0) , ( M7))						/* Not Used - gpio_154 */ \
	MV(CP(UART4_RX) , ( PTU | IEN | M0))  					/* uart4_rx - BT-UART4-RX */ \
	MV(CP(UART4_TX) , ( M0))  						/* uart4_tx - BT-UART4-TX */ \
	MV(CP(USBB2_ULPITLL_CLK) , ( M7))					/* Not Used */ \
	MV(CP(USBB2_ULPITLL_STP) , ( M3))  					/* gpio_158 - CHG-nEN (Remuxed in kernel for EVT1a) */ \
	MV(CP(USBB2_ULPITLL_DIR) , ( IEN | M5))					/* dispc2_data22 */ \
	MV(CP(USBB2_ULPITLL_NXT) , ( IEN | M5))					/* dispc2_data21 */ \
	MV(CP(USBB2_ULPITLL_DAT0) , ( IEN | M5))				/* dispc2_data20 */ \
	MV(CP(USBB2_ULPITLL_DAT1) , ( IEN | M5))				/* dispc2_data19 */ \
	MV(CP(USBB2_ULPITLL_DAT2) , ( IEN | M5))				/* dispc2_data18 */ \
	MV(CP(USBB2_ULPITLL_DAT3) , ( IEN | M5))				/* dispc2_data15 */ \
	MV(CP(USBB2_ULPITLL_DAT4) , ( IEN | M5))				/* dispc2_data14 */ \
	MV(CP(USBB2_ULPITLL_DAT5) , ( IEN | M5))				/* dispc2_data13 */ \
	MV(CP(USBB2_ULPITLL_DAT6) , ( IEN | M5))				/* dispc2_data12 */ \
	MV(CP(USBB2_ULPITLL_DAT7) , ( IEN | M5))				/* dispc2_data11 */ \
	MV(CP(USBB2_HSIC_DATA) , ( M7))						/* Not Used - gpio_169 */ \
	MV(CP(USBB2_HSIC_STROBE) , ( M7))					/* Not Used - gpio_170 */ \
	MV(CP(KPD_COL3) , ( M7))			  			/* Not Used - gpio_171 */ \
	MV(CP(KPD_COL4) , ( M7))			  			/* Not Used */ \
	MV(CP(KPD_COL5) , ( OFF_EN | OFF_PD | OFF_IN | M3))			/* gpio_173 - OMAP-PEN-nRESET */ \
	MV(CP(KPD_COL0) , ( M7))  						/* kpd_col0 - KEY-UP-DWN-COL0 (Remuxed in Kernel) */ \
	MV(CP(KPD_COL1) , ( M7))			  			/* Not Used - gpio_0 */ \
	MV(CP(KPD_COL2) , ( M3))						/* gpio_1 - PEN-PWR-EN */ \
	MV(CP(KPD_ROW3) , ( IEN | OFF_EN | OFF_PD | OFF_IN | M3))		/* gpio_175 - OMAP_PEN-nINT */ \
	MV(CP(KPD_ROW4) , ( IEN | M3))						/* gpio_176 - GG-SOC-INT */ \
	MV(CP(KPD_ROW5) , ( M7))						/* Not Used - gpio_177 */ \
	MV(CP(KPD_ROW0) , ( M7))						/* kpd_row0 - KEY-VOL-UP-ROW0 (Remuxed in Kernel) */ \
	MV(CP(KPD_ROW1) , ( M7))						/* kpd_row1 - KEY-VOL-DWN-ROW1 (Remuxed in Kernel) */ \
	MV(CP(KPD_ROW2) , ( OFF_EN | OFF_PD | OFF_IN | M3))			/* gpio_3 - OMAP-PEN-FW-update */ \
	MV(CP(USBA0_OTG_CE) , ( M0))						/* usba0_otg_ce */ \
	MV(CP(USBA0_OTG_DP) , ( IEN | OFF_EN | OFF_PD | OFF_IN | M0))  		/* usba0_otg_dp - USB-OTG-DP */ \
	MV(CP(USBA0_OTG_DM) , ( IEN | OFF_EN | OFF_PD | OFF_IN | M0))  		/* usba0_otg_dm - USB-OTG-DM */ \
	MV(CP(FREF_CLK1_OUT) , ( M0))  						/* fref_clk1_out - AUD-CLK-19M2-OUT */ \
	MV(CP(FREF_CLK2_OUT) , ( OFF_EN | OFF_PD | OFF_IN | M3))		/* gpio_182 - 30pin-HDMI-UART1-SEL (EVT1a) */ \
	MV(CP(SYS_NIRQ1) , ( PTU | IEN | M0))  					/* sys_nirq1 - PMIC-nINT */ \
	MV(CP(SYS_NIRQ2) , ( PTU | IEN | M0))					/* sys_nirq2 - AUD-CODEC-nINT */ \
	MV(CP(SYS_BOOT0) , ( IEN | M0))	  					/* SYSBOOT0 */ \
	MV(CP(SYS_BOOT1) , ( IEN | M0))						/* SYSBOOT1 */ \
	MV(CP(SYS_BOOT2) , ( IEN | M0))						/* SYSBOOT2 */ \
	MV(CP(SYS_BOOT3) , ( IEN | M0))						/* SYSBOOT3 */ \
	MV(CP(SYS_BOOT4) , ( IEN | M0))						/* SYSBOOT4 */ \
	MV(CP(SYS_BOOT5) , ( IEN | M0))						/* SYSBOOT5 */ \
	MV(CP(DPM_EMU0) , ( IEN | M0))  					/* dpm_emu0 */ \
	MV(CP(DPM_EMU1) , ( IEN | M0))  					/* dpm_emu1 */ \
	MV(CP(DPM_EMU2) , ( M7))						/* Not Used - dpm_emu2 */ \
	MV(CP(DPM_EMU3) , ( IEN | M5))  					/* dispc2_data10 */ \
	MV(CP(DPM_EMU4) , ( IEN | M5))  					/* dispc2_data9 */ \
	MV(CP(DPM_EMU5) , ( IEN | M5))  					/* dispc2_data16 */ \
	MV(CP(DPM_EMU6) , ( IEN | M5))  					/* dispc2_data17 */ \
	MV(CP(DPM_EMU7) , ( IEN | M5))  					/* dispc2_hsync */ \
	MV(CP(DPM_EMU8) , ( IEN | M5))  					/* dispc2_pclk */ \
	MV(CP(DPM_EMU9) , ( IEN | M5)) 						/* dispc2_vsync */ \
	MV(CP(DPM_EMU10) , ( IEN | M5))  					/* dispc2_de */ \
	MV(CP(DPM_EMU11) , ( IEN | M5))  					/* dispc2_data8 */ \
	MV(CP(DPM_EMU12) , ( IEN | M5))  					/* dispc2_data7 */ \
	MV(CP(DPM_EMU13) , ( IEN | M5))  					/* dispc2_data6 */ \
	MV(CP(DPM_EMU14) , ( IEN | M5))  					/* dispc2_data5 */ \
	MV(CP(DPM_EMU15) , ( IEN | M5))  					/* dispc2_data4 */ \
	MV(CP(DPM_EMU16) , ( M3))	  					/* gpio_27 - LCD_CABC_EN (Remuxed in kernel for EVT1a) */ \
	MV(CP(DPM_EMU17) , ( IEN | M5))  					/* dispc2_data2 */ \
	MV(CP(DPM_EMU18) , ( IEN | M5))  					/* dispc2_data1 */ \
	MV(CP(DPM_EMU19) , ( IEN | M5))  					/* dispc2_data0 */ \
	MV1(WK(PAD0_SIM_IO) , ( M7))						/* Not Used - sim_io */ \
	MV1(WK(PAD1_SIM_CLK) , ( M7))						/* Not Used - sim_clk */ \
	MV1(WK(PAD0_SIM_RESET) , ( M7))						/* Not Used - sim_reset */ \
	MV1(WK(PAD1_SIM_CD) , ( M7))						/* Not Used - sim_cd*/ \
	MV1(WK(PAD0_SIM_PWRCTRL) , ( PTD | IEN | M3))				/* gpio_wk4 - EXT-FET-EN (EVT1a) */ \
	MV1(WK(PAD1_SR_SCL) , ( PTU | IEN | M0))  				/* sr_scl */ \
	MV1(WK(PAD0_SR_SDA) , ( PTU | IEN | M0))  				/* sr_sda */ \
	MV1(WK(PAD1_FREF_XTAL_IN) , ( M7))  					/* Not Used */ \
	MV1(WK(PAD0_FREF_SLICER_IN) , ( M0))					/* fref_slicer_in - SYS-CLK-38M4-IN */ \
	MV1(WK(PAD1_FREF_CLK_IOREQ) , ( M0))					/* fref_clk_ioreq - TCXO-38M4-EN */ \
	MV1(WK(PAD0_FREF_CLK0_OUT) , ( M2))  					/* sys_drm_msecure - MSECURE */ \
	MV1(WK(PAD1_FREF_CLK3_REQ) , ( M7))					/* Not Used */ \
	MV1(WK(PAD0_FREF_CLK3_OUT), ( PTU | IEN | M3 ))				/* gpio_wk31 - HAL_nDETECT */ \
	MV1(WK(PAD1_FREF_CLK4_REQ) , ( M7))					/* Not Used */ \
	MV1(WK(PAD0_FREF_CLK4_OUT) , ( M7))					/* Not Used */ \
	MV1(WK(PAD1_SYS_32K) , ( IEN | M0))					/* sys_32k */ \
	MV1(WK(PAD0_SYS_NRESPWRON) , ( IEN | M0))				/* sys_nrespwron */ \
	MV1(WK(PAD1_SYS_NRESWARM) , ( M0))  					/* sys_nreswarm */ \
	MV1(WK(PAD0_SYS_PWR_REQ) , ( M0))					/* sys_pwr_req */ \
	MV1(WK(PAD1_SYS_PWRON_RESET) , ( PTD | IEN | M3))			/* gpio_wk29 - SYS-PWR-ON-KEY (EVT1a) */ \
	MV1(WK(PAD0_SYS_BOOT6) , ( IEN | M0))  					/* SYSBOOT6 */ \
	MV1(WK(PAD1_SYS_BOOT7) , ( IEN | M0))  					/* SYSBOOT7 */

/**********************************************************
 * Routine: set_muxconf_regs
 * Description: Setting up the configuration Mux registers
 *              specific to the hardware. Many pins need
 *              to be moved from protect to primary mode.
 *********************************************************/
void set_muxconf_regs(void)
{
	MUX_DEFAULT_OMAP4();

	/*
	 * Changes for OMAP4460:
	 * gpio_wk7 is used for TPS controlling
	 */
	if (omap_revision() >= OMAP4460_ES1_0)
		writew(M3, OMAP44XX_CTRL_PADCONF_WKUP_BASE + CONTROL_WKUP_PAD1_FREF_CLK4_REQ);

	return;
}

/* optionally do something like blinking LED */
void board_hang (void)
{ while (0) {};}
