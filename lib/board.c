/*
 * Copyright (C) 2005 Texas Instruments.
 *
 * (C) Copyright 2004
 * Jian Zhang, Texas Instruments, jzhang@ti.com.
 *
 * (C) Copyright 2002
 * Wolfgang Denk, DENX Software Engineering, wd@denx.de.
 *
 * (C) Copyright 2002
 * Sysgo Real-Time Solutions, GmbH <www.elinos.com>
 * Marius Groeger <mgroeger@sysgo.de>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
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
#include <part.h>
#include <fat.h>
#include <mmc.h>
#include <version.h>
#include <asm/io.h>
#include <asm/arch/bits.h>
#include <asm/arch/clocks.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch-omap4/mux.h>
#include <i2c.h>
#include "omap4_hs.h"

typedef struct {
	u32 image;
	u32  * data;
	u32 * res;
	u32 val;
} image_type;


#ifdef CFG_PRINTF
int print_info(void)
{
	printf ("\n\n"X_LOADER_VERSION" ("
		__DATE__ " - " __TIME__ ")\n");
	return 0;
}
#endif
typedef int (init_fnc_t) (void);

init_fnc_t *init_sequence[] = {
	cpu_init,		/* basic cpu dependent setup */
	serial_init,		/* serial communication setup */
	board_init,		/* basic board dependent setup */
#ifdef CFG_PRINTF
	print_info,
#endif
   	//nand_init,		/* board specific nand init */
	NULL,
};

#ifdef CONFIG_USBOOT_MEMTEST
int mem_test(void);
#endif

#ifdef CFG_CMD_FAT
extern char * strcpy(char * dest,const char *src);
#else
char * strcpy(char * dest,const char *src)
{
	 char *tmp = dest;

	 while ((*dest++ = *src++) != '\0')
	         /* nothing */;
	 return tmp;
}
#endif

#ifdef CFG_CMD_MMC
extern block_dev_desc_t *mmc_get_dev(int dev);
int mmc_read_bootloader(int dev)
{
	unsigned char ret = 0;
	unsigned long offset = ( CFG_LOADADDR - 0x120 );

	ret = mmc_init(dev);
	if (ret != 0){
		printf("\n MMC init failed \n");
		return -1;
	}

#ifdef CFG_CMD_FAT
	long size;
	block_dev_desc_t *dev_desc = NULL;

	if (fat_boot()) {
		dev_desc = mmc_get_dev(dev);
		fat_register_device(dev_desc, 1);
		size = file_fat_read("u-boot.bin", (unsigned char *)offset, 0);
		if (size == -1)
			return -1;
	} else {
		/* FIXME: OMAP4 specific */
		 mmc_read(dev, 0x200, (unsigned char *)( CFG_LOADADDR - 0x120 ),
							0x00060000);
	}
#endif
	return 0;
}
#endif

/*
 * OMAP On-die temperature sensor check.
 * If the current temperature value is
 * greater than T_SHUT_HOT stop boot
 */

void omap_temp_sensor_check(void)
{
	u32 temp;

	/* Set the counter to 1 ms */
	sr32(CORE_BANDGAP_COUNTER, BGAP_COUNTER_START_BIT,
			BGAP_COUNTER_NUM_BITS, BGAP_COUNTER_VALUE);

	/* Enable continuous mode. */
	sr32(CORE_BANDGAP_CTRL, BGAP_SINGLE_MODE_START_BIT,
			BGAP_SINGLE_MODE_NUM_BITS, BGAP_CONTINUOUS_MODE);

	/* Wait till the first conversion is done wait for at least 1ms */
	spin_delay(20000);

	/* Read the temperature adc_value */
	temp = readl(CORE_TEMP_SENSOR);
	temp = temp & BGAP_TEMP_SENSOR_DTEMP_MASK;

	/* If the samples are untrimmed divide by 1.2 */
	if (readl(STD_FUSE_OPP_BGAP) == 0)
		temp = temp * 5 / 6;

	/*
	 * Compare with TSHUT high temperature. If high ask the
	 * user to shut down and restart after sometime else
	 * Disable continuous mode.
	 */
	if (temp < TSHUT_HIGH_ADC_CODE) {
		/* Disable contiuous mode */
		sr32(CORE_BANDGAP_CTRL, BGAP_SINGLE_MODE_START_BIT,
			BGAP_SINGLE_MODE_NUM_BITS, ~BGAP_CONTINUOUS_MODE);
	} else {
		printf("OMAP chip temperature is too high!!!\n");
		printf("Please power off and try booting after sometime\n");

		/* Bypass MPU, CORE, IVA, PER, ABE, USB DPLLs */
		sr32(CM_CLKMODE_DPLL_MPU, 0, 3, PLL_FAST_RELOCK_BYPASS);
		wait_on_value(BIT0, 0, CM_IDLEST_DPLL_MPU, LDELAY);

		sr32(CM_CLKMODE_DPLL_CORE, 0, 3, PLL_FAST_RELOCK_BYPASS);
		wait_on_value(BIT0, 0, CM_IDLEST_DPLL_CORE, LDELAY);

		sr32(CM_CLKMODE_DPLL_IVA, 0, 3, PLL_FAST_RELOCK_BYPASS);
		wait_on_value(BIT0, 0, CM_IDLEST_DPLL_IVA, LDELAY);

		sr32(CM_CLKMODE_DPLL_PER, 0, 3, PLL_FAST_RELOCK_BYPASS);
		wait_on_value(BIT0, 0, CM_IDLEST_DPLL_PER, LDELAY);

		sr32(CM_CLKMODE_DPLL_ABE, 0, 3, PLL_FAST_RELOCK_BYPASS);
		wait_on_value(BIT0, 0, CM_IDLEST_DPLL_ABE, LDELAY);

		sr32(CM_CLKMODE_DPLL_USB, 0, 3, PLL_FAST_RELOCK_BYPASS);
		wait_on_value(BIT0, 0, CM_IDLEST_DPLL_USB, LDELAY);

		while (1);
	}
}


extern int do_load_serial_bin(ulong offset, int baudrate);

#define __raw_readl(a)	(*(volatile unsigned int *)(a))

/*************************************************************/
/* Check if button power on is still pressed after defined   */
/* duration. If not, power off the system.                   */
/*************************************************************/
#define OMAP4_32KSYNCNT_CR	0x4A304010
#define PWRON_CHECK_DURATION	32768 /* 32768 = 1s */
#define TWL6030_PMC_ID		0x48
#define STS_HW_CONDITIONS	0x21
#define STS_PWRON		(1 << 0)
#define STS_PLUG_DET		(1 << 3)
#define PHOENIX_START_CONDITION	0x1F
#define STRT_ON_PWRON		(1 << 0)
#define STRT_ON_RPWRON		(1 << 1)
#define STRT_ON_USB_ID		(1 << 2)
#define STRT_ON_PLUG_DET	(1 << 3)
#define STRT_ON_RTC		(1 << 4)
#define FIRST_BAT_INS		(1 << 5)
#define RESTART_BB		(1 << 6)
#define DISABLED_STRT_EVENTS	(STRT_ON_RTC)
#define PHOENIX_DEV_ON		0x25
#define APP_DEV_OFF		(1 << 0)
#define CON_DEV_OFF		(1 << 1)
#define MOD_DEV_OFF		(1 << 2)
#define SWITCH_OFF		(APP_DEV_OFF | CON_DEV_OFF | MOD_DEV_OFF)

int select_bus(int bus, int speed);

void pwron_check_switch_off(void)
{
	u32 t32k_t1, t32k_t2;
	u8 val;

	select_bus(CFG_I2C_BUS, CFG_I2C_SPEED);

	/* if start is on disabled start event, switch off the board */
	i2c_read(TWL6030_PMC_ID, PHOENIX_START_CONDITION, 1, &val, 1);
	if (val & DISABLED_STRT_EVENTS) {
		printf("Start on disabled event.\n");
		goto switch_off;
	}

	/* if start is not on PWRON, skip the power button check */
	if (!(val & STRT_ON_PWRON)) {
		printf("Start not on PWRON, skiping power button check.\n");
		goto clean_and_exit;
	}

	/* if USB or charger is detected, skip the check so the user
	 * doesn't need to keep the button pressed for 5sec
	 */
	i2c_read(TWL6030_PMC_ID, STS_HW_CONDITIONS, 1, &val, 1);
	if (val & STS_PLUG_DET) {
		printf("Plug detected, skiping power button check.\n");
		goto clean_and_exit;
	}

	printf("Checking power button state... ");

	t32k_t1 = t32k_t2 = *((volatile u32 *)OMAP4_32KSYNCNT_CR);
	do {
		if ((t32k_t2 - t32k_t1) % 328 == 0) {
			/* Check poweron button status every 10ms */
			i2c_read(TWL6030_PMC_ID, STS_HW_CONDITIONS, 1, &val, 1);
			if (val & STS_PWRON) {
				printf("RELEASED. NOK.\n");
				goto switch_off;
			}
		}
		t32k_t2 = *((volatile u32 *)OMAP4_32KSYNCNT_CR);
	} while (t32k_t2 - t32k_t1 < PWRON_CHECK_DURATION);

	printf("PRESSED. OK\n");
clean_and_exit:
	val = 0x00;
	i2c_write(TWL6030_PMC_ID, PHOENIX_START_CONDITION, 1, &val, 1);
	return;

switch_off:
	printf("Powering off!\n");
	val = SWITCH_OFF;
	i2c_write(TWL6030_PMC_ID, PHOENIX_DEV_ON, 1, &val, 1);
	/* we should never get here */
	hang();
}

#ifdef CONFIG_USBBOOT
extern void enable_irqs(void);
extern void disable_irqs(void);
extern struct usb usbdev;
int usb_read_bootloader(int *_len) {
	long len = 0;
	int n = 0;
	unsigned long offset = (CFG_LOADADDR -0x120);
	static unsigned MSG = 0xaabbccdd;

	/*Enable USB mux pin cfg  enable USB clock */	
	enable_irqs();
	if (usb_open(&usbdev))
		return -1;

	usb_queue_read(&usbdev, &len, 4);
	printf("USB write MSG flag\n");
	usb_write(&usbdev, &MSG, 4);
	n = usb_wait_read(&usbdev);
	if (n)
		return -1;
	printf("USB read to %x len=%d\n", offset, len);
	if (usb_read(&usbdev, (void*) offset, len))
		return -1;

	/* read the img file */
	usb_queue_read(&usbdev, &len, 4);
	n = usb_wait_read(&usbdev);
	if (n)
		return -1;

	/* read the boot.img for booti command */
	printf("USB read2 to %x len=%d\n", 0x81000000, len);
	if (usb_read(&usbdev, (void*) 0x81000000, len))
		return -1;

	usb_close(&usbdev);
	disable_irqs();
	*_len = len;
	return 0;
}
#endif

#ifdef CONFIG_USBBOOT_ERASER
void mlo_erase()
{
	static unsigned MSG = 0xaabbccdd;
	int ret = 0;
	u8 val;
	/*Enable USB mux pin cfg  enable USB clock */	
	enable_irqs();
	if (usb_open(&usbdev))
		return -1;

	ret = mmc_init(1);
	if (ret != 0){
		printf("\n MMC init failed \n");
		goto error;
	}

	ret = mmc_erase(1, (1024*128/MMC_BLOCK_SIZE), 1024*128);

	usb_write(&usbdev, &MSG, 4);
	/*power off  PMIC */
	printf("erase ret %d Powering off!\n", ret);
	select_bus(CFG_I2C_BUS, CFG_I2C_SPEED);
	val = SWITCH_OFF;
	i2c_write(TWL6030_PMC_ID, PHOENIX_DEV_ON, 1, &val, 1);
	/* we should never get here */
error:
	hang();
}
#endif

void start_armboot (void)
{
  	init_fnc_t **init_fnc_ptr;
	uchar *buf;
	char boot_dev_name[8];
	u32 si_type, omap4_rev;
	int len = 0;
	u8 val = SWITCH_OFF;
	image_type image;
 
   	for (init_fnc_ptr = init_sequence; *init_fnc_ptr; ++init_fnc_ptr) {
		if ((*init_fnc_ptr)() != 0) {
			hang ();
		}
	}

	image.image = 2;
        image.val =99;

	omap4_rev = omap_revision();
	if (omap4_rev >= OMAP4460_ES1_0) {
		omap_temp_sensor_check();
		if (omap4_rev == OMAP4470_ES1_0) {
			writel(((TSHUT_HIGH_ADC_CODE << 16) |
			TSHUT_COLD_ADC_CODE), CORE_TSHUT_THRESHOLD);
			MV1(WK(CONTROL_SPARE_RW) , (M1));
		}
		si_type = omap4_silicon_type();
		if (si_type == PROD_ID_1_SILICON_TYPE_HIGH_PERF)
			printf("OMAP4470: 1.5 GHz capable SOM\n");
		else if (si_type == PROD_ID_1_SILICON_TYPE_STD_PERF)
			printf("OMAP4470: 1.3 GHz capable SOM\n");
	}
#ifdef CONFIG_USBBOOT_ERASER
	/* Erase mlo and poweroff */
	mlo_erase();
#else

#ifdef CONFIG_USBBOOT
	/*usb boot does not check power button */
	printf("boot_device=0x%x boot_mode=0x%x\n", get_boot_device(), get_boot_mode());
	printf("id_code=%08x\n", readl(CONTROL_ID_CODE));
#endif

#ifdef CONFIG_USBOOT_MEMTEST
	/* the device will power off after the test */
	mem_test();
	/*power off  PMIC */
	printf("Memtest done, powering off!\n");
	select_bus(CFG_I2C_BUS, CFG_I2C_SPEED);
	val = SWITCH_OFF;
	i2c_write(TWL6030_PMC_ID, PHOENIX_DEV_ON, 1, &val, 1);
	/* we should never get here */
	hang();
#endif

#ifdef START_LOADB_DOWNLOAD
	strcpy(boot_dev_name, "UART");
	do_load_serial_bin (CFG_LOADADDR, 115200);
#else
	buf = (uchar *) (CFG_LOADADDR - 0x120);
	image.data = (uchar *) (CFG_LOADADDR - 0x120);

	switch (get_boot_device()) {
#ifdef CONFIG_USBBOOT
	case 0x45:
		printf("boot_dev=USB\n");
		strcpy(boot_dev_name, "USB");
		/* read data from usb and write to sdram */
		if (usb_read_bootloader(&len) != 0) {
			hang();
		}
		printf("usb read len=%d\n", len);
		break;
#else
	case 0x03:
		strcpy(boot_dev_name, "ONENAND");
#if defined(CFG_ONENAND)
		for (i = ONENAND_START_BLOCK; i < ONENAND_END_BLOCK; i++) {
			if (!onenand_read_block(buf, i))
				buf += ONENAND_BLOCK_SIZE;
			else
				goto error;
		}
#endif
		break;
	case 0x02:
	default:
		strcpy(boot_dev_name, "NAND");
#if defined(CFG_NAND)
		for (i = NAND_UBOOT_START; i < NAND_UBOOT_END;
				i+= NAND_BLOCK_SIZE) {
			if (!nand_read_block(buf, i))
				buf += NAND_BLOCK_SIZE; /* advance buf ptr */
		}
#endif
		break;
	case 0x05:
		strcpy(boot_dev_name, "MMC/SD1");
#if defined(CONFIG_MMC)
		if (mmc_read_bootloader(0) != 0)
			goto error;
#endif
		break;
	case 0x06:
		strcpy(boot_dev_name, "EMMC");
#if defined(CONFIG_MMC)
		if (mmc_read_bootloader(1) != 0)
			goto error;
#endif
		break;
#endif	/* CONFIG_USBBOOT */
	};
#endif
	SEC_ENTRY_Std_Ppa_Call ( PPA_SERV_HAL_BN_CHK , 1 , &image );
 
	if ( image.val == 0 ) {
		/* go run U-Boot and never return */
		printf("Starting OS Bootloader from %s ...\n", boot_dev_name);
		((init_fnc_t *)CFG_LOADADDR)();
	}

	/* should never come here */
#if defined(CFG_ONENAND) || defined(CONFIG_MMC)
error:
#endif
	printf("Could not read bootloader!\n");
	hang();
#endif   /* CONFIG_USBBOOT_ERASER */	
}

void hang (void)
{
	/* call board specific hang function */
	board_hang();
	
	/* if board_hang() returns, hange here */
	printf("X-Loader hangs\n");
	for (;;);
}

#ifdef CONFIG_USBOOT_MEMTEST
#define KILO_BYTE 1024
#define MEGA_BYTE (KILO_BYTE*1024)
#define BOARD_RAM_SIZE_IN_BYTES (MEGA_BYTE * 1024)
#define BOARD_RAM_SIZE_IN_WORDS (((BOARD_RAM_SIZE_IN_BYTES - 1)/4) + 1)
#define SDRAM_ADDR 0x80000000
//for modulus 20 memory test
#define SPINSZ          (0x8000000 * 2) /* 1GB */
#define MOD_SZ          20

#define MEM_TEST_SUCCESS 0
#define MEM_TEST_FAIL (-1)



void set_all_memory(unsigned int value)
{
	int i;
	unsigned int *ptr = (unsigned int *)SDRAM_ADDR;
	printf("Setting Memory [0x%08x,0x%08x] to: 0x%08x\n",SDRAM_ADDR,SDRAM_ADDR + BOARD_RAM_SIZE_IN_BYTES - 1, value);
	for(i = 0 ; i < BOARD_RAM_SIZE_IN_WORDS ; i++)
	{
		*ptr = value;
		ptr++;
	}
}

int check_all_memory(unsigned int value)
{
	int i;
	unsigned int *ptr = (unsigned int *)SDRAM_ADDR;
	printf("Check Memory for: 0x%08x\n", value);
	for(i = 0 ; i < BOARD_RAM_SIZE_IN_WORDS ; i++)
	{
		if(*ptr != value)
		{
			printf("mem_test: address 0x%08x - expected 0x%08x - actual 0x%08x\n", ptr, value, *ptr);
			return MEM_TEST_FAIL;
		}
		ptr++;
	}
	return MEM_TEST_SUCCESS;
}

int modtst(int offset, int iter, ulong p1, ulong p2);

int mem_test(void)
{
	unsigned int p1,p2;
	int i;
#ifdef INSERT_MEM_ERROR
	unsigned int *ptr = SDRAM_ADDR + 0x10;
#endif

	set_all_memory(0x00000000);

#ifdef INSERT_MEM_ERROR
	*ptr = 0xdeadbeef;
#endif

	i = check_all_memory(0x00000000);

	if(MEM_TEST_SUCCESS != i)
	{
		return i;
	}

	set_all_memory(0xFFFFFFFF);
	i = check_all_memory(0xFFFFFFFF);

	if(MEM_TEST_SUCCESS != i)
	{
		return i;
	}

	set_all_memory(0x33333333);
	i = check_all_memory(0x33333333);

	if(MEM_TEST_SUCCESS != i)
	{
		return i;
	}

	set_all_memory(0xCCCCCCCC);
	i = check_all_memory(0xCCCCCCCC);

	if(MEM_TEST_SUCCESS != i)
	{
		return i;
	}

	set_all_memory(0x55555555);
	i = check_all_memory(0x55555555);

	if(MEM_TEST_SUCCESS != i)
	{
		return i;
	}

	set_all_memory(0xAAAAAAAA);
	i = check_all_memory(0xAAAAAAAA);

	if(MEM_TEST_SUCCESS != i)
	{
		return i;
	}

	p1 = 0xCBAD9645;    //ideally this would be randomized
	for (i=0; i<MOD_SZ; i++) {
		p2 = ~p1;
		if( MEM_TEST_SUCCESS != modtst(i, 1, p1, p2)){
			return MEM_TEST_FAIL;
		}

		/* Switch patterns */
		if(MEM_TEST_SUCCESS != modtst(i, 1, p2, p1)){
			return MEM_TEST_FAIL;
		}
	}

	return MEM_TEST_SUCCESS;
}



int modtst(int offset, int iter, ulong p1, ulong p2)
{
	int k, l, done;
	unsigned int *p;
	unsigned int *pe;
	unsigned int *start, *end;
	unsigned int bad;


	// Write every nth location with pattern
	printf("Write every %uth location with pattern: 0x%08x\n",offset, p1);
	start = (unsigned int *) SDRAM_ADDR;
	end = (unsigned int *) (SDRAM_ADDR + BOARD_RAM_SIZE_IN_BYTES - 1);

	end -= MOD_SZ;	// adjust the ending address
	pe = (unsigned int *)start;
	p = start+offset;
	done = 0;
	do {

		// Check for overflow
		if (pe + SPINSZ > pe && pe != 0) {
			pe += SPINSZ;
		} else {
			pe = end;
		}
		if (pe >= end) {
			pe = end;
			done++;
		}
		if (p == pe ) {
			break;
		}

		for (; p <= pe; p += MOD_SZ) {
			*p = p1;
		}

	} while (!done);


	// Write the rest of memory "iter" times with the pattern complement
	for (l=0; l<iter; l++) {
		printf("Write rest of memory with: 0x%08x\n", p2);
		start = (unsigned int *) SDRAM_ADDR;
		end = (unsigned int *) (SDRAM_ADDR + BOARD_RAM_SIZE_IN_BYTES - 1);

		pe = (unsigned int *)start;
		p = start;
		done = 0;
		k = 0;
		do {

			// Check for overflow
			if (pe + SPINSZ > pe && pe != 0) {
				pe += SPINSZ;
			} else {
				pe = end;
			}
			if (pe >= end) {
				pe = end;
				done++;
			}
			if (p == pe ) {
				break;
			}

			for (; p <= pe; p++) {
				if (k != offset) {
					*p = p2;
				}
				if (++k > MOD_SZ-1) {
					k = 0;
				}
			}

			p = pe + 1;
		} while (!done);
	}

	// Now check every nth location

	printf("Check every %uth location\n", offset);
	start = (unsigned int *) SDRAM_ADDR;
	end = (unsigned int *) (SDRAM_ADDR + BOARD_RAM_SIZE_IN_BYTES - 1);

	pe = (unsigned int *)start;
	p = start+offset;
	done = 0;
	end -= MOD_SZ;	// adjust the ending address
	do {

		// Check for overflow
		if (pe + SPINSZ > pe && pe != 0) {
			pe += SPINSZ;
		} else {
			pe = end;
		}
		if (pe >= end) {
			pe = end;
			done++;
		}
		if (p == pe ) {
			break;
		}

		for (; p <= pe; p += MOD_SZ) {
			if ((bad=*p) != p1) {
				printf("Address 0x%08x failed pattern 0x%08x with reading of 0x%08x",p1, (unsigned int *)p, bad);
				return MEM_TEST_FAIL;
			}
		}

	} while (!done);

	return MEM_TEST_SUCCESS;
}

#endif
