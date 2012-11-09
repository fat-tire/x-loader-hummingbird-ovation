/*
 * (C) Copyright 2000
 * Rob Taylor, Flying Pig Systems. robt@flyingpig.com.
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

#if (CONFIG_CONS_INDEX == 1)
#include <asm/arch/cpu.h>
#endif

#ifdef CFG_PRINTF
#ifdef CFG_NS16550_SERIAL

#include <ns16550.h>
#ifdef CFG_NS87308
#include <ns87308.h>
#endif

#if CONFIG_CONS_INDEX == 1
static NS16550_t console = (NS16550_t) CFG_NS16550_COM1;
#elif CONFIG_CONS_INDEX == 2
static NS16550_t console = (NS16550_t) CFG_NS16550_COM2;
#elif CONFIG_CONS_INDEX == 3
static NS16550_t console = (NS16550_t) CFG_NS16550_COM3;
#elif CONFIG_CONS_INDEX == 4
static NS16550_t console = (NS16550_t) CFG_NS16550_COM4;
#else
#error no valid console defined
#endif

/*******************************************************
 * Routine: delay
 * Description: spinning delay to use before udelay works
 ******************************************************/
static inline void delay(unsigned long loops)
{
	__asm__ volatile ("1:\n" "subs %0, %1, #1\n"
			  "bne 1b":"=r" (loops):"0"(loops));
}
static inline void udelay(unsigned long us)
{
	delay(us * 200); /* approximate */
}

static int calc_divisor (void)
{
//	DECLARE_GLOBAL_DATA_PTR;
#ifdef CONFIG_OMAP1510
	/* If can't cleanly clock 115200 set div to 1 */
	if ((CFG_NS16550_CLK == 12000000) && (CONFIG_BAUDRATE == 115200)) {
		console->osc_12m_sel = OSC_12M_SEL;	/* enable 6.5 * divisor */
		return (1);				/* return 1 for base divisor */
	}
	console->osc_12m_sel = 0;			/* clear if previsouly set */
#endif
#if defined(CONFIG_OMAP1610) || defined(CONFIG_OMAP1710)
	/* If can't cleanly clock 115200 set div to 1 */
	if ((CFG_NS16550_CLK == 48000000) && (CONFIG_BAUDRATE == 115200)) {
		return (26);		/* return 26 for base divisor */
	}
#endif
	return (CFG_NS16550_CLK / 16 / CONFIG_BAUDRATE);
}

#if (CONFIG_CONS_INDEX == 1)
/* On Hummingbird these GPIOs must be High in order UART1 on
30-pin connector to work */
void set_uart1_gpios(void) {
	/* set gpio 182 - 30pin-I2C2-UART-SEL to High */
	sr32(0x4805D134, 182 - 160, 1, 0);
	sr32(0x4805D13C, 182 - 160, 1, 0x1);

	/* set gpio 60 to High */
	sr32(0x48055134, 60 - 32, 1, 0);
	sr32(0x4805513C, 60 - 32, 1, 0x1);

	/*  Startup time from CT_CP_HPD input to 5 V for DC-DC is 300us (TPD12S015A) */
	udelay(300);

	/* set gpio 81 to High */
	sr32(0x48057134, 81 - 64, 1, 0);
	sr32(0x4805713C, 81 - 64, 1, 0x1);
}
#endif

int serial_init (void)
{
	int clock_divisor = calc_divisor();

#ifdef CFG_NS87308
	initialise_ns87308();
#endif

	NS16550_init(console, clock_divisor);

#if (CONFIG_CONS_INDEX == 1)
	if (omap_revision() == OMAP4470_ES1_0)
		set_uart1_gpios();
#endif
	
	return (0);
}

void
serial_putc(const char c)
{
	if (c == '\n')
		NS16550_putc(console, '\r');

	NS16550_putc(console, c);
}

void
serial_puts (const char *s)
{
	while (*s) {
		serial_putc (*s++);
	}
}


int
serial_getc(void)
{
	return NS16550_getc(console);
}

int
serial_tstc(void)
{
	return NS16550_tstc(console);
}

void
serial_setbrg (void)
{
	int clock_divisor;

    clock_divisor = calc_divisor();
	NS16550_reinit(console, clock_divisor);
}

#endif
#endif
