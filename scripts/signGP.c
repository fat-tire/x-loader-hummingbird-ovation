//
// signGP.c
// Read the x-load.bin file and write out the x-load.bin.ift file.
// The signed image is the original pre-pended with the size of the image
// and the load address.  If not entered on command line, file name is
// assumed to be x-load.bin in current directory and load address is
// 0x40200800.

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <string.h>
#include <malloc.h>
#include <stdint.h>

#undef CH_WITH_CHRAM
struct chsettings {
	uint32_t section_key;
	uint8_t valid;
	uint8_t version;
	uint16_t reserved;
	uint32_t flags;
} __attribute__ ((__packed__));

/*    uint32_t  cm_clksel_core;
    uint32_t  reserved1;
    uint32_t  cm_autoidle_dpll_mpu;
    uint32_t  cm_clksel_dpll_mpu;
    uint32_t  cm_div_m2_dpll_mpu;
    uint32_t  cm_autoidle_dpll_core;
    uint32_t  cm_clksel_dpll_core;
    uint32_t  cm_div_m2_dpll_core;
    uint32_t  cm_div_m3_dpll_core;
    uint32_t  cm_div_m4_dpll_core;
    uint32_t  cm_div_m5_dpll_core;
    uint32_t  cm_div_m6_dpll_core;
    uint32_t  cm_div_m7_dpll_core;
    uint32_t  cm_autoidle_dpll_per;
    uint32_t  cm_clksel_dpll_per;
    uint32_t  cm_div_m2_dpll_per;
    uint32_t  cm_div_m3_dpll_per;
    uint32_t  cm_div_m4_dpll_per;
    uint32_t  cm_div_m5_dpll_per;
    uint32_t  cm_div_m6_dpll_per;
    uint32_t  cm_div_m7_dpll_per;
    uint32_t  cm_autoidle_dpll_usb;
    uint32_t  cm_clksel_dpll_usb;
    uint32_t  cm_div_m2_dpll_usb;
}*/

struct gp_header {
	uint32_t size;
	uint32_t load_addr;
} __attribute__ ((__packed__));

struct ch_toc {
	uint32_t section_offset;
	uint32_t section_size;
	uint8_t unused[12];
	uint8_t section_name[12];
} __attribute__ ((__packed__));

struct chram {
	/*CHRAM */
	uint32_t section_key_chr;
	uint8_t section_disable_chr;
	uint8_t pad_chr[3];
	/*EMIF1 */
	uint32_t config_emif1;
	uint32_t refresh_emif1;
	uint32_t tim1_emif1;
	uint32_t tim2_emif1;
	uint32_t tim3_emif1;
	uint32_t pwrControl_emif1;
	uint32_t phy_cntr1_emif1;
	uint32_t phy_cntr2_emif1;
	uint8_t modereg1_emif1;
	uint8_t modereg2_emif1;
	uint8_t modereg3_emif1;
	uint8_t pad_emif1;
	/*EMIF2 */
	uint32_t config_emif2;
	uint32_t refresh_emif2;
	uint32_t tim1_emif2;
	uint32_t tim2_emif2;
	uint32_t tim3_emif2;
	uint32_t pwrControl_emif2;
	uint32_t phy_cntr1_emif2;
	uint32_t phy_cntr2_emif2;
	uint8_t modereg1_emif2;
	uint8_t modereg2_emif2;
	uint8_t modereg3_emif2;
	uint8_t pad_emif2;

	uint32_t dmm_lisa_map;
	uint8_t flags;
	uint8_t pad[3];
} __attribute__ ((__packed__));


struct ch_chsettings_chram {
	struct ch_toc toc_chsettings;
	struct ch_toc toc_chram;
	struct ch_toc toc_terminator;
	struct chsettings section_chsettings;
	struct chram section_chram;
	uint8_t padding1[512 -
		    (sizeof(struct ch_toc) * 3 +
		     sizeof(struct chsettings) + sizeof(struct chram))];
	//struct gp_header gpheader;
} __attribute__ ((__packed__));

struct ch_chsettings_nochram {
	struct ch_toc toc_chsettings;
	struct ch_toc toc_terminator;
	struct chsettings section_chsettings;
	uint8_t padding1[512 -
		    (sizeof(struct ch_toc) * 2 +
		     sizeof(struct chsettings))];
	//struct gp_header gpheader;
} __attribute__ ((__packed__));


#ifdef CH_WITH_CHRAM
const struct ch_chsettings_chram config_header = {
	//CHSETTINGS TOC
	{sizeof(struct ch_toc) * 4,
	 sizeof(struct chsettings),
	 "",
	 {"CHSETTINGS"}
	 },
	//CHRAM TOC
	{sizeof(struct ch_toc) * 4 + sizeof(struct chsettings),
	 sizeof(struct chram),
	 "",
	 {"CHRAM"}
	 },
	// toc terminator
	{0xFFFFFFFF,
	 0xFFFFFFFF,
	 {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	  0xFF},
	 {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	  0xFF}
	 },
	//CHSETTINGS section
	{
	 0xC0C0C0C1,
	 0,
	 1,
	 0,
	 0},
	//CHRAM section
	{
	 0xc0c0c0c2,
	 0x01,
	 {0x00, 0x00, 0x00},

	 /*EMIF1 */
	 0x80800eb2,
	 0x00000010,
	 0x110d1624,
	 0x3058161b,
	 0x030060b2,
	 0x00000200,
	 0x901ff416,
	 0x00000000,
	 0x23,
	 0x01,
	 0x02,
	 0x00,

	 /*EMIF2 */
	 0x80800eb2,
	 0x000002ba,
	 0x110d1624,
	 0x3058161b,
	 0x03006542,
	 0x00000200,
	 0x901ff416,
	 0x00000000,
	 0x23,
	 0x01,
	 0x02,
	 0x00,

	 /* LISA map */
	 0x80700100,
	 0x05,
	 {0x00, 0x00, 0x00},
	 },
	""
};
#else
#ifdef __APPLE__
struct ch_chsettings_nochram config_header  __attribute__((section("__DATA, .config_header"))) = {
#else
struct ch_chsettings_nochram config_header  __attribute__((section(".config_header"))) = {
#endif
	//CHSETTINGS TOC
	{(sizeof(struct ch_toc)) * 2,
	 sizeof(struct chsettings),
	 "",
	 {"CHSETTINGS"}
	 },
	// toc terminator
	{0xFFFFFFFF,
	 0xFFFFFFFF,
	 {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	  0xFF},
	 {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	  0xFF}
	 },
	//CHSETTINGS section
	{
	 0xC0C0C0C1,
	 0,
	 1,
	 0,
	 0},
	""
};
#endif


main(int argc, char *argv[])
{
	int	i;
	char	ifname[FILENAME_MAX], ofname[FILENAME_MAX], ch;
	FILE	*ifile, *ofile;
	unsigned long	loadaddr, len;
	struct stat	sinfo;
	size_t ret;


	// Default to x-load.bin and 0x40200800.
	strcpy(ifname, "x-load.bin");
	loadaddr = 0x40200800;

	if ((argc == 2) || (argc == 3))
		strcpy(ifname, argv[1]);

	if (argc == 3)
		loadaddr = strtoul(argv[2], NULL, 16);

	// Form the output file name.
	strcpy(ofname, ifname);
	strcat(ofname, ".ift");

	// Open the input file.
	ifile = fopen(ifname, "rb");
	if (ifile == NULL) {
		printf("Cannot open %s\n", ifname);
		exit(0);
	}

	// Get file length.
	stat(ifname, &sinfo);
	len = sinfo.st_size;

	// Open the output file and write it.
	ofile = fopen(ofname, "wb");
	if (ofile == NULL) {
		printf("Cannot open %s\n", ofname);
		fclose(ifile);
		exit(0);
	}

	fwrite(&config_header, 1, 512, ofile);
	fwrite(&len, 1, 4, ofile);
	fwrite(&loadaddr, 1, 4, ofile);
	for (i=0; i<len; i++) {
		ret = fread(&ch, 1, 1, ifile);
		if (ret)
			fwrite(&ch, 1, 1, ofile);
	}

	fclose(ifile);
	fclose(ofile);
}
