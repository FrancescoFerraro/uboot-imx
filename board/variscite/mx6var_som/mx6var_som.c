// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2012-2016 Freescale Semiconductor, Inc.
 * Copyright 2017-2018 NXP
 *
 * Author: Fabio Estevam <fabio.estevam@freescale.com>
 *
 * Copyright (C) 2016-2023 Variscite Ltd.
 *
 * Author: Eran Matityahu <eran.m@variscite.com>
 *         Francesco Ferraro <francesco.f@variscite.com>
 */

#include <image.h>
#include <init.h>
#include <net.h>
#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/mx6-pins.h>
#include <asm/global_data.h>
#include <env.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <asm/gpio.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm/mach-imx/boot_mode.h>
#include <asm/mach-imx/video.h>
#include <mmc.h>
#include <fsl_esdhc_imx.h>
#include <miiphy.h>
#include <asm/arch/mxc_hdmi.h>
#include <asm/arch/crm_regs.h>
#include <asm/io.h>
#include <asm/arch/sys_proto.h>
#include <i2c.h>
#include <input.h>
#include <power/pmic.h>
#include <power/pfuze100_pmic.h>
#include "../common/pfuze.h"
#include <usb.h>
#include <usb/ehci-ci.h>
#include <asm/arch/mx6-ddr.h>
#include <power/regulator.h>
#ifdef CONFIG_FSL_FASTBOOT
#include <fb_fsl.h>
#ifdef CONFIG_ANDROID_RECOVERY
#include <recovery.h>
#endif
#endif /*CONFIG_FSL_FASTBOOT*/
#include <hang.h>
#include <splash.h>

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define USDHC_PAD_CTRL (PAD_CTL_PUS_47K_UP |			\
	PAD_CTL_SPEED_LOW | PAD_CTL_DSE_80ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define I2C_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS |	\
	PAD_CTL_ODE | PAD_CTL_SRE_FAST)

#define OTG_ID_PAD_CTRL (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_47K_UP  | PAD_CTL_SPEED_LOW |		\
	PAD_CTL_DSE_80ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define I2C_PMIC	1

#define I2C_PAD MUX_PAD_CTRL(I2C_PAD_CTRL)

#define VAR_SOM_BACKLIGHT_EN	IMX_GPIO_NR(4, 30)

bool lvds_enabled=false;

/*
 * Returns true if the SOM is VAR-SOM-SOLO
 */
static bool is_som_solo(void)
{
	bool ret;
	int oldbus = i2c_get_bus_num();

	i2c_set_bus_num(PMIC_I2C_BUS);
	/* Probing for PMIC which is preset on all SOM types but SOM-SOLO */
	ret = (0 != i2c_probe(CONFIG_POWER_PFUZE100_I2C_ADDR));

	i2c_set_bus_num(oldbus);
	return ret;
}

/*
 * Returns true if the carrier board is VAR-SOLOCustomBoard
 */
static bool is_solo_custom_board(void)
{
	bool ret;
	int oldbus = i2c_get_bus_num();

	i2c_set_bus_num(1);
	/* Probing for extra EEPROM present only on SOLOCustomBoard */
	ret = (0 == i2c_probe(0x51));

	i2c_set_bus_num(oldbus);
	return ret;
}

/*
 * Returns true if the carrier board is SymphonyBoard
 */
static bool is_symphony_board(void)
{
	bool ret;
	int oldbus = i2c_get_bus_num();

	i2c_set_bus_num(0);
	/* Probing for extra PCA9534 present only on SymphonyBoard */
	ret = (0 == i2c_probe(0x20));

	i2c_set_bus_num(oldbus);
	return ret;
}

static bool is_cpu_pop_packaged(void)
{
	struct src *src_regs = (struct src *)SRC_BASE_ADDR;
	u32 soc_sbmr = readl(&src_regs->sbmr1);
	u8  boot_cfg3 = (soc_sbmr >> 16) & 0xFF;

	/* DDR Memory Map config == 4KB Interleaving Enabled */
	return ((boot_cfg3 & 0x30) == 0x20);
}

/*
 * Returns true iff the carrier board is VAR-DT6CustomBoard
 *  (and the SOM is DART-MX6)
 */
static inline bool is_dart_board(void)
{
	return is_cpu_pop_packaged();
}

/*
 * Returns true iff the carrier board is VAR-MX6CustomBoard
 */
static inline bool is_mx6_custom_board(void)
{
	return (!is_dart_board() && !is_solo_custom_board() && !is_symphony_board());
}

enum current_board {
	DART_BOARD,
	SOLO_CUSTOM_BOARD,
	SYMPHONY_BOARD,
	MX6_CUSTOM_BOARD,
};

static enum current_board get_board_indx(void)
{
	if (is_dart_board())
		return DART_BOARD;
	if (is_symphony_board())
		return SYMPHONY_BOARD;
	if (is_solo_custom_board())
		return SOLO_CUSTOM_BOARD;
	if (is_mx6_custom_board())
		return MX6_CUSTOM_BOARD;

	printf("Error identifying carrier board!\n");
	hang();
}

/*
 * Returns DRAM size in MiB
 */
static u32 var_get_ram_size(void)
{
	u32 *p_ram_size = (u32 *)RAM_SIZE_ADDR;
	return *p_ram_size;
}

int dram_init(void)
{
	gd->ram_size = var_get_ram_size() * 1024 * 1024;
	return 0;
}

static iomux_v3_cfg_t const uart1_pads[] = {
	IOMUX_PADS(PAD_CSI0_DAT10__UART1_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL)),
	IOMUX_PADS(PAD_CSI0_DAT11__UART1_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL)),
};

static iomux_v3_cfg_t const bl_pads[] = {
	IOMUX_PADS(PAD_SD1_DAT3__GPIO1_IO21 | MUX_PAD_CTRL(NO_PAD_CTRL)),
};

static void enable_backlight(void)
{
	struct gpio_desc desc;
	int ret;

	SETUP_IOMUX_PADS(bl_pads);

	ret = dm_gpio_lookup_name("GPIO4_30", &desc);
	if (ret)
		return;

	ret = dm_gpio_request(&desc, "Display Power Enable");
	if (ret)
		return;

	dm_gpio_set_dir_flags(&desc, GPIOD_IS_OUT | GPIOD_IS_OUT_ACTIVE);
}

#if !defined(CONFIG_SPL_BUILD) || defined(CONFIG_SPL_ENV_SUPPORT)
static int env_check(char *var, char *val)
{
	char *read_val;
	if (var == NULL || val == NULL)
		return 0;

	read_val = env_get(var);

	if ((read_val != NULL) &&
			(strcmp(read_val, val) == 0)) {
		return 1;
	}

	return 0;
}
#endif

#ifdef CONFIG_SYS_I2C_LEGACY
#ifdef CONFIG_SYS_I2C_MXC
I2C_PADS(i2c_pad_info1,
	PAD_CSI0_DAT9__I2C1_SCL | MUX_PAD_CTRL(I2C_PAD_CTRL),
	PAD_CSI0_DAT9__GPIO5_IO27 | MUX_PAD_CTRL(I2C_PAD_CTRL),
	IMX_GPIO_NR(5, 27),
	PAD_CSI0_DAT8__I2C1_SDA | MUX_PAD_CTRL(I2C_PAD_CTRL),
	PAD_CSI0_DAT8__GPIO5_IO26 | MUX_PAD_CTRL(I2C_PAD_CTRL),
	IMX_GPIO_NR(5, 26));

I2C_PADS(i2c_pad_info2,
	PAD_KEY_COL3__I2C2_SCL | MUX_PAD_CTRL(I2C_PAD_CTRL),
	PAD_KEY_COL3__GPIO4_IO12 | MUX_PAD_CTRL(I2C_PAD_CTRL),
	IMX_GPIO_NR(4, 12),
	PAD_KEY_ROW3__I2C2_SDA | MUX_PAD_CTRL(I2C_PAD_CTRL),
	PAD_KEY_ROW3__GPIO4_IO13 | MUX_PAD_CTRL(I2C_PAD_CTRL),
	IMX_GPIO_NR(4, 13));

I2C_PADS(i2c_pad_info3,
	PAD_GPIO_5__I2C3_SCL | MUX_PAD_CTRL(I2C_PAD_CTRL),
	PAD_GPIO_5__GPIO1_IO05 | MUX_PAD_CTRL(I2C_PAD_CTRL),
	IMX_GPIO_NR(1, 5),
	PAD_GPIO_16__I2C3_SDA | MUX_PAD_CTRL(I2C_PAD_CTRL),
	PAD_GPIO_16__GPIO7_IO11 | MUX_PAD_CTRL(I2C_PAD_CTRL),
	IMX_GPIO_NR(7, 11));

static void setup_local_i2c(void)
{
	setup_i2c(0, CONFIG_SYS_I2C_SPEED, 0x7f, I2C_PADS_INFO(i2c_pad_info1));
	setup_i2c(1, CONFIG_SYS_I2C_SPEED, 0x7f, I2C_PADS_INFO(i2c_pad_info2));
	setup_i2c(2, CONFIG_SYS_I2C_SPEED, 0x7f, I2C_PADS_INFO(i2c_pad_info3));
}
#endif
#endif

static void setup_iomux_uart(void)
{
	SETUP_IOMUX_PADS(uart1_pads);
}

#ifdef CONFIG_ENV_IS_IN_MMC
int mmc_map_to_kernel_blk(int dev_no)
{
	if ((!is_dart_board()) && (dev_no == 1))
		return 0;
	return dev_no + 1;
}
#endif

#ifdef CONFIG_FSL_ESDHC_IMX
#if !CONFIG_IS_ENABLED(DM_MMC)
static iomux_v3_cfg_t const usdhc1_pads[] = {
	IOMUX_PADS(PAD_SD1_CLK__SD1_CLK		| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD1_CMD__SD1_CMD		| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD1_DAT0__SD1_DATA0	| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD1_DAT1__SD1_DATA1	| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD1_DAT2__SD1_DATA2	| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD1_DAT3__SD1_DATA3	| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
};

static iomux_v3_cfg_t const usdhc2_pads[] = {
	IOMUX_PADS(PAD_SD2_CLK__SD2_CLK		| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD2_CMD__SD2_CMD		| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD2_DAT0__SD2_DATA0	| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD2_DAT1__SD2_DATA1	| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD2_DAT2__SD2_DATA2	| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD2_DAT3__SD2_DATA3	| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
};

static iomux_v3_cfg_t const usdhc3_pads[] = {
	IOMUX_PADS(PAD_SD3_CLK__SD3_CLK		| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_CMD__SD3_CMD		| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT0__SD3_DATA0	| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT1__SD3_DATA1	| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT2__SD3_DATA2	| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT3__SD3_DATA3	| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
};

struct fsl_esdhc_cfg usdhc_cfg[2] = {
	{USDHC1_BASE_ADDR},
	{USDHC2_BASE_ADDR},
};

enum mmc_boot_device {
	USDHC1,
	USDHC2,
	USDHC3,
	USDHC4,
};

int board_mmc_get_env_dev(int devno)
{
	if ((devno == USDHC1) || (devno == USDHC3))
		return 1; /* eMMC (non DART || DART) */
	else if (devno == USDHC2)
		return 0; /* SD card */
	else
		return -1;
}

static int usdhc2_cd_gpio[] = {
	/* DART */
	IMX_GPIO_NR(1, 6),
	/* Non-DART */
	IMX_GPIO_NR(4, 14)
};

int board_mmc_getcd(struct mmc *mmc)
{
	struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;
	int board = is_dart_board() ? 0 : 1;
	int cd;

	/* SD card */
	if (cfg->esdhc_base == USDHC2_BASE_ADDR) {
		cd = gpio_get_value(usdhc2_cd_gpio[board]);
		return !cd;
	}

	/*
	 * On DART SOMs eMMC is always present.
	 *
	 * On non DART SOMs eMMC can be present or not,
	 * but we can't know until we try to init it
	 * so return 1 here anyway
	 */
	return 1;
}

#ifdef CONFIG_SPL_BUILD
static enum mmc_boot_device get_mmc_boot_device(void)
{
	struct src *psrc = (struct src *)SRC_BASE_ADDR;
	unsigned reg = readl(&psrc->sbmr1) >> 11;
	/*
	 * Upon reading BOOT_CFG register
	 * Bit 11 and 12 of BOOT_CFG register can determine the current
	 * mmc port
	 */

	return (reg & 0x3);
}

int board_mmc_init(struct bd_info *bis)
{
	/*
	 * Possible MMC boot devices:
	 * SD1 (eMMC on non DART boards)
	 * SD2 (SD)
	 * SD3 (eMMC on DART board)
	 */
	puts("MMC Boot Device: ");
	switch (get_mmc_boot_device()) {
	case USDHC1:
		puts("mmc1 (eMMC)");
		SETUP_IOMUX_PADS(usdhc1_pads);
		usdhc_cfg[0].esdhc_base = USDHC1_BASE_ADDR;
		usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC_CLK);
		gd->arch.sdhc_clk = usdhc_cfg[0].sdhc_clk;
		usdhc_cfg[0].max_bus_width = 4;
		break;
	case USDHC2:
		puts("mmc0 (SD)");
		SETUP_IOMUX_PADS(usdhc2_pads);
		usdhc_cfg[0].esdhc_base = USDHC2_BASE_ADDR;
		usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC2_CLK);
		gd->arch.sdhc_clk = usdhc_cfg[0].sdhc_clk;
		usdhc_cfg[0].max_bus_width = 4;
		break;
	case USDHC3:
		puts("mmc1 (eMMC)");
		SETUP_IOMUX_PADS(usdhc3_pads);
		usdhc_cfg[0].esdhc_base = USDHC3_BASE_ADDR;
		usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC3_CLK);
		gd->arch.sdhc_clk = usdhc_cfg[0].sdhc_clk;
		usdhc_cfg[0].max_bus_width = 4;
		break;
	default:
		break;
	}
	puts("\n");

	return fsl_esdhc_initialize(bis, &usdhc_cfg[0]);
}
#endif /* ifdef CONFIG_SPL_BUILD */
#endif /* if !CONFIG_IS_ENABLED(DM_MMC) */
#endif /* ifdef CONFIG_FSL_ESDHC_IMX */

int board_phy_config(struct phy_device *phydev)
{
	if (phydev->drv->config)
		phydev->drv->config(phydev);

	return 0;
}

#ifdef CONFIG_SPLASH_SCREEN
#ifdef CONFIG_SPLASH_SOURCE
static void set_splashsource_to_boot_rootfs(void)
{
	if (!env_check("splashsourceauto", "yes"))
		return;

#ifdef CONFIG_NAND_BOOT
	env_set("splashsource", env_get("rootfs_device"));
#else
	if (mmc_get_env_dev() == 0)
		env_set("splashsource", "sd");
	else if (mmc_get_env_dev() == 1)
		env_set("splashsource", "emmc");
#endif
}

int splash_screen_prepare(void)
{
	int ret=0;

	char sd_devpart_str[5];
	char emmc_devpart_str[5];
	u32 sd_part, emmc_part;

	sd_part = emmc_part = env_get_ulong("mmcrootpart", 10, 0);

	sprintf(sd_devpart_str, "1:%d", sd_part);
	sprintf(emmc_devpart_str, "1:%d", emmc_part);

	struct splash_location var_splash_locations[] = {
		{
			.name = "sd",
			.storage = SPLASH_STORAGE_MMC,
			.flags = SPLASH_STORAGE_FS,
			.devpart = sd_devpart_str,
		},
		{
			.name = "emmc",
			.storage = SPLASH_STORAGE_MMC,
			.flags = SPLASH_STORAGE_FS,
			.devpart = emmc_devpart_str,
		},
		{
			.name = "nand",
			.storage = SPLASH_STORAGE_NAND,
			.flags = SPLASH_STORAGE_FS,
			.mtdpart = "rootfs",
			.ubivol = "ubi0:rootfs",
		},
	};

	set_splashsource_to_boot_rootfs();

	ret = splash_source_load(var_splash_locations,
			ARRAY_SIZE(var_splash_locations));

	/* Turn on backlight */
	if (lvds_enabled)
		gpio_set_value(VAR_SOM_BACKLIGHT_EN, 1);

	return ret;
}
#endif /* ifdef CONFIG_SPLASH_SOURCE */
#endif /* ifdef CONFIG_SPLASH_SCREEN */

#if defined(CONFIG_VIDEO_IPUV3)
static void disable_lvds(struct display_info_t const *dev)
{
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;
	int reg = readl(&iomux->gpr[2]);

	reg &= ~(IOMUXC_GPR2_LVDS_CH0_MODE_MASK |
		 IOMUXC_GPR2_LVDS_CH1_MODE_MASK);

	writel(reg, &iomux->gpr[2]);
}

#ifdef HDMI_ENABLE
static void do_enable_hdmi(struct display_info_t const *dev)
{
	disable_lvds(dev);
	imx_enable_hdmi_phy();
}
#endif

static void lvds_enable_disable(struct display_info_t const *dev)
{
	if (env_get("splashimage") != NULL)
		lvds_enabled=true;
	else
		disable_lvds(dev);
}

static int detect_dart_vsc_display(struct display_info_t const *dev)
{
	return (!is_mx6_custom_board());
}

static int detect_mx6cb_cdisplay(struct display_info_t const *dev)
{
	if (!is_mx6_custom_board())
		return 0;

	i2c_set_bus_num(dev->bus);
	return (0 == i2c_probe(dev->addr));
}

static int detect_mx6cb_rdisplay(struct display_info_t const *dev)
{
	if (!is_mx6_custom_board())
		return 0;

	/* i2c probe the *c*display */
	i2c_set_bus_num(MX6CB_CDISPLAY_I2C_BUS);
	return (0 != i2c_probe(MX6CB_CDISPLAY_I2C_ADDR));
}

#define MHZ2PS(f)	(1000000/(f))

struct display_info_t const displays[] = {{
#ifdef HDMI_ENABLE
	.bus	= -1,
	.addr	= 0,
	.pixfmt	= IPU_PIX_FMT_RGB24,
	.detect	= detect_hdmi,
	.enable	= do_enable_hdmi,
	.mode	= {
		.name           = "HDMI",
		.refresh        = 60,  /* optional */
		.xres           = 800,
		.yres           = 480,
		.pixclock       = 31777,
		.left_margin    = 48,
		.right_margin   = 16,
		.upper_margin   = 33,
		.lower_margin   = 10,
		.hsync_len      = 96,
		.vsync_len      = 2,
		.sync           = 0,
		.vmode          = FB_VMODE_NONINTERLACED
} }, {
#endif
	.bus	= -1,
	.addr	= 0,
	.pixfmt	= IPU_PIX_FMT_RGB666,
	.detect	= detect_dart_vsc_display,
	.enable	= lvds_enable_disable,
	.mode	= {
		.name           = "VAR-WVGA",
		.refresh        = 60,  /* optional */
		.xres           = 800,
		.yres           = 480,
		.pixclock       = MHZ2PS(50),
		.left_margin    = 40,
		.right_margin   = 40,
		.upper_margin   = 29,
		.lower_margin   = 13,
		.hsync_len      = 48,
		.vsync_len      = 3,
		.sync           = FB_SYNC_EXT,
		.vmode          = FB_VMODE_NONINTERLACED
} }, {
	.bus	= MX6CB_CDISPLAY_I2C_BUS,
	.addr	= MX6CB_CDISPLAY_I2C_ADDR,
	.pixfmt	= IPU_PIX_FMT_RGB24,
	.detect	= detect_mx6cb_cdisplay,
	.enable	= lvds_enable_disable,
	.mode	= {
		.name           = "VAR-WVGA MX6CB-C",
		.refresh        = 60,  /* optional */
		.xres           = 800,
		.yres           = 480,
		.pixclock       = MHZ2PS(50),
		.left_margin    = 39,
		.right_margin   = 39,
		.upper_margin   = 29,
		.lower_margin   = 13,
		.hsync_len      = 128,
		.vsync_len      = 2,
		.sync           = FB_SYNC_EXT,
		.vmode          = FB_VMODE_NONINTERLACED
} }, {
	.bus	= -1,
	.addr	= 0,
	.pixfmt	= IPU_PIX_FMT_RGB24,
	.detect	= detect_mx6cb_rdisplay,
	.enable	= lvds_enable_disable,
	.mode	= {
		.name           = "VAR-WVGA MX6CB-R",
		.refresh        = 60,  /* optional */
		.xres           = 800,
		.yres           = 480,
		.pixclock       = MHZ2PS(50),
		.left_margin    = 0,
		.right_margin   = 40,
		.upper_margin   = 20,
		.lower_margin   = 13,
		.hsync_len      = 48,
		.vsync_len      = 3,
		.sync           = FB_SYNC_EXT,
		.vmode          = FB_VMODE_NONINTERLACED
} } };
size_t display_count = ARRAY_SIZE(displays);

static void setup_display(void)
{
	struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;
	int reg;

	/* Setup backlight */
	SETUP_IOMUX_PAD(PAD_DISP0_DAT9__GPIO4_IO30 | MUX_PAD_CTRL(NO_PAD_CTRL));

	/* Turn off backlight until display is ready */
	gpio_request(VAR_SOM_BACKLIGHT_EN, "Display Backlight Enable");
	gpio_direction_output(VAR_SOM_BACKLIGHT_EN , 1);

	enable_ipu_clock();
	imx_setup_hdmi();

	/* Turn on LDB0, LDB1, IPU,IPU DI0 clocks */
	reg = readl(&mxc_ccm->CCGR3);
	reg |=  MXC_CCM_CCGR3_LDB_DI0_MASK | MXC_CCM_CCGR3_LDB_DI1_MASK;
	writel(reg, &mxc_ccm->CCGR3);

	/* set LDB0, LDB1 clk select to 011/011 */
	reg = readl(&mxc_ccm->cs2cdr);
	reg &= ~(MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_MASK
		 | MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_MASK);
	/* 1 -> ~50MHz , 2 -> ~56MHz, 3 -> ~75MHz, 4 -> ~68MHz */
	reg |= (3 << MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_OFFSET)
	      | (3 << MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_OFFSET);
	writel(reg, &mxc_ccm->cs2cdr);

	reg = readl(&mxc_ccm->cscmr2);
	reg |= MXC_CCM_CSCMR2_LDB_DI0_IPU_DIV | MXC_CCM_CSCMR2_LDB_DI1_IPU_DIV;
	writel(reg, &mxc_ccm->cscmr2);

	reg = readl(&mxc_ccm->chsccdr);
	reg |= (CHSCCDR_CLK_SEL_LDB_DI0
		<< MXC_CCM_CHSCCDR_IPU1_DI0_CLK_SEL_OFFSET);
	reg |= (CHSCCDR_CLK_SEL_LDB_DI0
		<< MXC_CCM_CHSCCDR_IPU1_DI1_CLK_SEL_OFFSET);
	writel(reg, &mxc_ccm->chsccdr);

	reg = IOMUXC_GPR2_BGREF_RRMODE_EXTERNAL_RES
	     | IOMUXC_GPR2_DI1_VS_POLARITY_ACTIVE_LOW
	     | IOMUXC_GPR2_DI0_VS_POLARITY_ACTIVE_LOW
	     | IOMUXC_GPR2_BIT_MAPPING_CH1_SPWG
	     | IOMUXC_GPR2_DATA_WIDTH_CH1_18BIT
	     | IOMUXC_GPR2_BIT_MAPPING_CH0_SPWG
	     | (is_mx6_custom_board() ? IOMUXC_GPR2_DATA_WIDTH_CH0_24BIT : IOMUXC_GPR2_DATA_WIDTH_CH0_18BIT)
	     | IOMUXC_GPR2_LVDS_CH0_MODE_ENABLED_DI0
	     | IOMUXC_GPR2_LVDS_CH1_MODE_ENABLED_DI0;
	writel(reg, &iomux->gpr[2]);

	reg = readl(&iomux->gpr[3]);
	reg = (reg & ~(IOMUXC_GPR3_LVDS1_MUX_CTL_MASK | IOMUXC_GPR3_HDMI_MUX_CTL_MASK))
		| (IOMUXC_GPR3_MUX_SRC_IPU1_DI0 << IOMUXC_GPR3_LVDS1_MUX_CTL_OFFSET);
	writel(reg, &iomux->gpr[3]);
}
#endif /* CONFIG_VIDEO_IPUV3 */

/*
 * Do not overwrite the console
 * Use always serial for U-Boot console
 */
int overwrite_console(void)
{
	return 1;
}

int board_early_init_f(void)
{
	setup_iomux_uart();
#ifdef CONFIG_SYS_I2C_LEGACY
	setup_local_i2c();
#endif
#ifdef CONFIG_NAND_MXS
	setup_gpmi_nand();
#endif

	return 0;
}

int board_init(void)
{
#if defined(CONFIG_VIDEO_IPUV3)
	setup_display();
#elif defined(CONFIG_IMX_HDMI)
	setup_hdmi();
#endif
	return 0;
}

#ifdef CONFIG_POWER_LEGACY
int power_init_board(void)
{
	//struct pmic *pfuze;
	return 0;
}

#elif defined(CONFIG_DM_PMIC_PFUZE100)
int power_init_board(void)
{
	//struct udevice *dev;
	return 0;
}
#endif

#ifdef CONFIG_LDO_BYPASS_CHECK
#ifdef CONFIG_POWER_LEGACY
void ldo_mode_set(int ldo_bypass)
{
	//struct pmic *p = pmic_get("PFUZE100");
}
#elif defined(CONFIG_DM_PMIC_PFUZE100)
void ldo_mode_set(int ldo_bypass)
{
	//struct udevice *dev;
}
#endif
#endif

int board_late_init(void)
{
#if 0
#ifdef CONFIG_ENV_IS_IN_MMC
	mmc_late_init();
#endif

	print_emmc_size();
#endif

#ifdef CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG
	if (is_dart_board())
		env_set("board_name", "DT6CUSTOM");
	else if (is_symphony_board())
		env_set("board_name", "SYMPHONY");
	else if (is_solo_custom_board())
		env_set("board_name", "SOLOCUSTOM");
	else
		env_set("board_name", "MX6CUSTOM");

	if (is_som_solo())
		env_set("board_som", "SOM-SOLO");
	else if (is_dart_board())
		env_set("board_som", "DART-MX6");
	else
		env_set("board_som", "SOM-MX6");

	if (is_mx6dqp())
		env_set("board_rev", "MX6QP");
	else if (is_mx6dq())
		env_set("board_rev", "MX6Q");
	else if (is_mx6sdl())
		env_set("board_rev", "MX6DL");
#endif

#ifdef CONFIG_EXTCON_PTN5150
	extcon_ptn5150_setup(&usb_ptn5150);
#endif

	return 0;
}

#ifdef CONFIG_FSL_FASTBOOT
#ifdef CONFIG_ANDROID_RECOVERY

#define GPIO_VOL_DN_KEY IMX_GPIO_NR(1, 5)
iomux_v3_cfg_t const recovery_key_pads[] = {
	IOMUX_PADS(PAD_GPIO_5__GPIO1_IO05 | MUX_PAD_CTRL(NO_PAD_CTRL)),
};

int is_recovery_key_pressing(void)
{
	int button_pressed = 0;
	int ret;
	struct gpio_desc desc;

	/* Check Recovery Combo Button press or not. */
	SETUP_IOMUX_PADS(recovery_key_pads);

	ret = dm_gpio_lookup_name("GPIO1_5", &desc);
	if (ret) {
		printf("%s lookup GPIO1_5 failed ret = %d\n", __func__, ret);
		return;
	}

	ret = dm_gpio_request(&desc, "volume_dn_key");
	if (ret) {
		printf("%s request volume_dn_key failed ret = %d\n", __func__, ret);
		return;
	}

	dm_gpio_set_dir_flags(&desc, GPIOD_IS_IN);

	if (dm_gpio_get_value(&desc) == 0) { /* VOL_DN key is low assert */
		button_pressed = 1;
		printf("Recovery key pressed\n");
	}

	return  button_pressed;
}

#endif /*CONFIG_ANDROID_RECOVERY*/

#endif /*CONFIG_FSL_FASTBOOT*/

#ifdef CONFIG_SPL_BUILD
#include <asm/arch/mx6-ddr.h>
#include <spl.h>
#include <linux/libfdt.h>
#include "mx6var_dram.h"

#ifdef CONFIG_SPL_OS_BOOT
int spl_start_uboot(void)
{
	return 0;
}
#endif

/*
 * Writes RAM size (MiB) to RAM_SIZE_ADDR so U-Boot can read it
 */
void var_set_ram_size(u32 ram_size)
{
	u32 *p_ram_size = (u32 *)RAM_SIZE_ADDR;
	if (ram_size > 3840)
		ram_size = 3840;
	*p_ram_size = ram_size;
}

static void ccgr_init(void)
{
	struct mxc_ccm_reg *ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;

	writel(0x00C03F3F, &ccm->CCGR0);
	writel(0x0030FC03, &ccm->CCGR1);
	writel(0x0FFFC000, &ccm->CCGR2);
	writel(0x3FF00000, &ccm->CCGR3);
	writel(0x00FFF300, &ccm->CCGR4);
	writel(0x0F0000C3, &ccm->CCGR5);
	writel(0x000003FF, &ccm->CCGR6);
}

/*
 * Bugfix: Fix Freescale wrong processor documentation.
 */
static void spl_mx6qd_dram_setup_iomux_check_reset(void)
{
	if (is_mx6dq() || is_mx6dqp()) {
		volatile struct mx6dq_iomux_ddr_regs *mx6dq_ddr_iomux;

		mx6dq_ddr_iomux = (struct mx6dq_iomux_ddr_regs *) MX6DQ_IOM_DDR_BASE;

		if (mx6dq_ddr_iomux->dram_reset == (u32)0x000C0030)
			mx6dq_ddr_iomux->dram_reset = (u32)0x00000030;
	}
}

static void spl_dram_init(void)
{
	if (is_dart_board())
		var_eeprom_v2_dram_init();
	else
		if (var_eeprom_v1_dram_init())
			var_legacy_dram_init(is_som_solo());

	spl_mx6qd_dram_setup_iomux_check_reset();
}

void board_init_f(ulong dummy)
{
	/* setup AIPS and disable watchdog */
	arch_cpu_init();

	ccgr_init();
	gpr_init();

	/* iomux and setup of i2c */
	board_early_init_f();

	/* setup GP timer */
	timer_init();

	/* UART clocks enabled and gd valid - init serial console */
	preloader_console_init();

	/* DDR initialization */
	spl_dram_init();

	/* Clear the BSS. */
	memset(__bss_start, 0, __bss_end - __bss_start);	/* load/boot image from boot device */

	board_init_r(NULL, 0);
}
#endif

#ifdef CONFIG_SPL_LOAD_FIT
int board_fit_config_name_match(const char *name)
{
	int idx = get_board_indx();

	if ((idx == DART_BOARD) && !strcmp(name, "imx6q-var-dart"))
		return 0;
	else if ((idx == SYMPHONY_BOARD) && !strcmp(name, "imx6dl-var-som-solo-symphony"))
		return 0;
#if 0
	else if ((idx == SOLO_CUSTOM_BOARD) && !strcmp(name, ""))
		return 0;
	else if ((idx == MX6_CUSTOM_BOARD) && !strcmp(name, ""))
		return 0;
#endif
	else
		return -1;
}
#endif
