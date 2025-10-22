// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2023-2024 NXP
 * Copyright 2024-2025 Variscite Ltd.
 */

#include <env.h>
#include <init.h>
#include <asm/arch/clock.h>
#include <usb.h>
#include <miiphy.h>
#include <netdev.h>
#include <dwc3-uboot.h>
#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <asm/gpio.h>
#include <power/regulator.h>
#include <scmi_agent.h>
#include "../dts/upstream/src/arm64/freescale/imx95-power.h"
#include <asm/arch/sys_proto.h>
#include <i2c.h>
#include <dm/uclass.h>
#include <dm/uclass-internal.h>

#include "../common/imx9_eeprom.h"
#include "../common/extcon-ptn5150.h"

DECLARE_GLOBAL_DATA_PTR;

extern int var_setup_mac(struct var_eeprom *eeprom);
extern int board_fix_fdt_fuse(void *fdt);
static int board_sm_cfg_info(void);

/* Carrier board EEPROM */
#define CARRIER_EEPROM_I2C_NAME		"i2c@42530000"
#define CARRIER_EEPROM_ADDR		0x54

static struct var_eeprom eeprom = {0};
static bool m7_is_powered = false;

int board_early_init_f(void)
{
	/* UART1: A55, UART3: M7 */
	init_uart_clk(0);

	return 0;
}

#ifdef CONFIG_USB_DWC3

#define PHY_CTRL0			0xF0040
#define PHY_CTRL0_REF_SSP_EN		BIT(2)
#define PHY_CTRL0_FSEL_MASK		GENMASK(10, 5)
#define PHY_CTRL0_FSEL_24M		0x2a
#define PHY_CTRL0_FSEL_100M		0x27
#define PHY_CTRL0_SSC_RANGE_MASK	GENMASK(23, 21)
#define PHY_CTRL0_SSC_RANGE_4003PPM	(0x2 << 21)

#define PHY_CTRL1			0xF0044
#define PHY_CTRL1_RESET			BIT(0)
#define PHY_CTRL1_COMMONONN		BIT(1)
#define PHY_CTRL1_ATERESET		BIT(3)
#define PHY_CTRL1_DCDENB		BIT(17)
#define PHY_CTRL1_CHRGSEL		BIT(18)
#define PHY_CTRL1_VDATSRCENB0		BIT(19)
#define PHY_CTRL1_VDATDETENB0		BIT(20)

#define PHY_CTRL2			0xF0048
#define PHY_CTRL2_TXENABLEN0		BIT(8)
#define PHY_CTRL2_OTG_DISABLE		BIT(9)

#define PHY_CTRL6			0xF0058
#define PHY_CTRL6_RXTERM_OVERRIDE_SEL	BIT(29)
#define PHY_CTRL6_ALT_CLK_EN		BIT(1)
#define PHY_CTRL6_ALT_CLK_SEL		BIT(0)

static struct dwc3_device dwc3_device_data = {
	.maximum_speed = USB_SPEED_HIGH,
	.base = USB1_BASE_ADDR,
	.dr_mode = USB_DR_MODE_PERIPHERAL,
	.index = 0,
	.power_down_scale = 2,
};

static void dwc3_nxp_usb_phy_init(struct dwc3_device *dwc3)
{
	u32 value;

	/* USB3.0 PHY signal fsel for 24M ref */
	value = readl(dwc3->base + PHY_CTRL0);
	value &= ~PHY_CTRL0_FSEL_MASK;
	value |= FIELD_PREP(PHY_CTRL0_FSEL_MASK, PHY_CTRL0_FSEL_24M);
	writel(value, dwc3->base + PHY_CTRL0);

	/* Disable alt_clk_en and use internal MPLL clocks */
	value = readl(dwc3->base + PHY_CTRL6);
	value &= ~(PHY_CTRL6_ALT_CLK_SEL | PHY_CTRL6_ALT_CLK_EN);
	writel(value, dwc3->base + PHY_CTRL6);

	value = readl(dwc3->base + PHY_CTRL1);
	value &= ~(PHY_CTRL1_VDATSRCENB0 | PHY_CTRL1_VDATDETENB0);
	value |= PHY_CTRL1_RESET | PHY_CTRL1_ATERESET;
	writel(value, dwc3->base + PHY_CTRL1);

	value = readl(dwc3->base + PHY_CTRL0);
	value |= PHY_CTRL0_REF_SSP_EN;
	writel(value, dwc3->base + PHY_CTRL0);

	value = readl(dwc3->base + PHY_CTRL2);
	value |= PHY_CTRL2_TXENABLEN0 | PHY_CTRL2_OTG_DISABLE;
	writel(value, dwc3->base + PHY_CTRL2);

	udelay(10);

	value = readl(dwc3->base + PHY_CTRL1);
	value &= ~(PHY_CTRL1_RESET | PHY_CTRL1_ATERESET);
	writel(value, dwc3->base + PHY_CTRL1);
}
#endif

static int imx9_scmi_power_domain_enable(u32 domain, bool enable)
{
	struct udevice *dev;
	int ret;

	ret = uclass_get_device_by_name(UCLASS_CLK, "protocol@14", &dev);
	if (ret)
		return ret;

	return scmi_pwd_state_set(dev, 0, domain, enable ? 0 : BIT(30));
}

#ifdef CONFIG_EXTCON_PTN5150
static struct extcon_ptn5150 usb_ptn5150;
#endif

int board_usb_init(int index, enum usb_init_type init)
{
	int ret = 0;

#if (!defined(CONFIG_SPL_BUILD) && defined(CONFIG_EXTCON_PTN5150))
	if (index == 0) {
		/* Verify port is in proper mode */
		int phy_mode = extcon_ptn5150_phy_mode(&usb_ptn5150);

		/* Only verify phy_mode if ptn5150 is initialized */
		if (phy_mode >= 0 && phy_mode != init)
			return -ENODEV;
	}
#endif

	if (index == 0 && init == USB_INIT_DEVICE) {
		ret = imx9_scmi_power_domain_enable(IMX95_PD_HSIO_TOP, true);
		if (ret) {
			printf("SCMI_POWWER_STATE_SET Failed for USB\n");
			return ret;
		}

#ifdef CONFIG_USB_DWC3
		dwc3_nxp_usb_phy_init(&dwc3_device_data);
		return dwc3_uboot_init(&dwc3_device_data);
#endif
	} else if (index == 0 && init == USB_INIT_HOST) {
		return ret;
	}

	return 0;
}

int board_usb_cleanup(int index, enum usb_init_type init)
{
	int ret = 0;
	if (index == 0 && init == USB_INIT_DEVICE) {
#ifdef CONFIG_USB_DWC3
		dwc3_uboot_exit(index);
#endif
	}

	return ret;
}

static void netc_phy_rst(const char *gpio_name, const char *label)
{
	int ret;
	struct gpio_desc desc;

	/* ENET_RST_B */
	ret = dm_gpio_lookup_name(gpio_name, &desc);
	if (ret) {
		printf("%s lookup %s failed ret = %d\n", __func__, gpio_name, ret);
		return;
	}

	ret = dm_gpio_request(&desc, label);
	if (ret) {
		printf("%s request %s failed ret = %d\n", __func__, label, ret);
		return;
	}

	/* assert the ENET_RST_B */
	dm_gpio_set_dir_flags(&desc, GPIOD_IS_OUT | GPIOD_IS_OUT_ACTIVE | GPIOD_ACTIVE_LOW);
	udelay(10000);
	dm_gpio_set_value(&desc, 0); /* deassert the ENET_RST_B */
	udelay(100000);

}

void netc_init(void)
{
	int ret;

	/* Power up the NETC MIX. */
	ret = imx9_scmi_power_domain_enable(IMX95_PD_NETC, true);
	if (ret) {
		printf("SCMI_POWWER_STATE_SET Failed for NETC MIX\n");
		return;
	}

	set_clk_netc(ENET_125MHZ);

	netc_phy_rst("GPIO5_16", "ENET1_RST_B");
	netc_phy_rst("i2c8_io@21_0", "ENET2_RST_B");
	netc_phy_rst("i2c3_io@22_5", "ETH10G_SEL");

	pci_init();
}

#if CONFIG_IS_ENABLED(NET)
int board_phy_config(struct phy_device *phydev)
{
	if (phydev->drv->config)
		phydev->drv->config(phydev);
	return 0;
}
#endif

int board_init(void)
{
	int ret;
	ret = imx9_scmi_power_domain_enable(IMX95_PD_HSIO_TOP, true);
	if (ret) {
		printf("SCMI_POWWER_STATE_SET Failed for USB\n");
		return ret;
	}

	imx9_scmi_power_domain_enable(IMX95_PD_DISPLAY, false);
	imx9_scmi_power_domain_enable(IMX95_PD_CAMERA, false);

	netc_init();

	m7_is_powered = false;
	if (power_on_m7("dart-mx95-m7") == 0) {
		m7_is_powered = true;
	} else if (power_on_m7("dart-mx95-m7deb") == 0) {
		m7_is_powered = true;
	} else {
		printf("Cortex M7 core not powered ON\n");
	}

#ifdef CONFIG_EXTCON_PTN5150
	extcon_ptn5150_setup(&usb_ptn5150);
#endif
	return 0;
}

int board_late_init(void)
{
	struct var_eeprom *ep = &eeprom;
	struct var_carrier_eeprom carrier_eeprom;
	char som_rev[CARRIER_REV_LEN] = {0};
	char carrier_rev[CARRIER_REV_LEN] = {0};
	char carrier_name[CARRIER_REV_LEN] = {0};

#ifdef CONFIG_ENV_IS_IN_MMC
	board_late_mmc_env_init();
#endif

	env_set("sec_boot", "no");
#ifdef CONFIG_AHAB_BOOT
	env_set("sec_boot", "yes");
#endif
	/* Read EEPROM data */
	var_eeprom_read_header(ep);

	/* SoM Features ENV */
	env_set("som_has_wbe", (ep->features & VAR_EEPROM_F_WBE) ? "1" : "0");

	/* SoM Rev and Board name ENV */
	snprintf(som_rev, CARRIER_REV_LEN, "%ld.%ld", SOMREV_MAJOR(ep->somrev), SOMREV_MINOR(ep->somrev));
	env_set("som_rev", som_rev);
	env_set("board_name", "DART-MX95");

	/* Carrier Rev ENV */
	var_carrier_eeprom_read(CARRIER_EEPROM_I2C_NAME, CARRIER_EEPROM_ADDR, &carrier_eeprom);
	var_carrier_eeprom_get_revision(&carrier_eeprom, carrier_rev, sizeof(carrier_rev), VAR_DART);
	env_set("carrier_rev", carrier_rev);

	if (var_carrier_eeprom_get_name(&carrier_eeprom, carrier_name, VAR_DART) > 0)
		env_set("carrier_name", carrier_name);

	/* To avoid U-Boot crash running Cortex M7 demos */
	if (m7_is_powered == false) {
		printf ("Force use_m7=no because Cortex-M7 is not powered");
		env_set("use_m7", "no");
		env_set("m7_dtb_suffix", "");
	}
	else
		env_set("m7_dtb_suffix", "-m7");

	var_setup_mac(ep);

	var_eeprom_print_prod_info(ep);

	board_sm_cfg_info();

	return 0;
}

#ifdef CONFIG_OF_BOARD_SETUP
int board_fix_fdt_eeprom(void *fdt)
{
	struct var_eeprom *ep = &eeprom;
	int node_offset;
	int ret;

	if (!var_eeprom_is_valid(ep)) {
		printf("%s: Invalid EEPROM data\n", __func__);
		return -EINVAL;
	}

	if (!fdt) {
		printf("%s: Error: fdt is NULL\n", __func__);
		return -EINVAL;
	}

	/* Disable WiFi/BT nodes if module absent */
	if (!(ep->features & VAR_EEPROM_F_WIFI)) {
		/* Disable WiFi USDHC3 node */
		node_offset = fdt_path_offset(fdt, "/soc/bus@42800000/mmc@428b0000");
		if (node_offset < 0) {
			debug("%s: Could not find usdhc3 (WiFi) node in device tree: %s\n",
				__func__, fdt_strerror(node_offset));
			return 0;
		}

		ret = fdt_setprop_string(fdt, node_offset, "status", "disabled");
		if (ret < 0) {
			printf("%s: Failed to disable usdhc3 (WiFi) node: %s\n",
				__func__, fdt_strerror(ret));
			return ret;
		}

		/* Disable BT node */
		node_offset = fdt_path_offset(fdt, "/soc/bus@42000000/serial@42590000/bluetooth");
		if (node_offset < 0) {
			debug("%s: Could not find bluetooth node in device tree: %s\n",
				__func__, fdt_strerror(node_offset));
			return 0;
		}

		ret = fdt_setprop_string(fdt, node_offset, "status", "disabled");
		if (ret < 0) {
			printf("%s: Failed to disable bluetooth node: %s\n",
				__func__, fdt_strerror(ret));
			return ret;
		}
	}

	return 0;
}

int ft_board_setup(void *blob, struct bd_info *bd)
{
	(void)bd;
	board_fix_fdt_eeprom(blob);

	return 0;
}
#endif

int board_phys_sdram_size(phys_size_t *size)
{
	*size = PHYS_SDRAM_SIZE + PHYS_SDRAM_2_SIZE;

	return 0;
}

void board_quiesce_devices(void)
{
	int ret;

	ret = imx9_scmi_power_domain_enable(IMX95_PD_HSIO_TOP, false);
	if (ret) {
		printf("%s: Failed for HSIO MIX: %d\n", __func__, ret);
		return;
	}

	ret = imx9_scmi_power_domain_enable(IMX95_PD_NETC, false);
	if (ret) {
		printf("%s: Failed for NETC MIX: %d\n", __func__, ret);
		return;
	}
}

static int imx9_scmi_misc_cfginfo(u32 *msel, char *cfgname)
{
	struct scmi_cfg_info_out out;
	struct scmi_msg msg = {
		.protocol_id = SCMI_PROTOCOL_ID_IMX_MISC,
		.message_id = SCMI_MISC_CFG_INFO,
		.in_msg = (u8 *)NULL,
		.in_msg_sz = 0,
		.out_msg = (u8 *)&out,
		.out_msg_sz = sizeof(out),
	};
	int ret;
	struct udevice *dev;

	ret = uclass_get_device_by_name(UCLASS_CLK, "protocol@14", &dev);
	if (ret)
		return ret;

	ret = devm_scmi_process_msg(dev, &msg);
	if(ret == 0 && out.status == 0) {
		strcpy(cfgname, (const char *)out.cfgname);
	} else {
		printf("Failed to get cfg name, scmi_err = %d\n",
			out.status);
		return -EINVAL;
	}

	return 0;
}

static int board_sm_cfg_info(void)
{
	int ret;
	u32 msel = 0;
	char cfgname[SCMI_MISC_MAX_CFGNAME];

	ret = imx9_scmi_misc_cfginfo(&msel, cfgname);
	if (ret)
		snprintf(cfgname, sizeof(cfgname), "Unknown");

	printf("SM: cfg: %s, msel: %u\n", cfgname, msel);
	return 0;
}

#if IS_ENABLED(CONFIG_OF_BOARD_FIXUP)
int board_fix_fdt(void *fdt)
{
	/* Remove nodes based on fuses. */
	board_fix_fdt_fuse(fdt);
	
	return 0;
}
#endif /*CONFIG_OF_BOARD_FIXUP*/

#ifdef CONFIG_FSL_FASTBOOT
#ifdef CONFIG_ANDROID_RECOVERY
int is_recovery_key_pressing(void)
{
	return 0;
}
#endif /*CONFIG_ANDROID_RECOVERY*/
#endif /*CONFIG_FSL_FASTBOOT*/
