/*
 * Copyright 2019-2020 Variscite Ltd.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <env.h>
#include <dm.h>
#include <i2c.h>
#include <spl.h>
#include <image.h>
#include <init.h>
#include <log.h>
#include <dm/uclass.h>
#include <dm/device.h>
#include <dm/uclass-internal.h>
#include <dm/device-internal.h>
#include <dm/lists.h>
#include <bootm.h>
#include <asm/gpio.h>
#include <asm/arch/sci/sci.h>
#include <asm/arch/imx8-pins.h>
#include <asm/arch/iomux.h>

#include "../common/imx8_eeprom.h"
#include "imx8qm_var_som.h"

DECLARE_GLOBAL_DATA_PTR;

#define GPIO_PAD_CTRL	((SC_PAD_CONFIG_NORMAL << PADRING_CONFIG_SHIFT) | \
			(SC_PAD_ISO_OFF << PADRING_LPCONFIG_SHIFT) | \
			(SC_PAD_28FDSOI_DSE_DV_LOW << PADRING_DSE_SHIFT) | \
			(SC_PAD_28FDSOI_PS_PU << PADRING_PULL_SHIFT))

static iomux_cfg_t usdhc2_sd_pwr[] = {
	SC_P_USDHC1_RESET_B | MUX_PAD_CTRL(GPIO_PAD_CTRL),
};

static struct var_eeprom eeprom = {0};

void spl_board_init(void)
{
	struct var_eeprom *ep = VAR_EEPROM_DATA;
	struct udevice *dev;
	int node, ret;

	node = fdt_node_offset_by_compatible(gd->fdt_blob, -1, "fsl,imx8-mu");

	ret = uclass_get_device_by_of_offset(UCLASS_MISC, node, &dev);
	if (ret) {
		return;
	}
	device_probe(dev);

	uclass_find_first_device(UCLASS_MISC, &dev);
	for (; dev; uclass_find_next_device(&dev)) {
		if (device_probe(dev))
			continue;
	}

	board_early_init_f();

	timer_init();

#ifdef CONFIG_SPL_SERIAL_SUPPORT
	preloader_console_init();
#endif

	memset(ep, 0, sizeof(*ep));
	if (!var_scu_eeprom_read_header(&eeprom)) {
		/* Copy EEPROM contents to DRAM */
		memcpy(ep, &eeprom, sizeof(*ep));

		/*
		 * The pad configuration is need to avoid failure when we do reboot from uSD
		 * with SP8CustomBoard. The GPIO4,7 used to reset the uSD go low at reboot so when
		 * SPL try to access uSD fails. The following error messages are printed:
		 *
		 * U-Boot SPL 2021.04-lf_v2021.04_var03+ga36a857455 (Mar 13 2022 - 11:33:18 +0000)
		 * Normal Boot
		 * Trying to boot from MMC2_2
		 * Card did not respond to voltage select! : -110
		 * spl: mmc init failed with error: -95
		 * SPL: failed to boot from all boot devices
		 * ### ERROR ### Please RESET the board ###
		 *
		 * To fix the issue we configure the pad like device tree configuration (0x00000021);
		 */
		if (var_get_board_id(ep) == SPEAR_MX8)
			imx8_iomux_setup_multiple_pads(usdhc2_sd_pwr, ARRAY_SIZE(usdhc2_sd_pwr));
	}
#ifdef CONFIG_SPL_SERIAL_SUPPORT
	puts("Normal Boot\n");
#endif
}

void spl_board_prepare_for_boot(void)
{
	board_quiesce_devices();
}

int board_fit_config_name_match(const char *name)
{
	return 0;
}

void board_init_f(ulong dummy)
{
	/* Clear the BSS. */
	memset(__bss_start, 0, __bss_end - __bss_start);

	arch_cpu_init();

	board_init_r(NULL, 0);
}
