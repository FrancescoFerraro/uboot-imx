/*
 * Copyright (C) 2015-2016 Freescale Semiconductor, Inc.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <config.h>

/* DDR script */
.macro imx7d_ddrphy_latency_setting
	ldr r2, =ANATOP_BASE_ADDR
	ldr r3, [r2, #0x800]
	and r3, r3, #0xFF
	cmp r3, #0x11
	bne NO_DELAY

	/*TO 1.1*/
	ldr r1, =0x00000dee
	str r1, [r0, #0x9c]
	ldr r1, =0x18181818
	str r1, [r0, #0x7c]
	ldr r1, =0x18181818
	str r1, [r0, #0x80]
	ldr r1, =0x40401818
	str r1, [r0, #0x84]
	ldr r1, =0x00000040
	str r1, [r0, #0x88]
	ldr r1, =0x40404040
	str r1, [r0, #0x6c]
	b TUNE_END

NO_DELAY:
	/*TO 1.0*/
	ldr r1, =0x00000b24
	str r1, [r0, #0x9c]

TUNE_END:
.endm

.macro imx7d_ddr_freq_setting
	ldr r2, =ANATOP_BASE_ADDR
	ldr r3, [r2, #0x800]
	and r3, r3, #0xFF
	cmp r3, #0x11
	bne FREQ_DEFAULT_533

	/* Change to 400Mhz for TO1.1 */
	ldr r0, =ANATOP_BASE_ADDR
	ldr r1, =0x70
	ldr r2, =0x00703021
	str r2, [r0, r1]
	ldr r1, =0x90
	ldr r2, =0x0
	str r2, [r0, r1]
	ldr r1, =0x70
	ldr r2, =0x00603021
	str r2, [r0, r1]

	ldr r3, =0x80000000
wait_lock:
	ldr r2, [r0, r1]
	and r2, r3
	cmp r2, r3
	bne wait_lock

	ldr r0, =CCM_BASE_ADDR
	ldr r1, =0x9880
	ldr r2, =0x1
	str r2, [r0, r1]

FREQ_DEFAULT_533:
.endm

.macro imx7d_12x12_ddr3_val_ddr_setting
	imx7d_ddr_freq_setting

	/* Configure ocram_epdc */
	ldr r0, =IOMUXC_GPR_BASE_ADDR
	ldr r1, =0x4f400005
	str r1, [r0, #0x4]

	/* clear/set bit30 of SNVS_MISC_CTRL to ensure exit from ddr retention */
	ldr r0, =ANATOP_BASE_ADDR
	ldr r1, =(0x1 << 30)
	str r1, [r0, #0x388]
	str r1, [r0, #0x384]

	ldr r0, =SRC_BASE_ADDR
	ldr r1, =0x2
	ldr r2, =0x1000
	str r1, [r0, r2]

	ldr r0, =DDRC_IPS_BASE_ADDR
	ldr r1, =0x01040001
	str r1, [r0]
	ldr r1, =0x80400003
	str r1, [r0, #0x1a0]
	ldr r1, =0x00100020
	str r1, [r0, #0x1a4]
	ldr r1, =0x80100004
	str r1, [r0, #0x1a8]
	ldr r1, =0x00400046
	str r1, [r0, #0x64]
	ldr r1, =0x1
	str r1, [r0, #0x490]
	ldr r1, =0x00020001
	str r1, [r0, #0xd0]
	ldr r1, =0x00690000
	str r1, [r0, #0xd4]
	ldr r1, =0x09300004
	str r1, [r0, #0xdc]
	ldr r1, =0x04080000
	str r1, [r0, #0xe0]
	ldr r1, =0x00100004
	str r1, [r0, #0xe4]
	ldr r1, =0x33f
	str r1, [r0, #0xf4]
	ldr r1, =0x09081109
	str r1, [r0, #0x100]
	ldr r1, =0x0007020d
	str r1, [r0, #0x104]
	ldr r1, =0x03040407
	str r1, [r0, #0x108]
	ldr r1, =0x00002006
	str r1, [r0, #0x10c]
	ldr r1, =0x04020205
	str r1, [r0, #0x110]
	ldr r1, =0x03030202
	str r1, [r0, #0x114]
	ldr r1, =0x00000803
	str r1, [r0, #0x120]
	ldr r1, =0x00800020
	str r1, [r0, #0x180]
	ldr r1, =0x02000100
	str r1, [r0, #0x184]
	ldr r1, =0x02098204
	str r1, [r0, #0x190]
	ldr r1, =0x00030303
	str r1, [r0, #0x194]

	ldr r1, =0x00000016
	str r1, [r0, #0x200]
	ldr r1, =0x00080808
	str r1, [r0, #0x204]
	ldr r1, =0x00000f0f
	str r1, [r0, #0x210]
	ldr r1, =0x07070707
	str r1, [r0, #0x214]
	ldr r1, =0x0f070707
	str r1, [r0, #0x218]

	ldr r1, =0x06000604
	str r1, [r0, #0x240]
	ldr r1, =0x00000001
	str r1, [r0, #0x244]

	ldr r0, =SRC_BASE_ADDR
	mov r1, #0x0
	ldr r2, =0x1000
	str r1, [r0, r2]

	ldr r0, =DDRPHY_IPS_BASE_ADDR
	ldr r1, =0x17420f40
	str r1, [r0]
	ldr r1, =0x10210100
	str r1, [r0, #0x4]
	ldr r1, =0x00060807
	str r1, [r0, #0x10]
	ldr r1, =0x1010007e
	str r1, [r0, #0xb0]
	imx7d_ddrphy_latency_setting
	ldr r1, =0x08080808
	str r1, [r0, #0x20]
	ldr r1, =0x08080808
	str r1, [r0, #0x30]
	ldr r1, =0x01000010
	str r1, [r0, #0x50]

	ldr r1, =0x0e407304
	str r1, [r0, #0xc0]
	ldr r1, =0x0e447304
	str r1, [r0, #0xc0]
	ldr r1, =0x0e447306
	str r1, [r0, #0xc0]

wait_zq:
	ldr r1, [r0, #0xc4]
	tst r1, #0x1
	beq wait_zq

	ldr r1, =0x0e407304
	str r1, [r0, #0xc0]

	ldr r0, =CCM_BASE_ADDR
	mov r1, #0x0
	ldr r2, =0x4130
	str r1, [r0, r2]
	ldr r0, =IOMUXC_GPR_BASE_ADDR
	mov r1, #0x178
	str r1, [r0, #0x20]
	ldr r0, =CCM_BASE_ADDR
	mov r1, #0x2
	ldr r2, =0x4130
	str r1, [r0, r2]
	ldr r0, =DDRPHY_IPS_BASE_ADDR
	ldr r1, =0x0000000f
	str r1, [r0, #0x18]

	ldr r0, =DDRC_IPS_BASE_ADDR
wait_stat:
	ldr r1, [r0, #0x4]
	tst r1, #0x1
	beq wait_stat
.endm

.macro imx7_clock_gating
.endm

.macro imx7_qos_setting
.endm

.macro imx7_ddr_setting
	imx7d_12x12_ddr3_val_ddr_setting
.endm

/* include the common plugin code here */
#include <asm/arch/mx7_plugin.S>
