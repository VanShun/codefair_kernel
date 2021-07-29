/* SPDX-License-Identifier: GPL-2.0-only */
/* Debugging macro include header
 *
 *  Copyright (c) 2020 Codefair Semiconductor Technology Co., Ltd.
 * 
 */
		.macro	addruart, rp, rv, tmp
		ldr	\rp, = CONFIG_DEBUG_UART_PHYS
		ldr	\rv, = CONFIG_DEBUG_UART_VIRT
		.endm

		.macro	waituart,rd,rx
		.endm

		.macro	senduart,rd,rx
		ARM_BE8(rev \rd, \rd)
		str	\rd, [\rx, #0x00]	@ TXDATA
		.endm

		.macro	busyuart,rd,rx
1002:	ldr	\rd, [\rx, #0x14]	@ STAT
		ARM_BE8(rev \rd, \rd)
		tst	\rd, #1 << 5		@ TXEMPTY
		beq	1002b			@ wait until transmit done
		.endm
