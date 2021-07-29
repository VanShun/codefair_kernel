/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright Everest Semiconductor Co.,Ltd
 *
 */

#ifndef _ES8311_H
#define _ES8311_H

/*
 * ES8311 register space
 */

/* Reset Control */
#define ES8311_RESET			0x00

/* Clock Management */
#define ES8311_CLKMGR_CLKSW		0x01
#define ES8311_CLKMGR_CLKSEL	0x02
#define ES8311_CLKMGR_ADCOSR	0x03

/* Serial Data Port Control */
#define ES8311_SERDATA_DAC		0x09
#define ES8311_SERDATA_ADC		0x0a



/* System Control */

#define ES8311_SYS_LP1			0x0d
#define ES8311_SYS_LP2			0x0e

#define ES8311_SYS_VMIDLOW		0x10
#define ES8311_SYS_VSEL			0x11
#define ES8311_SYS_REF			0x12
#define ES8311_HPMIX_SEL		0x13

/* ADC Control */
#define ES8311_ADC_VOLUME		0x17

/* DAC Control */
#define ES8311_DAC_VOLUME		0x32

#define ES8311_GPIO_FLAG		0x4f

/*
 * Field definitions
 */

/* ES8311_RESET */
#define ES8311_RESET_CSM_ON			0x80

/* ES8311_CLKMGR_CLKSW */
#define ES8311_CLKMGR_CLKSW_MCLK_ON	0x20
#define ES8311_CLKMGR_CLKSW_BCLK_ON	0x10

/* ES8311_SERDATA1 */
#define ES8311_SERDATA1_BCLK_INV	0x20

/* ES8311_SERDATA_ADC and _DAC */
#define ES8311_SERDATA2_FMT_MASK	0x3
#define ES8311_SERDATA2_FMT_I2S		0x00
#define ES8311_SERDATA2_FMT_LEFTJ	0x01
#define ES8311_SERDATA2_FMT_RIGHTJ	0x02
#define ES8311_SERDATA2_FMT_PCM		0x03
#define ES8311_SERDATA2_ADCLRP		0x20
#define ES8311_SERDATA2_LEN_MASK	0x1c
#define ES8311_SERDATA2_LEN_24		0x00
#define ES8311_SERDATA2_LEN_20		0x04
#define ES8311_SERDATA2_LEN_18		0x08
#define ES8311_SERDATA2_LEN_16		0x0c
#define ES8311_SERDATA2_LEN_32		0x10

#endif
