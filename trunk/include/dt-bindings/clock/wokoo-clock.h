/* SPDX-License-Identifier: GPL-2.0
 *
 * Copyright (c) 2020 Codefair Semiconductor Technology Co., Ltd.
 *		http://www.codefairsemi.com 
 *
 */

#ifndef __DT_BINDINGS_CLOCK_WOKOO_H
#define __DT_BINDINGS_CLOCK_WOKOO_H

#define WOKOO_CLK_DUMMY             0
#define WOKOO_CLK_OSC32K            1
#define WOKOO_CLK_OSC16M            2
#define WOKOO_CLK_OSC50M            3
/* For pll out */
#define WOKOO_CLK_PLLOUT            4
/* For timer, lcdc, clkout */
#define WOKOO_CLK_PLL2              5
/* For cphy, dsi, pwm */
#define WOKOO_CLK_PLL3              6

/* Clock source is pll out */
#define WOKOO_SCLK_A7               7
#define WOKOO_SCLK_A7_AXI           8
#define WOKOO_MCLK_BUS              9
#define WOKOO_MCLK_PLL              10
#define WOKOO_MCLK_PLL_DIV          11
#define WOKOO_MCLK_ARITH            12
#define WOKOO_HCLK_ARITH            13
#define WOKOO_CLK_DDR_AXI           14
#define WOKOO_CLK_ISP_AXI           15
#define WOKOO_SCLK_ISP_P            16
#define WOKOO_SCLK_ISP_2            17
#define WOKOO_SCLK_ATB              18
#define WOKOO_SCLK_CS_CFG           19
#define WOKOO_SCLK_CS_CTM           20
#define WOKOO_SCLK_UAC_SUB          21
#define WOKOO_SCLK_LCDC_AXI         22

/* Clock source is A7_CLK */
#define WOKOO_PCLK_A7_DBG           23
#define WOKOO_ATCLK_A7              24
#define WOKOO_ACLKS_A7_ADB          25
#define WOKOO_SCLK_A7_TSP           26
#define WOKOO_CLK_A7_SYNC           27

/* Clock source is BUS_MCLK */
#define WOKOO_MCLK_BUS_SAME         28
#define WOKOO_PCLK_APB              29
#define WOKOO_HCLK_ISP              30

/* Clock source is PLL_MCLK */
#define WOKOO_SCLK_LCDC0            32
#define WOKOO_SCLK_I2C0             36
#define WOKOO_SCLK_I2C1             37
#define WOKOO_SCLK_I2S0             45
#define WOKOO_SCLK_I2S1             46
#define WOKOO_SCLK_SSI0             47
#define WOKOO_SCLK_SSI1             48
#define WOKOO_SCLK_UART0            54
#define WOKOO_SCLK_UART1            55
#define WOKOO_SCLK_UART2            56
#define WOKOO_SCLK_UAC_RNG_SLOW     57
#define WOKOO_SCLK_USB_12M          58


/* Clock source is OSC16M */
#define WOKOO_SCLK_DSI_CFG          60
#define WOKOO_SCLK_DSI_REF          61
#define WOKOO_SCLK_CPHY_CFG         62
#define WOKOO_SCLK_PWM              63
#define WOKOO_MCLK_FUNC             64

/* CTL_PCLK */
#define WOKOO_PCLK_CTL_IDLE         65
#define WOKOO_PCLK_CTL_WORK         66

#define WOKOO_CLK_DDR_AXI_IDLE      67
#define WOKOO_SCLK_I2S0_GR          68
#define WOKOO_SCLK_SSI0_GR          69
#define WOKOO_SCLK_SSI1_GR          70
#define WOKOO_SCLK_UART0_GR         71
#define WOKOO_SCLK_UART1_GR         72
#define WOKOO_SCLK_UART2_GR         73
#define WOKOO_SCLK_I2S1_GR          74

/* Other clock source */
/* Clock source is DDR_AXI_CLK */
#define WOKOO_ACLK_DDR_GPV          80
/* Clock source is BUS_MCLK_SAME */
#define WOKOO_SCLK_USBOTG_12M       81
/* Clock source is ATB_CLK */
#define WOKOO_PCLK_DBG              82
/* Clock source is select able */
#define WOKOO_SCLK_OUT1             100
#define WOKOO_SCLK_OUT2             101
#define WOKOO_SCLK_OUT3             102

/* A7_AXI  clock control, WOKOO_SCLK_A7_AXI enable / disable */
#define WOKOO_CLK_A7_AXI_GPV        126

/* System clock control : enable or disable */
#define WOKOO_HCLK_CTL              127
#define WOKOO_CLK_AP_DMAG           128
#define WOKOO_CLK_AP_DMA_BUS        129
#define WOKOO_CLK_AXI_AHB           130
#define WOKOO_CLK_DMA_AXI           131
#define WOKOO_CLK_PERI_BUS_GPV      132
#define WOKOO_CLK_PERI_BUS          133

#define WOKOO_ACLK_A7_AXI_PERI      134
#define WOKOO_HCLK_PBR              135
#define WOKOO_HCLK_UAC              136
#define WOKOO_HCLK_LCDC             137
#define WOKOO_HCLK_BOOTROM          138
#define WOKOO_HCLK_CIPHER           139
#define WOKOO_HCLK_SEC_RAM          140
#define WOKOO_HCLK_SECURITY         141
#define WOKOO_HCLK_USBOTG           142

#define WOKOO_CLK_ISP_BUS_LP        143
#define WOKOO_CLK_DDR_AXI_BUS_LP    144
#define WOKOO_CLK_A7_AXI_BUS_LP     145
#define WOKOO_CLK_SYS_BUS_LP        146

/* UAC clock control */
#define WOKOO_CLK_UAC_RNG_SLOW      147
#define WOKOO_CLK_UAC_SM4           148
#define WOKOO_CLK_UAC_SM3           149
#define WOKOO_CLK_UAC_SM1           150
#define WOKOO_CLK_UAC_PKI           151
#define WOKOO_CLK_UAC_HRNG          152
#define WOKOO_CLK_UAC_DES           153
#define WOKOO_CLK_UAC_ALG           154
#define WOKOO_CLK_UAC_AES           155

/* ARITH clock control */
#define WOKOO_CLK_ARITH             156

/* USB clock control */
#define WOKOO_CLK_USB_PRE           157
#define WOKOO_CLK_USBOTG_12M        158

/* LCDC clock control */
#define WOKOO_CLK_LCDC_AXI          159
#define WOKOO_MCLK_LCDC0            160

/* ISP clock control */
#define WOKOO_CLK_CPHY_CFG          161
#define WOKOO_SCLK_LVDS_IDSP        162
#define WOKOO_HCLK_ISP_EN           163
#define WOKOO_SCLK_ISP_P_EN         164

/* APB bus clock control0 */
#define WOKOO_PCLK_BP147            165
#define WOKOO_PCLK_RTC              166
#define WOKOO_PCLK_TPZCTL           167
#define WOKOO_PCLK_QSPI             168
#define WOKOO_PCLK_GPIO0            169
#define WOKOO_PCLK_I2S1             170
#define WOKOO_PCLK_TIMER            171
#define WOKOO_PCLK_WDT0             172
#define WOKOO_PCLK_PWM              173
#define WOKOO_PCLK_UART0            174
#define WOKOO_PCLK_UART1            175
#define WOKOO_PCLK_UART2            176
#define WOKOO_PCLK_SSI0             177
#define WOKOO_PCLK_I2C0             178
#define WOKOO_PCLK_I2C1             179
#define WOKOO_PCLK_I2S0             180
/* APB bus clock control1 */
#define WOKOO_PCLK_ISP              181
#define WOKOO_PCLK_DDR_PWR          182
#define WOKOO_PCLK_MUXPIN           183
#define WOKOO_PCLK_MCTL_DDR         184
#define WOKOO_PCLK_MPHY_DDR         185
/* APB working clock control */
#define WOKOO_MCLK_PWM              186
#define WOKOO_MCLK_I2C0             187
#define WOKOO_MCLK_I2C1             188

#define WOKOO_CLK_ATB               189
#define WOKOO_CLK_DBG               190

/* TIMER working clock control */
#define WOKOO_CLK_TIMER0_SEL        191 
#define WOKOO_SCLK_TIMER0_FUNC      192 
#define WOKOO_SCLK_TIMER0_PBUS      193 
#define WOKOO_SCLK_TIMER0_PLL       194 
#define WOKOO_SCLK_TIMER0_BUS       195 
#define WOKOO_CLK_TIMER1_SEL        196 
#define WOKOO_SCLK_TIMER1_FUNC      197 
#define WOKOO_SCLK_TIMER1_PBUS      198 
#define WOKOO_SCLK_TIMER1_PLL       199 
#define WOKOO_SCLK_TIMER1_BUS       200 
#define WOKOO_CLK_TIMER2_SEL        201 
#define WOKOO_SCLK_TIMER2_FUNC      202 
#define WOKOO_SCLK_TIMER2_PBUS      203 
#define WOKOO_SCLK_TIMER2_PLL       204 
#define WOKOO_SCLK_TIMER2_BUS       205 
#define WOKOO_CLK_TIMER3_SEL        206 
#define WOKOO_SCLK_TIMER3_FUNC      207 
#define WOKOO_SCLK_TIMER3_PBUS      208 
#define WOKOO_SCLK_TIMER3_PLL       209 
#define WOKOO_SCLK_TIMER3_BUS       210 
#define WOKOO_CLK_TIMER4_SEL        211 
#define WOKOO_SCLK_TIMER4_FUNC      212 
#define WOKOO_SCLK_TIMER4_PBUS      213 
#define WOKOO_SCLK_TIMER4_PLL       214 
#define WOKOO_SCLK_TIMER4_BUS       215 
#define WOKOO_CLK_TIMER5_SEL        216 
#define WOKOO_SCLK_TIMER5_FUNC      217 
#define WOKOO_SCLK_TIMER5_PBUS      218 
#define WOKOO_SCLK_TIMER5_PLL       219 
#define WOKOO_SCLK_TIMER5_BUS       220 
#define WOKOO_CLK_TIMER6_SEL        221 
#define WOKOO_SCLK_TIMER6_FUNC      222 
#define WOKOO_SCLK_TIMER6_PBUS      223 
#define WOKOO_SCLK_TIMER6_PLL       224
#define WOKOO_SCLK_TIMER6_BUS       225

/* Clock out */
#define WOKOO_CLK_OUT1_SEL          240
#define WOKOO_CLK_OUT1_32K          241
#define WOKOO_CLK_OUT1_FUNC         242
#define WOKOO_CLK_OUT1_PLL_DIV0     243
#define WOKOO_CLK_OUT1_PLL          244
#define WOKOO_CLK_OUT1_BUS          245
#define WOKOO_CLK_OUT2_SEL          246
#define WOKOO_CLK_OUT2_32K          247
#define WOKOO_CLK_OUT2_FUNC         248
#define WOKOO_CLK_OUT2_PLL_DIV0     249
#define WOKOO_CLK_OUT2_PLL          250
#define WOKOO_CLK_OUT2_BUS          251
#define WOKOO_CLK_OUT3_SEL          252
#define WOKOO_CLK_OUT3_32K          253
#define WOKOO_CLK_OUT3_FUNC         254
#define WOKOO_CLK_OUT3_PLL_DIV0     255
#define WOKOO_CLK_OUT3_PLL          256
#define WOKOO_CLK_OUT3_BUS          257

#define WOKOO_CLK_END               257

/* Cortex-A7 reset register, offset = 0x184, (32 * 0 + x) */
#define WOKOO_RST_A7C0_PO           0
#define WOKOO_RST_A7C0              4
#define WOKOO_RST_A7_DBG0           8
#define WOKOO_RST_A7_DBG1           9
#define WOKOO_RST_A7_ETM0           12
#define WOKOO_RST_A7_L2             16
#define WOKOO_RST_A7_SOCDBG         17

/* ARITH reset register, offset = 0x190, (32 * 3 + x) */
#define WOKOO_RST_ARITH_HRESET      100

/* ARITH reset register, offset = 0x194, (32 * 4 + x) */
#define WOKOO_RST_ISP_AHB           128
#define WOKOO_RST_ISP_P             129

/* DISPLAY reset register, offset = 0x198, (32 * 5 + x) */
#define WOKOO_RST_DISPLAY           160

/* COM_APB reset register, offset = 0x19C, (32 * 6 + x) */
#define WOKOO_RST_COM_CLR_MD        144
#define WOKOO_RST_COM               208

/* Modules reset register 0, offset = 0x1A4, (32 * 8 + x) */
#define WOKOO_RST_PERI_AXI          256
#define WOKOO_RST_ACP_AXI           257
#define WOKOO_RST_DMA_AXI           258
#define WOKOO_RST_DDR_AXI           259
#define WOKOO_RST_AXI_AHB           263
#define WOKOO_RST_AP_DMA_BUS        264
#define WOKOO_RST_AP_DMAG           265
#define WOKOO_RST_A7_AXI_MAIN       266
#define WOKOO_RST_A7_AXI_GPV        267

/* Modules reset register 1, offset = 0x1A8, (32 * 9 + x) */
#define WOKOO_RST_CTL               289
#define WOKOO_RST_NFC               290
#define WOKOO_RST_USBOTG            293
#define WOKOO_RST_USBOTG_P          294
#define WOKOO_RST_AP_SEC_RAM        299
#define WOKOO_RST_CIPHER            300
#define WOKOO_RST_BOOTROM           301
#define WOKOO_RST_A7_AXI_PERI       302
#define WOKOO_RST_UAC               303

/* Modules reset register 2, offset = 0x1AC, (32 * 10 + x) */
#define WOKOO_RST_RTC               321
#define WOKOO_RST_TIMER0            322
#define WOKOO_RST_TIMER1            323
#define WOKOO_RST_TIMER2            324
#define WOKOO_RST_TIMER3            325
#define WOKOO_RST_I2C0              326
#define WOKOO_RST_I2C1              327
#define WOKOO_RST_GPIO0             328
#define WOKOO_RST_PWM               329
#define WOKOO_RST_TIMER4            330
#define WOKOO_RST_TIMER5            331
#define WOKOO_RST_TIMER6            332
#define WOKOO_RST_WDT0              333

/* Modules reset register 3, offset = 0x1B0, (32 * 11 + x) */
#define WOKOO_RST_I2S0              353
#define WOKOO_RST_I2S1              354
#define WOKOO_RST_SSI0              355
#define WOKOO_RST_UART0             358
#define WOKOO_RST_UART1             359
#define WOKOO_RST_UART2             360

/* Modules reset register 4, offset = 0x1B4, (32 * 12 + x) */
#define WOKOO_RST_MUXPIN            385
#define WOKOO_RST_QSPI              387
#define WOKOO_RST_BP147             388
#define WOKOO_RST_TPZCTL            389

/* CoreSight reset register, offset = 0x1B8, (32 * 13 + x) */
#define WOKOO_RST_CS_ATB            416
#define WOKOO_RST_CS_DBG_PM         417
#define WOKOO_RST_CS_DBG_PS         418
#define WOKOO_RST_CS_AHB            419
#define WOKOO_RST_CS_CTM            420


#endif /* __DT_BINDINGS_CLOCK_WOKOO_H */
