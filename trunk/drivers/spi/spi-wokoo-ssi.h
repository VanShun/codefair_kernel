// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2020 Codefair Semiconductor Technology Co., Ltd.
 *		http://www.codefairsemi.com
 *
 */


#ifndef SPI_WOKOO_SSI_H_
#define SPI_WOKOO_SSI_H_


#define WOKOO_SSI0_CTRL0			0x00	/*Control Register 0*/
#define WOKOO_SSI0_CTRL1			0x04	/*Control Register 1*/
#define WOKOO_SSI0_EN				0x08 	/*SSI Enable Register*/

#define WOKOO_SSI0_SE				0x10 	/*Slave Enable Register*/
#define WOKOO_SSI0_BAUD				0x14	/*Baud Rate Select*/
#define WOKOO_SSI0_TXFTL			0x18	/*Transmit FIFO Threshold Level*/
#define WOKOO_SSI0_RXFTL			0x1C	/*Receive FIFO Threshold Level*/
#define WOKOO_SSI0_TXFL				0x20	/*Transmit FIFO Level Register*/
#define WOKOO_SSI0_RXFL				0x24	/*Receive FIFO Level Register*/
#define WOKOO_SSI0_STS				0x28	/*Status Register*/
#define WOKOO_SSI0_IE				0x2C 	/*Interrupt Mask Register*/
#define WOKOO_SSI0_IS				0x30 	/*Interrupt Status Register*/
#define WOKOO_SSI0_RIS				0x34	/*Raw Interrupt Status Register*/
#define WOKOO_SSI0_TXOIC			0x38	/*Transmit FIFO Overflow Interrupt Clear Register Register*/
#define WOKOO_SSI0_RXOIC			0x3C	/*Receive FIFO Overflow Interrupt Clear Register*/
#define WOKOO_SSI0_RXUIC			0x40	/*Receive FIFO Underflow Interrupt Clear Register*/

#define WOKOO_SSI0_IC				0x48 	/*Interrupt Clear Register*/
#define WOKOO_SSI0_DMAC				0x4C	/*DMA Control Register*/
#define WOKOO_SSI0_DMATDLR			0x50	/*DMA Transmit Data Level*/
#define WOKOO_SSI0_DMARDLR			0x54	/*DMA Receive Data Level*/

#define WOKOO_SSI0_DATA				0x60	/*Data Register*/
#define WOKOO_SSI0_DATA1			0x64	/*Data Register*/
#define WOKOO_SSI0_DATA2			0x68	/*Data Register*/
#define WOKOO_SSI0_DATA3			0x6C	/*Data Register*/


#define WOKOO_SSI0_CTRL0_DFS_POS			(0)
#define WOKOO_SSI0_CTRL0_DFS_MSK			(0xFUL << WOKOO_SSI0_CTRL0_DFS_POS)
#define WOKOO_SSI0_CTRL0_FRF_POS			(4)
#define WOKOO_SSI0_CTRL0_FRF_MSK			(0x3UL << WOKOO_SSI0_CTRL0_FRF_POS)
#define WOKOO_SSI0_CTRL0_SCPH				(0x1UL << 6)
#define WOKOO_SSI0_CTRL0_SCPOL				(0x1UL << 7)
#define WOKOO_SSI0_CTRL0_TMOD_POS			(8)
#define WOKOO_SSI0_CTRL0_TMOD_MSK			(0x3UL << WOKOO_SSI0_CTRL0_TMOD_POS)
#define WOKOO_SSI0_CTRL0_SLV_OE				(0x1UL << 10)
#define WOKOO_SSI0_CTRL0_SRL				(0x1UL << 11)
#define WOKOO_SSI0_CTRL0_CFS_POS			(12)
#define WOKOO_SSI0_CTRL0_CFS_MSK			(0xFUL << WOKOO_SSI0_CTRL0_CFS_POS)

#define WOKOO_SSI0_CTRL1_NDF_POS			(0)
#define WOKOO_SSI0_CTRL1_NDF_MSK			(0xFFFFUL << WOKOO_SSI0_CTRL1_NDF_POS)

#define WOKOO_SSI0_EN_SEN					(0x1UL << 0)

#define WOKOO_SSI0_SE_SS0					(0x1UL << 0)

#define WOKOO_SSI0_BAUD_SCKDV_POS			(0)
#define WOKOO_SSI0_BAUD_SCKDV_MSK			(0xFFFFUL << WOKOO_SSI0_BAUD_SCKDV_POS)

#define WOKOO_SSI0_TXFTL_TFT_MSK			(0xFUL)

#define WOKOO_SSI0_RXFTL_RFT_MSK			(0xFUL)

#define WOKOO_SSI0_TXFL_TXTFL_MSK			(0x1FUL)

#define WOKOO_SSI0_RXFL_RXTFL_MSK			(0x1FUL)

#define WOKOO_SSI0_STS_BUSY					(0x1UL << 0)
#define WOKOO_SSI0_STS_TFNF					(0x1UL << 1)
#define WOKOO_SSI0_STS_TFE					(0x1UL << 2)
#define WOKOO_SSI0_STS_RFNE					(0x1UL << 3)
#define WOKOO_SSI0_STS_RFF					(0x1UL << 4)

#define WOKOO_SSI0_IE_TXEIE					(0x1UL << 0)
#define WOKOO_SSI0_IE_TXOIE					(0x1UL << 1)
#define WOKOO_SSI0_IE_RXUIE					(0x1UL << 2)
#define WOKOO_SSI0_IE_RXOIE					(0x1UL << 3)
#define WOKOO_SSI0_IE_RXFIE					(0x1UL << 4)

#define WOKOO_SSI0_IS_TXEIS					(0x1UL << 0)
#define WOKOO_SSI0_IS_TXOIS					(0x1UL << 1)
#define WOKOO_SSI0_IS_RXUIS					(0x1UL << 2)
#define WOKOO_SSI0_IS_RXOIS					(0x1UL << 3)
#define WOKOO_SSI0_IS_RXFIS					(0x1UL << 4)

#define WOKOO_SSI0_RIS_TXEIR				(0x1UL << 0)
#define WOKOO_SSI0_RIS_TXOIR				(0x1UL << 1)
#define WOKOO_SSI0_RIS_RXUIR				(0x1UL << 2)
#define WOKOO_SSI0_RIS_RXOIR				(0x1UL << 3)
#define WOKOO_SSI0_RIS_RXFIR				(0x1UL << 4)

#define WOKOO_SSI0_TXOIC_TXOIC				(0x1Ul << 0)
#define WOKOO_SSI0_RXOIC_RXOIC				(0x1UL << 0)
#define WOKOO_SSI0_RXUIC_RXUIC				(0x1UL << 0)

#define WOKOO_SSI0_IC_IC					(0x1UL << 0)
#define WOKOO_SSI0_DMAC_RDMAE				(0x1UL << 0)
#define WOKOO_SSI0_DMAC_TDMAE				(0x1UL << 1)

#define WOKOO_SSI0_DMATDL_DMATDL_MSK		(0xFUL)

#define WOKOO_SSI0_DMARDL_DMARDL_MSK		(0xFUL)

#define WOKOO_SSI0_CTRL0_TMOD_TXRX			(0x0UL << WOKOO_SSI0_CTRL0_TMOD_POS)
#define WOKOO_SSI0_CTRL0_TMOD_TX			(0x1UL << WOKOO_SSI0_CTRL0_TMOD_POS)
#define WOKOO_SSI0_CTRL0_TMOD_RX			(0x2UL << WOKOO_SSI0_CTRL0_TMOD_POS)
#define WOKOO_SSI0_CTRL0_TMOD_ROM_RD		(0x3UL << WOKOO_SSI0_CTRL0_TMOD_POS)

#define SSI0_CTRL0_FRF_MOTOROLA_SPI			(0x0UL)
#define SSI0_CTRL0_FRF_TI_SSP				(0x1UL)
#define SSI0_SE_SLAVE_0 					(0x1UL << 0)
#define SSI0_SE_SLAVE_1 					(0x1UL << 1)




#endif
