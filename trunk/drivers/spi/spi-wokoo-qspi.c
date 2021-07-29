// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2020 Codefair Semiconductor Technology Co., Ltd.
 *		http://www.codefairsemi.com 
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/omap-dma.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/pm_runtime.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/pinctrl/consumer.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/sizes.h>

#include <linux/spi/spi.h>
#include <linux/spi/spi-mem.h>
#include <linux/mtd/spi-nor.h>





struct wokoo_qspi_regs {
	uint32_t CFG;		/*!< Offset: 0x00  QSPI Configuration Register */
	uint32_t DEVRD;		/*!< Offset: 0x04  Device Read Instruction Register */
	uint32_t DEVWR;		/*!< Offset: 0x08  Device Write Instruction Configuration Register */
	uint32_t DELAY;		/*!< Offset: 0x0C  QSPI Device Delay Register */
	uint32_t RDDATACAP;	/*!< Offset: 0x10  Read Data Capture Register */
	uint32_t DEVSZ;		/*!< Offset: 0x14  Device Size Configuration Register */
	uint32_t SRAMPART;	/*!< Offset: 0x18  SRAM Partition Configuration Register */
	uint32_t INDADDRTRIG;/*!< Offset: 0x1C  Indirect AHB Address Trigger register */
	uint32_t DMAPER;	/*!< Offset: 0x20  DMA Peripheral Configuration Register */
	uint32_t REMAPADDR;	/*!< Offset: 0x24  Remap Address Register */
	uint32_t MODEBIT;	/*!< Offset: 0x28  Mode Bit Configuration Register */
	uint32_t SRAMFILL;	/*!< Offset: 0x2C  SRAM Fill Level Register */
	uint32_t TTR;		/*!< Offset: 0x30  TX Threshold Register */
	uint32_t RTR;		/*!< Offset: 0x34  RX Threshold Register */
	uint32_t WCCR;		/*!< Offset: 0x38  Write Completion Control Register */
	uint32_t PER;		/*!< Offset: 0x3C  Polling Expiration Register */
	uint32_t ISR;		/*!< Offset: 0x40  Interrupt Status Register */
	uint32_t IMR;		/*!< Offset: 0x44  Interrupt Mask Register */
	uint32_t Reserved1;	/*!< Offset: 0x48  reserved Register */
	uint32_t Reserved2;	/*!< Offset: 0x4C  reserved Register */
	uint32_t LWPR;		/*!< Offset: 0x50  Lower Write Protection Register */
	uint32_t UWPR;		/*!< Offset: 0x54  Upper Write Protection Register */
	uint32_t WPR;		/*!< Offset: 0x58  Write Protection Register */
	uint32_t Reserved3;	/*!< Offset: 0x5C  reserved Register */
	uint32_t IRTCR;		/*!< Offset: 0x60  Indirect Read Transfer Control Register */
	uint32_t IRTWR;		/*!< Offset: 0x64  Indirect Read Transfer Watermark Register */
	uint32_t IRTSAR;	/*!< Offset: 0x68  Indirect Read Transfer Start Address Register */
	uint32_t IRTNBR;	/*!< Offset: 0x6C  Indirect Read Transfer Number Bytes Register */
	uint32_t IWTR;		/*!< Offset: 0x70  Indirect Write Transfer Control Register */
	uint32_t IWTWR;		/*!< Offset: 0x74  Indirect Write Transfer Watermark Register */
	uint32_t IWTSAR;	/*!< Offset: 0x78  Indirect Write Transfer Start Address Register */
	uint32_t IWTNBR;	/*!< Offset: 0x7C  Indirect Write Transfer Number Bytes Register */
	uint32_t ITARR;		/*!< Offset: 0x80  Indirect Trigger Address Range Register */
	uint32_t Reserved4;	/*!< Offset: 0x84  reserved Register */
	uint32_t Reserved5;	/*!< Offset: 0x88  reserved Register */
	uint32_t Reserved6;	/*!< Offset: 0x8C  reserved Register */
	uint32_t FCR;		/*!< Offset: 0x90  Flash Command Control Register (Using STIG) */
	uint32_t FCAR;		/*!< Offset: 0x94  Flash Command Address Register */
	uint32_t Reserved7; /*!< Offset: 0x98  reserved Register */
	uint32_t Reserved8; /*!< Offset: 0x9C  reserved Register */
	uint32_t FCRDRL;	/*!< Offset: 0xA0  Flash Command Read Data Register (Lower) */
	uint32_t FCRDRU;	/*!< Offset: 0xA4  Flash Command Read Data Register (Upper) */
	uint32_t FCWDRL;	/*!< Offset: 0xA8  Flash Command Write Data Register (Lower) */
	uint32_t FCWDRU;	/*!< Offset: 0xAC  Flash Command Write Data Register (Upper) */
	uint32_t PFSR;		/*!< Offset: 0xB0  Polling Flash Status Register */
	uint32_t Reserved9[18];/*!< Offset: 0xB4~0xF8  reserved Register */
	uint32_t MODULEID;	/*!< Offset: 0xFC  Module ID Register Register */
};

#define QSPI_CFG_ENABLE									(0x1UL << 0)
#define QSPI_CFG_CPOSW									(0x1UL << 1)
#define QSPI_CFG_CP										(0x1UL << 2)
#define QSPI_CFG_PME									(0x1UL << 3)
#define QSPI_CFG_EDAC									(0x1UL << 7)
#define QSPI_CFG_LIME									(0x1UL << 8)
#define QSPI_CFG_PSD									(0x1UL << 9)
#define QSPI_CFG_PCS_POS								(10)
#define QSPI_CFG_PCS_MSK								(0xFUL << QSPI_CFG_PCS_POS)
#define QSPI_CFG_STDTWP									(0x1UL << 14)
#define QSPI_CFG_EDPI									(0x1UL << 15)
#define QSPI_CFG_EAARMAP								(0x1UL << 16)
#define QSPI_CFG_EXMONREAD								(0x1UL << 17)
#define QSPI_CFG_EXM_IMMEDIATELY						(0x1UL << 18)
#define QSPI_CFG_MMBRD_POS								(19)
#define QSPI_CFG_MMBRD_MSK								(0xFUL << QSPI_CFG_MMBRD_POS)
#define QSPI_CFG_ENABLE_AHB_DECODER						(0x1UL << 23)
#define QSPI_CFG_ENABLE_DTR_PROTOCOL					(0x1UL << 24)
#define QSPI_CFG_PIPELINE_PHY_MODE_ENABLE				(0x1UL << 25)
#define QSPI_CFG_STATUS_IDLE							(0x1UL << 31)

#define QSPI_DEVRD_READ_OPCODE_POS						(0)
#define QSPI_DEVRD_READ_OPCODE_MSK						(0xFFUL << QSPI_DEVRD_READ_OPCODE_POS)
#define QSPI_DEVRD_INSTRUCTION_TYPE_POS					(8)
#define QSPI_DEVRD_INSTRUCTION_TYPE_MSK					(0x3UL << QSPI_DEVRD_INSTRUCTION_TYPE_POS)
#define QSPI_DEVRD_DDR_BIT_ENABLE						(0x1UL << 10)
#define QSPI_DEVRD_ADDRESS_TRANSFER_TYPE_POS			(12)
#define QSPI_DEVRD_ADDRESS_TRANSFER_TYPE_MSK			(0x3UL << QSPI_DEVRD_ADDRESS_TRANSFER_TYPE_POS)
#define QSPI_DEVRD_DATA_TRANSFER_TYPE_POS				(16)
#define QSPI_DEVRD_DATA_TRANSFER_TYPE_MSK				(0x3UL << QSPI_DEVRD_DATA_TRANSFER_TYPE_POS)
#define QSPI_DEVRD_MODE_BIT_ENABLE						(0x1UL << 20)
#define QSPI_DEVRD_READ_INSTRUCTION_POS					(24)
#define QSPI_DEVRD_READ_INSTRUCTION_MSK					(0x1FUL << QSPI_DEVRD_READ_INSTRUCTION_POS)

#define QSPI_DEVWR_WRITE_OPCODE_POS						(0)
#define QSPI_DEVWR_WRITE_OPCODE_MSK						(0xFFUL << QSPI_DEVWR_WRITE_OPCODE_POS)
#define QSPI_DEVWR_WEL_DISABLE							(0x1UL << 8)
#define QSPI_DEVWR_ADDRESS_TRANSFER_TYPE_POS			(12)
#define QSPI_DEVWR_ADDRESS_TRANSFER_TYPE_MSK			(0x3UL << QSPI_DEVWR_ADDRESS_TRANSFER_TYPE_POS)
#define QSPI_DEVWR_DATA_TRANSFER_TYPE_POS				(16)
#define QSPI_DEVWR_DATA_TRANSFER_TYPE_MSK				(0x3UL << QSPI_DEVWR_DATA_TRANSFER_TYPE_POS)
#define QSPI_DEVRD_WRITE_INSTRUCTION_POS				(24)
#define QSPI_DEVRD_WRITE_INSTRUCTION_MSK				(0x1FUL << QSPI_DEVRD_WRITE_INSTRUCTION_POS)

#define QSPI_DELAY_CSSOT_POS							(0)
#define QSPI_DELAY_CSSOT_MSK							(0xFFUL << QSPI_DELAY_CSSOT_POS)
#define QSPI_DELAY_CSEOT_POS							(8)
#define QSPI_DELAY_CSEOT_MSK							(0xFFUL << QSPI_DELAY_CSEOT_POS)
#define QSPI_DELAY_CSDADS_POS							(16)
#define QSPI_DELAY_CSDADS_MSK							(0xFFUL << QSPI_DELAY_CSDADS_POS)
#define QSPI_DELAY_CSDA_POS								(24)
#define QSPI_DELAY_CSDA_MSK								(0xFFUL << QSPI_DELAY_CSDA_POS)

#define QSPI_RDDATACAP_ENABLE_CLOCK						(0x1UL << 0)
#define QSPI_RDDATACAP_DELAY_READ_POS					(1)
#define QSPI_RDDATACAP_DELAY_READ_MSK					(0xFUL << QSPI_RDDATACAP_DELAY_READ_POS)
#define QSPI_RDDATACAP_SAMPLE_EDGE_SELECTION			(0x1UL << 5)
#define QSPI_RDDATACAP_DELAY_TRANSMITTED_POS			(16)
#define QSPI_RDDATACAP_DELAY_TRANSMITTED_MSK			(0xFUL << QSPI_RDDATACAP_DELAY_TRANSMITTED_POS)

#define QSPI_DEVSZ_ADDRESS_BYTES_POS					(0)
#define QSPI_DEVSZ_ADDRESS_BYTES_MSK					(0xFUL << QSPI_DEVSZ_ADDRESS_BYTES_POS)
#define QSPI_DEVSZ_DEVICE_PAGE_POS						(4)
#define QSPI_DEVSZ_DEVICE_PAGE_MSK						(0xFFFUL << QSPI_DEVSZ_DEVICE_PAGE_POS)
#define QSPI_DEVSZ_PER_BLCOK_POS						(16)
#define QSPI_DEVSZ_PER_BLCOK_MSK						(0x1FUL << QSPI_DEVSZ_PER_BLCOK_POS)
#define QSPI_DEVSZ_CONNECTED_CS0_POS					(21)
#define QSPI_DEVSZ_CONNECTED_CS0_MSK					(0x3UL << QSPI_DEVSZ_CONNECTED_CS0_POS)
#define QSPI_DEVSZ_CONNECTED_CS1_POS					(23)
#define QSPI_DEVSZ_CONNECTED_CS1_MSK					(0x3UL << QSPI_DEVSZ_CONNECTED_CS1_POS)
#define QSPI_DEVSZ_CONNECTED_CS2_POS					(25)
#define QSPI_DEVSZ_CONNECTED_CS2_MSK					(0x3UL << QSPI_DEVSZ_CONNECTED_CS2_POS)
#define QSPI_DEVSZ_CONNECTED_CS3_POS					(27)
#define QSPI_DEVSZ_CONNECTED_CS3_MSK					(0x3UL << QSPI_DEVSZ_CONNECTED_CS3_POS)

#define QSPI_SRAMPART_INDIRECT_READ_PARTITION_POS		(0)	//note
#define QSPI_SRAMPART_INDIRECT_READ_PARTITION_MSK		(0x3UL << QSPI_SRAMPART_INDIRECT_READ_PARTITION_POS)

#define QSPI_INDADDRTRIG_INDIRECT_TRIGGER_ADDRESS_POS	(0)
#define QSPI_INDADDRTRIG_INDIRECT_TRIGGER_ADDRESS_MSK	(0xFFFFFFFFUL << QSPI_INDADDRTRIG_INDIRECT_TRIGGER_ADDRESS_POS)

#define QSPI_DMAPER_SINGLE_TYPE_POS						(0)
#define QSPI_DMAPER_SINGLE_TYPE_MSK						(0xFUL << QSPI_DMAPER_SINGLE_TYPE_POS)
#define QSPI_DMAPER_BURST_TYPE_POS						(8)
#define QSPI_DMAPER_BURST_TYPE_MSK						(0xFUL << QSPI_DMAPER_BURST_TYPE_POS)

#define QSPI_REMAPADDR_AHB_ADDRESS_POS					(0)
#define QSPI_REMAPADDR_AHB_ADDRESS_MSK					(0xFFFFFFFFUL << QSPI_REMAPADDR_AHB_ADDRESS_POS)

#define QSPI_MODEBIT_ADDRESS_BYTES_POS					(0)
#define QSPI_MODEBIT_ADDRESS_BYTES_MSK					(0xFFUL << QSPI_MODEBIT_ADDRESS_BYTES_POS)

#define QSPI_SRAMFILL_READ_PARTITION_POS				(0)
#define QSPI_SRAMFILL_READ_PARTITION_MSK				(0xFFFFUL << QSPI_SRAMFILL_READ_PARTITION_POS)
#define QSPI_SRAMFILL_WRITE_PARTITION_POS				(15)
#define QSPI_SRAMFILL_WRITE_PARTITION_MSK				(0xFFFFUL << QSPI_SRAMFILL_WRITE_PARTITION_POS)

#define QSPI_TTR_TX_FIFO								(0x1UL << 0)

#define QSPI_RTR_RX_FIFO								(0x1UL << 0)

#define QSPI_WCCR_OPCODE_POS							(0)
#define QSPI_WCCR_OPCODE_MSK							(0xFFUL << QSPI_WCCR_OPCODE_POS)
#define QSPI_WCCR_POLLING_BIT_INDEX_POS					(8)
#define QSPI_WCCR_POLLING_BIT_INDEX_MSK					(0x7UL << QSPI_WCCR_POLLING_BIT_INDEX_POS)
#define QSPI_WCCR_POLLING_POLARITY						(0x1UL << 13)
#define QSPI_WCCR_DISABLE_POLLING						(0x1UL << 14)
#define QSPI_WCCR_POLLING_COUNT_POS						(16)
#define QSPI_WCCR_POLLING_COUNT_MSK						(0xFFUL << QSPI_WCCR_POLLING_COUNT_POS)
#define QSPI_WCCR_POLLING_DELAY_POS						(24)
#define QSPI_WCCR_POLLING_DELAY_MSK						(0xFFUL << QSPI_WCCR_POLLING_DELAY_POS)

#define QSPI_PER_POLLS_CYCLES_POS						(0)
#define QSPI_PER_POLLS_CYCLES_MSK						(0xFFFFFFFFUL << QSPI_PER_POLLS_CYCLES_POS)

#define QSPI_ISR_MODE_FAIL_M							(0x1UL << 0)
#define QSPI_ISR_UNDERFLOW_DETECTED						(0x1UL << 1)
#define QSPI_ISR_INDIRECT_OPERATION						(0x1UL << 2)
#define QSPI_ISR_NO_ACCEPTED							(0x1UL << 3)
#define QSPI_ISR_WRITE_TO_PROTECTED						(0x1UL << 4)
#define QSPI_ISR_ILLEGAL_AHB_ACCESS						(0x1UL << 5)
#define QSPI_ISR_LEVEL_BREACHED							(0x1UL << 6)
#define QSPI_ISR_RECEIVE_OVERFLOW						(0x1UL << 7)
#define QSPI_ISR_SMALL_TX_FIFO_EMPTY					(0x1UL << 8)
#define QSPI_ISR_SMALL_TX_FIFO_FULL						(0x1UL << 9)
#define QSPI_ISR_SMALL_RX_FIFO_EMPTY					(0x1UL << 10)
#define QSPI_ISR_SMALL_RX_FIFO_FULL						(0x1UL << 11)
#define QSPI_ISR_INDIRECT_READ_PARTITION				(0x1UL << 12)
#define QSPI_ISR_POLLS_CYCLES							(0x1UL << 13)

#define QSPI_IMR_INTERRUPT_STATUS_POS					(0)
#define QSPI_IMR_INTERRUPT_STATUS_MSK					(0x3FFUL << QSPI_IMR_INTERRUPT_STATUS_POS)

#define QSPI_LWPR_LOWER_WRITE_POS						(0)
#define QSPI_LWPR_LOWER_WRITE_MSK						(0xFFFFFFFFUL << QSPI_LWPR_LOWER_WRITE_POS)

#define QSPI_UWPR_UPPER_WRITE_POS						(0)
#define QSPI_UWPR_UPPER_WRITE_MSK						(0xFFFFFFFFUL << QSPI_UWPR_UPPER_WRITE_POS)

#define	QSPI_WPR_WRITE_EINVERSION						(0x1UL << 0)
#define	QSPI_WPR_WRITE_ENABLE							(0x1UL << 1)

#define	QSPI_IRTCR_START_INDIRECT_READ					(0x1UL << 0)
#define	QSPI_IRTCR_CANCEL_INDIRECT_READ					(0x1UL << 1)
#define	QSPI_IRTCR_INDIRECT_READ_OPERATION				(0x1UL << 2)
#define	QSPI_IRTCR_COMPLETE_INDIRECT_OPERATION			(0x1UL << 3)
#define	QSPI_IRTCR_TWO_INDIRECT_READ					(0x1UL << 4)
#define	QSPI_IRTCR_INDIRECT_COMPLETION_STATUS			(0x1UL << 5)
#define	QSPI_IRTCR_INDIRECT_NUMBER_POS					(6)
#define	QSPI_IRTCR_INDIRECT_NUMBER_MSK					(0x03UL << QSPI_IRTCR_INDIRECT_NUMBER_POS)

#define	QSPI_IRTWR_WATEMARK_VALUE_POS					(0)
#define	QSPI_IRTWR_WATEMARK_VALUE_MSK					(0xFFFFFFFFUL << QSPI_IRTWR_WATEMARK_VALUE_POS)

#define	QSPI_IRTSAR_INDIRECT_ACCESS_POS					(0)
#define	QSPI_IRTSAR_INDIRECT_ACCESS_MSK					(0xFFFFFFFFUL << QSPI_IRTSAR_INDIRECT_ACCESS_POS)

#define	QSPI_IRTNBR_INDIRECT_NUMBER_BYTTES_POS			(0)
#define	QSPI_IRTNBR_INDIRECT_NUMBER_BYTTES_MSK			(0xFFFFFFFFUL << QSPI_IRTNBR_INDIRECT_NUMBER_BYTTES_POS)

#define	QSPI_IWTR_START_INDIRECT_WRITE					(0x1UL << 0)
#define	QSPI_IWTR_CANCEL_INDIRECT_WRITE					(0x1UL << 1)
#define	QSPI_IWTR_INDIRECT_WRITE_OPERATION				(0x1UL << 2)
#define	QSPI_IWTR_TWO_INDIRECT_WRITE					(0x1UL << 4)
#define	QSPI_IWTR_INDIRECT_COMPLETION_STATUS			(0x1UL << 5)
#define	QSPI_IWTR_INDIRECT_NUMBER_POS					(6)
#define	QSPI_IWTR_INDIRECT_NUMBER_MSK					(0x03UL << QSPI_IWTR_INDIRECT_NUMBER_POS)

#define	QSPI_IWTWR_WATEMARK_VALUE_POS					(0)
#define	QSPI_IWTWR_WATEMARK_VALUE_MSK					(0xFFFFFFFFUL << QSPI_IWTWR_WATEMARK_VALUE_POS)

#define	QSPI_IWTSAR_INDIRECT_ACCESS_POS					(0)
#define	QSPI_IWTSAR_INDIRECT_ACCESS_MSK					(0xFFFFFFFFUL << QSPI_IWTSAR_INDIRECT_ACCESS_POS)

#define	QSPI_IWTNBR_INDIRECT_NUMBER_BYTTES_POS			(0)
#define	QSPI_IWTNBR_INDIRECT_NUMBER_BYTTES_MSK			(0xFFFFFFFFUL << QSPI_IWTNBR_INDIRECT_NUMBER_BYTTES_POS)

#define	QSPI_ITARR_INDIRECT_RANGE_WIDTH_POS				(0)
#define	QSPI_ITARR_INDIRECT_RANGE_WIDTH_MSK				(0xFUL << QSPI_ITARR_INDIRECT_RANGE_WIDTH_POS)

#define	QSPI_FCR_EXECUTE_THE_COMMAND					(0x1UL << 0)
#define	QSPI_FCR_COMMAND_EXECUTE_IN_PROGRESS			(0x1UL << 1)
#define	QSPI_FCR_DUMMY_BYTES_POS						(7)
#define	QSPI_FCR_DUMMY_BYTES_MSK						(0x1FUL << QSPI_FCR_DUMMY_BYTES_POS)
#define	QSPI_FCR_WRITE_DATA_BYTES_POS					(12)
#define	QSPI_FCR_WRITE_DATA_BYTES_MSK					(0x7UL << QSPI_FCR_WRITE_DATA_BYTES_POS)
#define	QSPI_FCR_WRITE_DATA_ENABLE						(0x1UL << 15)
#define	QSPI_FCR_ADDRESS_BYTES_POS						(16)
#define	QSPI_FCR_ADDRESS_BYTES_MSK						(0x3UL << QSPI_FCR_ADDRESS_BYTES_POS)
#define	QSPI_FCR_MODE_BIT_ENABLE						(0x1UL << 18)
#define	QSPI_FCR_COMMAND_ADDRESS_ENABLE					(0x1UL << 19)
#define	QSPI_FCR_READ_DATA_BYTES_POS					(20)
#define	QSPI_FCR_READ_DATA_BYTES_MSK					(0x7UL << QSPI_FCR_READ_DATA_BYTES_POS)
#define	QSPI_FCR_READ_DATA_ENABLE						(0x1UL << 23)
#define	QSPI_FCR_COMMAND_OPCODE_POS						(24)
#define	QSPI_FCR_COMMAND_OPCODE_MSK						(0xFFUL << QSPI_FCR_COMMAND_OPCODE_POS)

#define	QSPI_FCAR_COMMAND_ADDRESS_POS					(0)
#define	QSPI_FCAR_COMMAND_ADDRESS_MSK					(0xFFFFFFFFUL << QSPI_FCAR_COMMAND_ADDRESS_POS)

#define	QSPI_FCRDRL_READ_DATA_LOWER_BYTE_POS			(0)
#define	QSPI_FCRDRL_READ_DATA_LOWER_BYTE_MSK 			(0xFFFFFFFFUL << QSPI_FCRDRL_READ_DATA_LOWER_BYTE_POS)

#define	QSPI_FCRDRU_READ_DATA_UPPER_BYTE_POS			(0)
#define	QSPI_FCRDRU_READ_DATA_UPPER_BYTE_MSK			(0xFFFFFFFFUL << QSPI_FCRDRU_READ_DATA_UPPER_BYTE_POS)

#define	QSPI_FCWDRL_WRITE_DATA_LOWER_BYTE_POS			(0)
#define	QSPI_FCWDRL_WRITE_DATA_LOWER_BYTE_MSK			(0xFFFFFFFFUL << QSPI_FCWDRL_WRITE_DATA_LOWER_BYTE_POS)

#define	QSPI_FCWDRU_WRITE_DATA_UPPER_BYTE_POS			(0)
#define	QSPI_FCWDRU_WRITE_DATA_UPPER_BYTE_MSK			(0xFFFFFFFFUL << QSPI_FCWDRU_WRITE_DATA_UPPER_BYTE_POS)

#define	QSPI_PFSR_FLASH_STATUS_POS						(0)
#define	QSPI_PFSR_FLASH_STATUS_MSK						(0xFFUL << QSPI_PFSR_FLASH_STATUS_POS)
#define QSPI_PFSR_POLLING_STATUS_VALID					(0x1UL << 8)

/*
 * Commands
 */
#define QSPI_CMD_WRITE_ENABLE							0x06
#define QSPI_CMD_WRITE_DISABLE							0x04
#define QSPI_CMD_READ_IDENT								0x9F
#define QSPI_CMD_READ_STATUS							0x05
#define QSPI_CMD_WRITE_STATUS							0x01
#define QSPI_CMD_READ									0x03
#define QSPI_CMD_FAST_READ								0x0B
#define QSPI_CMD_PAGE_PROGRAM							0x02
#define QSPI_CMD_SECTOR_ERASE							0x20
#define QSPI_CMD_BULK_ERASE								0xC7
#define QSPI_CMD_BLOCK_4K_ERASE							0x20
#define QSPI_CMD_BLOCK_32K_ERASE						0x52
#define QSPI_CMD_ENTER_4B_MODE							0xB7
#define QSPI_CMD_EXIT_4B_MODE							0xE9

/* Quad 4B-addressing operations. */
#define QSPI_CMD_QUAD_SECTOR_ERASE						0xDC
#define QSPI_CMD_QUAD_PAGE_PROGRAM						0x34
#define QSPI_CMD_READ_4B_QUAD_OUTPUT					0x6C

/* Used for Spansion S25FS-S family flash only. */
#define QSPI_CMD_RDAR									0x65	/* Read any device register */
#define QSPI_CMD_WRAR									0x71	/* Write any device register */

#define QSPI_CMD_WRITE_QUAD								0x32
#define QSPI_CMD_READ_QUAD								0x6B

#define QSPI_CMD_SE										0xd8	/* Sector erase (usually 64KiB) */

/*
 * Status register flags
 */
#define QSPI_STATUS_SRWD								(1 << 7)
#define QSPI_STATUS_BP2									(1 << 4)
#define QSPI_STATUS_BP1									(1 << 3)
#define QSPI_STATUS_BP0									(1 << 2)
#define QSPI_STATUS_WEL									(1 << 1)
#define QSPI_STATUS_WIP									(1 << 0)

#define WOKOO_QSPI_MAP_ADDR								0xB0000000
#define WOKOO_QSPI_MAP_SIZE								0x01000000

struct wokoo_qspi {
	struct completion	transfer_complete;

	/* list synchronization */
	struct mutex        list_lock;

	struct spi_master	*master;
	void __iomem        *base;
	struct clk			*pclk;
	struct device       *dev;

	struct wokoo_qspi_regs *qspi_regs;

	dma_addr_t			mmap_phys_base;
	dma_addr_t			rx_bb_dma_addr;
	dma_addr_t			tx_bb_dma_addr;
	void				*rx_bb_addr;
	void				*tx_bb_addr;
	int					irq;
	int					dma_busy;
	struct dma_chan		*rx_chan;
	struct dma_chan		*tx_chan;

	uint8_t				cur_readcmd;
	uint8_t				cur_writecmd;
};

#define QSPI_DMA_BUFFER_SIZE			SZ_64K
#define OFFSET_BITS_MASK				GENMASK(23, 0)



static int wokoo_qspi_wait_for_completion(struct wokoo_qspi_regs *regs);
static int wokoo_qspi_cmd_read(struct wokoo_qspi_regs *regs, uint8_t cmd, uint8_t *addr, uint32_t len);
static int wokoo_qspi_wait_idle(struct wokoo_qspi_regs *regs);
static int wokoo_qspi_wait_ready(struct wokoo_qspi_regs *regs);
static int wokoo_qspi_direct_quad_init(struct wokoo_qspi_regs *regs);
static int wokoo_qspi_write(struct wokoo_qspi_regs *regs, uint32_t addr, uint32_t *data, uint32_t count);
static int wokoo_qspi_read(struct wokoo_qspi_regs *regs, uint32_t addr, uint32_t *data, uint32_t count);
static int wokoo_qspi_cmd_write(struct wokoo_qspi_regs *regs, uint8_t cmd, uint32_t addr,
									uint8_t bytes, uint8_t *buf, uint32_t len);

static void __iomem        *qspi_map_addr;





/*
 * wokoo_qspi_wait_for_completion - wait completion
 *
 * @regs: regs
 */
static int wokoo_qspi_wait_for_completion(struct wokoo_qspi_regs *regs) {
	int timeout;
	int i;

	timeout = 10000;
	for (i = timeout; i > 0; i--) {
		if ((readl(&regs->FCR) & QSPI_FCR_COMMAND_EXECUTE_IN_PROGRESS) == 0) {
			break;
		}

		udelay(1);
	}

	if (i == 0) {
		return (-1);
	}
	return (0);
}

/*
 * wokoo_qspi_cmd_read - read command
 *
 * @regs: regs
 * @cmd: command
 * @buf: buffer
 * @len: buffer length
 */
static int wokoo_qspi_cmd_read(struct wokoo_qspi_regs *regs, uint8_t cmd, uint8_t *buf, uint32_t len) {
	uint32_t data;
	uint32_t reg;
	int ret;
	int i, j;

	wokoo_qspi_wait_idle(regs);
	reg = (cmd << QSPI_FCR_COMMAND_OPCODE_POS);
	if (len > 0) {
		reg |= ((len - 1) << QSPI_FCR_READ_DATA_BYTES_POS);
		reg |= QSPI_FCR_READ_DATA_ENABLE;
	}
	writel(reg, &regs->FCR);

	reg |= QSPI_FCR_EXECUTE_THE_COMMAND;
	writel(reg, &regs->FCR);

	ret = wokoo_qspi_wait_for_completion(regs);
	if (ret != 0) {
		return (ret);
	}

	if (len > 4) {
		j = 4;
	} if (len > 8) {
		j   = 4;
		len = 8;
	} else {
		j = len;
	}

	/* Lower bytes */
	if (j) {
		data = readl(&regs->FCRDRL);
		for (i = 0; i < j; i++)
			buf[i] = (data >> (i * 8)) & 0xff;
	}

	/* Upper bytes */
	if (len > 4) {
		data = readl(&regs->FCRDRU);
		for (i = 4; i < len; i++)
			buf[i] = (data >> ((i - 4) * 8)) & 0xff;
	}
	return (0);
}

/*
 * wokoo_qspi_cmd_write - write command
 *
 * @regs: regs
 * @cmd: command
 * @addr: address
 * @bytes: address bytes
 * @addr: buffer
 * @len: buffer length
 */
static int wokoo_qspi_cmd_write(struct wokoo_qspi_regs *regs, uint8_t cmd, uint32_t addr,
								uint8_t bytes, uint8_t *buf, uint32_t len) {
	uint32_t reg = 0;
	uint32_t data_rl = 0;
	uint32_t data_ru = 0;
	uint32_t nl, nu, i;
	int ret;

	wokoo_qspi_wait_idle(regs);
	if (len > 8) {
		len = 8;
	}

	if (len < 4) {
		nu = 0;
		nl = len;
	} else {
		nl = 4;
		nu = len - nl;
	}

	/* Lower bytes */
	if (nl) {
		for (i = 0; i < nl; i++) {
			data_rl |= buf[i] << i * 8;
		}

		writel(data_rl, &regs->FCWDRL);
	}

	/* Upper bytes */
	if (nu) {
		for (i = 0; i < nu; i++) {
			data_ru |= buf[i + 4] << i * 8;
		}

		writel(data_ru, &regs->FCWDRU);
	}

	reg = (cmd << QSPI_FCR_COMMAND_OPCODE_POS);
	if (bytes > 0) {
		writel(addr, &regs->FCAR);
		reg |= QSPI_FCR_COMMAND_ADDRESS_ENABLE;
		reg |= ((bytes - 1) << QSPI_FCR_ADDRESS_BYTES_POS);
	}

	if (len > 0) {
		reg |= QSPI_FCR_WRITE_DATA_ENABLE;
		reg |= ((len - 1) << QSPI_FCR_WRITE_DATA_BYTES_POS);
	}
	writel(reg, &regs->FCR);
	reg |= QSPI_FCR_EXECUTE_THE_COMMAND;
	writel(reg, &regs->FCR);
	ret = wokoo_qspi_wait_for_completion(regs);
	wokoo_qspi_wait_idle(regs);
	return (ret);
}

/*
 * wokoo_qspi_wait_idle - wait idle
 *
 * @regs: regs
 */
static int wokoo_qspi_wait_idle(struct wokoo_qspi_regs *regs) {
	uint32_t regv;

	do {
		regv = readl(&regs->CFG);
		if (regv & QSPI_CFG_STATUS_IDLE) {
			break;
		}

		usleep_range(1, 2);
	} while (1);

	return (0);
}

/*
 * wokoo_qspi_wait_ready - wait ready
 *
 * @regs: regs
 */
static int wokoo_qspi_wait_ready(struct wokoo_qspi_regs *regs) {
	uint8_t data = 0;
	int     ret  = 0;

	do {
		ret = wokoo_qspi_cmd_read(regs, QSPI_CMD_READ_STATUS, &data, 1);
	
	} while (data & QSPI_STATUS_WIP);

	return ret;
}

/*
 * wokoo_qspi_direct_quad_init - qspi init
 *
 * @regs: regs
 */
static int wokoo_qspi_direct_quad_init(struct wokoo_qspi_regs *regs) {
	uint32_t reg;
	int ret = 0;


	writel(readl(&regs->CFG) & ~QSPI_CFG_ENABLE, &regs->CFG);
	writel(readl(&regs->DEVSZ) & (~QSPI_DEVSZ_ADDRESS_BYTES_MSK | (4 - 1)), &regs->DEVSZ);
	writel(0, &regs->IMR);

	reg = (3 << QSPI_DELAY_CSDA_POS);
	reg |= (3 << QSPI_DELAY_CSDADS_POS);
	reg |= (1 << QSPI_DELAY_CSEOT_POS);
	reg |= (1 << QSPI_DELAY_CSSOT_POS);
	writel(reg, &regs->DELAY);

	reg = readl(&regs->RDDATACAP);
	reg &= ~(QSPI_RDDATACAP_DELAY_READ_MSK);
	reg |= (1 << QSPI_RDDATACAP_DELAY_READ_POS);
	writel(reg, &regs->RDDATACAP);

	writel(readl(&regs->CFG) | QSPI_CFG_ENABLE, &regs->CFG);
	wokoo_qspi_wait_idle(regs);

	writel(readl(&regs->CFG) | QSPI_CFG_EAARMAP, &regs->CFG);
	wokoo_qspi_wait_idle(regs);

	return ret;
}

/*
 * wokoo_qspi_write - qspi write data
 *
 * @regs: regs
 * @addr: offset address
 * @data: data buffer
 * @count: data length
 */
static int wokoo_qspi_write(struct wokoo_qspi_regs *regs, uint32_t addr, uint32_t *data, uint32_t count) {
	uint32_t s;
	uint32_t *p;

	wokoo_qspi_wait_idle(regs);
	writel(addr, &regs->REMAPADDR);

	p = (uint32_t *) qspi_map_addr;
	for (s = 0; s < count; s = s + 1) {
		*p++ = data[s];
	}

	wokoo_qspi_wait_ready(regs);
	wokoo_qspi_wait_idle(regs);
	return (0);
}

/*
 * wokoo_qspi_read - qspi read data
 *
 * @regs: regs
 * @addr: offset address
 * @data: data buffer
 * @count: data length
 */
static int wokoo_qspi_read(struct wokoo_qspi_regs *regs, uint32_t addr, uint32_t *data, uint32_t count) {
	uint32_t s;
	uint32_t *p;

	wokoo_qspi_wait_idle(regs);
	writel(addr, &regs->REMAPADDR);

	p = (uint32_t *) qspi_map_addr;
	for (s = 0; s < count; s = s + 1) {
		data[s] = *p++;
	}

	return (0);
}

/*
 * wokoo_qspi_dma_complete - qspi dma comlete callback
 *
 */
static void wokoo_qspi_dma_complete(void *arg)
{
	struct wokoo_qspi *qspi = arg;

	qspi->dma_busy = false;
	//dev_dbg(qspi->dev, "wokoo_qspi_dma_complete\n");
	complete(&qspi->transfer_complete);
}

/*
 * wokoo_qspi_dma_write - qspi dma write data
 *
 * @regs: regs
 * @addr: offset address
 * @data: data buffer
 * @count: data length
 */
static int wokoo_qspi_dma_write(struct wokoo_qspi *qspi, uint32_t addr, uint32_t *data, uint32_t count)
{
	uint32_t txlen = 0;
	uint32_t hadlen = 0;
	struct wokoo_qspi_regs *regs = qspi->qspi_regs;
	struct dma_async_tx_descriptor *desc = NULL;
	uint8_t *p = (uint8_t *)data;
	dma_cookie_t cookie;

	if (qspi->dma_busy)
		return -1;

	qspi->dma_busy = true;

	/* Quad data size */
	count <<= 2;
	while (count > 0) {
		if (count >= QSPI_DMA_BUFFER_SIZE) {
			txlen  = QSPI_DMA_BUFFER_SIZE;
			count -= QSPI_DMA_BUFFER_SIZE;
		} else {
			txlen = count;
			count = 0;
		}

		wokoo_qspi_wait_idle(regs);
		writel(addr + hadlen, &regs->REMAPADDR);

		memcpy(qspi->tx_bb_addr, p + hadlen, txlen);

		desc = dmaengine_prep_slave_single(qspi->tx_chan,
										   qspi->tx_bb_dma_addr,
										   txlen,
										   DMA_MEM_TO_DEV,
										   DMA_PREP_INTERRUPT);

		reinit_completion(&qspi->transfer_complete);

		if (!desc) {
			dev_err(qspi->dev, "dmaengine_prep_slave_single error\n");
		}

		if (qspi->dma_busy) {
			qspi->dma_busy = false;
		}

		desc->callback = wokoo_qspi_dma_complete;
		desc->callback_param = qspi;

		/* Push current DMA RX transaction in the pending queue */
		cookie = dmaengine_submit(desc);

		/* Issue pending DMA RX requests */
		dma_async_issue_pending(qspi->tx_chan);

		wait_for_completion_timeout(&qspi->transfer_complete,
						msecs_to_jiffies(MSEC_PER_SEC));
		hadlen += txlen;
	}

	wokoo_qspi_wait_ready(regs);
	wokoo_qspi_wait_idle(regs);
	return (0);
}

/*
 * wokoo_qspi_dma_read - qspi read data
 *
 * @regs: regs
 * @addr: offset address
 * @data: data buffer
 * @count: data length
 */
static int wokoo_qspi_dma_read(struct wokoo_qspi *qspi, uint32_t addr, uint32_t *data, uint32_t count)
{
	uint32_t rxlen = 0;
	uint32_t hadlen = 0;
	struct wokoo_qspi_regs *regs = qspi->qspi_regs;
	struct dma_async_tx_descriptor *desc = NULL;
	uint8_t *p = (uint8_t *)data;
	dma_cookie_t cookie;

	if (qspi->dma_busy)
		return -1;

	qspi->dma_busy = true;

	count <<= 2;
	while (count > 0) {
		if (count >= QSPI_DMA_BUFFER_SIZE) {
			rxlen  = QSPI_DMA_BUFFER_SIZE;
			count -= QSPI_DMA_BUFFER_SIZE;
		} else {
			rxlen = count;
			count = 0;
		}

		reinit_completion(&qspi->transfer_complete);

		/* Rewrite aadress */
		wokoo_qspi_wait_idle(regs);
		writel(addr + hadlen, &regs->REMAPADDR);

		desc = dmaengine_prep_slave_single(qspi->rx_chan,
										   qspi->rx_bb_dma_addr,
										   rxlen,
										   DMA_DEV_TO_MEM,
										   DMA_PREP_INTERRUPT);

		if (!desc) {
			dev_err(qspi->dev, "dmaengine_prep_slave_single error\n");
		}

		if (qspi->dma_busy) {
			qspi->dma_busy = false;
		}

		desc->callback = wokoo_qspi_dma_complete;
		desc->callback_param = qspi;

		/* Push current DMA RX transaction in the pending queue */
		cookie = dmaengine_submit(desc);

		/* Issue pending DMA RX requests */
		dma_async_issue_pending(qspi->rx_chan);

		wait_for_completion_timeout(&qspi->transfer_complete,
						msecs_to_jiffies(MSEC_PER_SEC));
		/* Copy for DMA pool */
		memcpy(p + hadlen, qspi->rx_bb_addr, rxlen);
		hadlen += rxlen;
	}

	return (0);
}

/*
 * qspi_op_read_init - qspi read init
 *
 * @qspi: regs
 * @readcmd: read command
 * @addrlen: address bytes
 */
static void qspi_op_read_init(struct wokoo_qspi *qspi, uint8_t readcmd, uint32_t addrlen)
{
	u32 reg;

	if (readcmd != qspi->cur_readcmd) {
		/* Set register only in idle status */
		wokoo_qspi_wait_idle(qspi->qspi_regs);
		/* Write address length */
		reg = readl(&qspi->qspi_regs->DEVSZ) & ~QSPI_DEVSZ_ADDRESS_BYTES_MSK;
		reg |= (addrlen - 1);
		writel(reg, &qspi->qspi_regs->DEVSZ);
		reg = 0;
		qspi->cur_readcmd = readcmd;

		if (readcmd == SPINOR_OP_READ_1_1_4 ||
			readcmd == SPINOR_OP_READ_1_4_4 ||
			readcmd == SPINOR_OP_READ_1_1_4_4B ||
			readcmd == SPINOR_OP_READ_1_4_4_4B) {
			if (readcmd == SPINOR_OP_READ_1_4_4 ||
				readcmd == SPINOR_OP_READ_1_4_4_4B) {
				/* Quad I/O Fast Read£¬ 3B or 4B address */
				reg = (0x06 << QSPI_DEVRD_READ_INSTRUCTION_POS);
				reg |= (0x02 << QSPI_DEVRD_ADDRESS_TRANSFER_TYPE_POS);
			} else {
				/* Quad Output Fast Read */
				reg = (0x08 << QSPI_DEVRD_READ_INSTRUCTION_POS);
				reg |= (0x00 << QSPI_DEVRD_ADDRESS_TRANSFER_TYPE_POS);
			}
			reg |= (0x02 << QSPI_DEVRD_DATA_TRANSFER_TYPE_POS);
		} else {
			if (readcmd == SPINOR_OP_READ_FAST ||
				readcmd == SPINOR_OP_READ ||
				readcmd == SPINOR_OP_READ_FAST_4B ||
				readcmd == SPINOR_OP_READ_4B) {
				/* Read Data Bytes or Fast Read */
				reg  = (0x08 << QSPI_DEVRD_READ_INSTRUCTION_POS);
				reg |= (0x00 << QSPI_DEVRD_DATA_TRANSFER_TYPE_POS);
				reg |= (0x00 << QSPI_DEVRD_ADDRESS_TRANSFER_TYPE_POS);
			} else if (readcmd == SPINOR_OP_READ_1_1_2 ||
				   readcmd == SPINOR_OP_READ_1_2_2 ||
				   readcmd == SPINOR_OP_READ_1_1_2_4B ||
				   readcmd == SPINOR_OP_READ_1_2_2_4B ) {
				if (readcmd == SPINOR_OP_READ_1_2_2 ||
					readcmd == SPINOR_OP_READ_1_2_2_4B) {
					/* Dual I/O Fast Read */
					reg  = (0x04 << QSPI_DEVRD_READ_INSTRUCTION_POS);
					reg |= (0x01 << QSPI_DEVRD_ADDRESS_TRANSFER_TYPE_POS);
				} else {
					/* Dual Output Fast Read */
					reg  = (0x08 << QSPI_DEVRD_READ_INSTRUCTION_POS);
					reg |= (0x00 << QSPI_DEVRD_ADDRESS_TRANSFER_TYPE_POS);
				}

				reg |= (0x01 << QSPI_DEVRD_DATA_TRANSFER_TYPE_POS);
			} else {
				/* Other command, like 0x5A etc */
				reg  = (0x08 << QSPI_DEVRD_READ_INSTRUCTION_POS);
				reg |= (0x00 << QSPI_DEVRD_DATA_TRANSFER_TYPE_POS);
				reg |= (0x00 << QSPI_DEVRD_ADDRESS_TRANSFER_TYPE_POS);
			}
		}

		reg |= (0x00 << QSPI_DEVRD_INSTRUCTION_TYPE_POS);
		reg |= (readcmd << QSPI_DEVRD_READ_OPCODE_POS);
		writel(reg, &qspi->qspi_regs->DEVRD);
	}
}

/*
 * wokoo_qspi_op_read - qspi read data
 *
 * @qspi: qspi
 * @readcmd: read command
 * @sf_addr: offset address
 * @addrlen: address bytes
 * @rxbuf: data buffer
 * @len: data length
 */
static void wokoo_qspi_op_read(struct wokoo_qspi *qspi, uint8_t readcmd, 
							   uint32_t sf_addr, uint32_t addrlen, uint8_t *rxbuf, u32 len)
{
	u32 nquad, ntail, tail, i;
	u8 *rdata;

	nquad = len / 4;
	ntail = len % 4;

	if (rxbuf != NULL) {
		qspi_op_read_init(qspi, readcmd, addrlen);
		if (nquad != 0) {
			if (qspi->rx_chan) {
				wokoo_qspi_dma_read(qspi, sf_addr, (uint32_t *)rxbuf, nquad);
			} else {
				wokoo_qspi_read(qspi->qspi_regs, sf_addr, (uint32_t *)rxbuf, nquad);
			}
		}

		/* data format 0x112233ff */
		if (ntail) {
			rdata = rxbuf + 4 * nquad;
			wokoo_qspi_read(qspi->qspi_regs, sf_addr + nquad * 4, &tail, 1);
			for (i = 0; i < ntail; i++) {
				*(rdata + i) = (tail & (0xff << (i * 8))) >> (8 * i);
			}
		}
	}
}

/*
 * qspi_op_write_init - qspi write init
 *
 * @qspi: regs
 * @writecmd: write command
 * @addrlen: address bytes
 */
static void qspi_op_write_init(struct wokoo_qspi *qspi, uint8_t writecmd, uint32_t addrlen)
{
	u32 reg;

	if (writecmd != qspi->cur_writecmd) {
		wokoo_qspi_wait_idle(qspi->qspi_regs);
		reg = readl(&qspi->qspi_regs->DEVSZ) & ~QSPI_DEVSZ_ADDRESS_BYTES_MSK;
		reg |= (addrlen - 1);
		writel(reg, &qspi->qspi_regs->DEVSZ);
		reg = 0;

		if (writecmd == SPINOR_OP_PP_1_1_4 ||
			writecmd == SPINOR_OP_PP_1_4_4 ||
			writecmd == SPINOR_OP_PP_1_1_4_4B ||
			writecmd == SPINOR_OP_PP_1_4_4_4B) {
			reg |= (0x02 << QSPI_DEVWR_DATA_TRANSFER_TYPE_POS);
			if (writecmd == SPINOR_OP_PP_1_4_4 ||
				writecmd == SPINOR_OP_PP_1_4_4_4B) {
				reg |= (0x02 << QSPI_DEVWR_ADDRESS_TRANSFER_TYPE_POS);
			}
		} else {
			if (writecmd == SPINOR_OP_PP ||
				writecmd == SPINOR_OP_PP_4B) {
				reg |= (0x00 << QSPI_DEVWR_DATA_TRANSFER_TYPE_POS);
			}
		}

		reg |= (0x00 << QSPI_DEVRD_WRITE_INSTRUCTION_POS);
		reg |= (writecmd << QSPI_DEVWR_WRITE_OPCODE_POS);
		writel(reg, &qspi->qspi_regs->DEVWR);

		qspi->cur_writecmd = writecmd;
	}
}

/*
 * wokoo_qspi_op_write - qspi write data
 *
 * @regs: regs
 * @sf_addr: offset address
 * @rxbuf: data buffer
 * @len: data length
 */
static void wokoo_qspi_op_write(struct wokoo_qspi *qspi, uint8_t writecmd,
								uint32_t sf_addr, uint32_t addrlen, uint8_t *txbuf, u32 len)
{
	u32 nquad, ntail, tail, i;
	uint8_t *tdata;

	nquad = len / 4;
	ntail = len % 4;

	if (txbuf != NULL) {
		qspi_op_write_init(qspi, writecmd, addrlen);
		if (nquad != 0) {
			if (qspi->tx_chan) {
				wokoo_qspi_dma_write(qspi, sf_addr, (uint32_t *)txbuf, nquad);
			} else {
				wokoo_qspi_write(qspi->qspi_regs, sf_addr, (uint32_t *)txbuf, nquad);
			}
		}

		/* data format 0x112233ff */
		if (ntail) {
			tail  = 0;
			tdata = txbuf + 4 * nquad;
			for (i = 0; i < ntail; i++) {
				tail |= (*(tdata + i) << (i * 8));
			}

			for (; i < 4; i++) {
				tail |= 0xff << (i * 8);
			}

			wokoo_qspi_write(qspi->qspi_regs, sf_addr + nquad * 4, &tail, 1);
		}
	}
}

/**
 * wokoo_qspi_setup - Configure the QSPI controller
 * @spi:	Pointer to the spi_device structure
 *
 * Sets the operational mode of QSPI controller for the next QSPI transfer, baud
 * rate and divisor value to setup the requested qspi clock.
 *
 * Return:	0 on success and error value on failure
 */
static int wokoo_qspi_setup(struct spi_device *spi)
{
	struct wokoo_qspi	*qspi = spi_master_get_devdata(spi->master);

	if (spi->master->busy) {
		dev_dbg(qspi->dev, "master busy doing other transfers\n");
		return -EBUSY;
	}

	wokoo_qspi_direct_quad_init(qspi->qspi_regs);
	return 0;
}

/**
 * wokoo_qspi_irq - Interrupt service routine of the QSPI
 * controller
 * @irq:	IRQ number
 * @dev_id:	Pointer to the xqspi structure
 *
 * This function handles TX empty only.
 * On TX empty interrupt this function reads the received data from RX FIFO and
 * fills the TX FIFO if there is any data remaining to be transferred.
 *
 * Return:	IRQ_HANDLED when interrupt is handled; IRQ_NONE otherwise.
 */
static irqreturn_t wokoo_qspi_irq(int irq, void *dev_id)
{
	return IRQ_NONE;
}

/**
 * wokoo_qspi_exec_mem_op() - Initiates the QSPI transfer
 * @mem: the SPI memory
 * @op: the memory operation to execute
 *
 * Executes a memory operation.
 *
 * This function first selects the chip and starts the memory operation.
 *
 * Return: 0 in case of success, a negative error code otherwise.
 */
static int wokoo_qspi_exec_mem_op(struct spi_mem *mem,
					const struct spi_mem_op *op)
{
	struct wokoo_qspi *qspi = spi_controller_get_devdata(mem->spi->master);
	int err = 0;

	dev_dbg(qspi->dev, "cmd:%#x mode:%d.%d.%d.%d\n",
		op->cmd.opcode, op->cmd.buswidth, op->addr.buswidth,
		op->dummy.buswidth, op->data.buswidth);

	if (op->addr.nbytes && op->data.nbytes && op->data.dir == SPI_MEM_DATA_IN) {
		wokoo_qspi_op_read(qspi, op->cmd.opcode, op->addr.val, op->addr.nbytes,
						   (uint8_t *)op->data.buf.in, op->data.nbytes);
	} else if (op->addr.nbytes && op->data.nbytes && op->data.dir == SPI_MEM_DATA_OUT) {
		wokoo_qspi_op_write(qspi, op->cmd.opcode, op->addr.val, op->addr.nbytes,
							(uint8_t *)op->data.buf.out, op->data.nbytes);
	} else if (op->cmd.opcode && op->data.dir == SPI_MEM_DATA_IN) {
		wokoo_qspi_cmd_read(qspi->qspi_regs, op->cmd.opcode, op->data.buf.in, op->data.nbytes);
	} else if (op->cmd.opcode && op->data.dir == SPI_MEM_DATA_OUT) {
		wokoo_qspi_cmd_write(qspi->qspi_regs, op->cmd.opcode, op->addr.val, op->addr.nbytes,
								 (uint8_t *)op->data.buf.out, op->data.nbytes);
	} else if (op->cmd.opcode && op->data.dir == SPI_MEM_NO_DATA) {
		wokoo_qspi_cmd_write(qspi->qspi_regs, op->cmd.opcode, op->addr.val, op->addr.nbytes,
								 (uint8_t *)op->data.buf.out, op->data.nbytes);
	}

	return err;
}

static const struct spi_controller_mem_ops wokoo_qspi_mem_ops = {
	.exec_op = wokoo_qspi_exec_mem_op,
};

static const struct of_device_id wokoo_qspi_match[] = {
	{.compatible = "wokoo,wokoo-qspi" },
	{},
};
MODULE_DEVICE_TABLE(of, wokoo_qspi_match);

/**
 * wokoo_qspi_dma_init - DMA init for the QSPI driver
 * @pdev:	Pointer to the platform_device structure
 * @spi:	Pointer to the spi_device structure
 *
 * This function initializes the driver data structures and the hardware.
 *
 * Return:	0 on success and error value on failure
 */
static int wokoo_qspi_dma_init(struct platform_device *pdev, struct wokoo_qspi *qspi)
{
	int  ret = -1;
	struct device *dev = &pdev->dev;
	struct dma_slave_config config;

	qspi->rx_chan = dma_request_slave_channel(dev, "rx");
	if (!qspi->rx_chan) {
		dev_err(dev, "qspi rx dma request failed\n");
	}

	qspi->tx_chan = dma_request_slave_channel(dev, "tx");
	if (!qspi->tx_chan) {
		dev_dbg(dev, "qspi tx dma request failed\n");
	}

	/* Alloc buffer for DMA pool */
	qspi->rx_bb_addr = dma_alloc_coherent(&pdev->dev, QSPI_DMA_BUFFER_SIZE,
										  &qspi->rx_bb_dma_addr, GFP_DMA);
	if (!qspi->rx_bb_addr) {
		dev_err(dev, "qspi rx dma alloc failed\n");
		dma_release_channel(qspi->rx_chan);
		qspi->rx_chan = NULL;
	}

	qspi->tx_bb_addr = dma_alloc_coherent(&pdev->dev, QSPI_DMA_BUFFER_SIZE,
										  &qspi->tx_bb_dma_addr, GFP_DMA);
	if (!qspi->tx_bb_addr) {
		dev_dbg(dev, "qspi tx dma alloc failed\n");
		dma_release_channel(qspi->tx_chan);
		qspi->tx_chan = NULL;
	}

	config.dst_addr = WOKOO_QSPI_MAP_ADDR;
	config.dst_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	config.src_addr = WOKOO_QSPI_MAP_ADDR;
	config.src_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;

	if (qspi->rx_chan) {
		ret = dmaengine_slave_config(qspi->rx_chan, &config);
		if (ret < 0) {
			dev_err(dev, "qspi rx dma channel config failed\n");
			dma_free_coherent(&pdev->dev,
							  QSPI_DMA_BUFFER_SIZE, qspi->rx_bb_addr,
							  qspi->rx_bb_dma_addr);
			dma_release_channel(qspi->rx_chan);
			qspi->rx_chan = NULL;
		}
	}

	if (qspi->tx_chan) {
		ret = dmaengine_slave_config(qspi->tx_chan, &config);
		if (ret < 0) {
			dev_dbg(dev, "qspi tx dma channel config failed\n");
			dma_free_coherent(&pdev->dev,
							  QSPI_DMA_BUFFER_SIZE, qspi->tx_bb_addr,
							  qspi->tx_bb_dma_addr);
			dma_release_channel(qspi->tx_chan);
			qspi->tx_chan = NULL;
		}
	}

	/* DMA wait for completion init */
	init_completion(&qspi->transfer_complete);
	return ret;
}

/**
 * wokoo_qspi_probe - Probe method for the QSPI driver
 * @pdev:	Pointer to the platform_device structure
 *
 * This function initializes the driver data structures and the hardware.
 *
 * Return:	0 on success and error value on failure
 */
static int wokoo_qspi_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct spi_controller *ctlr;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct wokoo_qspi *qspi;

	ctlr = spi_alloc_master(&pdev->dev, sizeof(*qspi));
	if (!ctlr)
		return -ENOMEM;

	qspi = spi_controller_get_devdata(ctlr);
	qspi->dev = dev;
	platform_set_drvdata(pdev, qspi);
	qspi->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(qspi->base)) {
		ret = PTR_ERR(qspi->base);
		goto remove_master;
	}

	qspi->pclk = devm_clk_get(&pdev->dev, "pclk");
	if (IS_ERR(qspi->pclk)) {
		dev_err(&pdev->dev, "pclk clock not found.\n");
		ret = PTR_ERR(qspi->pclk);
		goto remove_master;
	}

	init_completion(&qspi->transfer_complete);

	ret = clk_prepare_enable(qspi->pclk);
	if (ret) {
		dev_err(&pdev->dev, "Unable to enable pclk clock.\n");
		goto remove_master;
	}

	qspi->qspi_regs = (struct wokoo_qspi_regs *)qspi->base;

	/* Need iomap for WOKOO_QSPI_MAP_ADDR */
	qspi_map_addr = ioremap(WOKOO_QSPI_MAP_ADDR, WOKOO_QSPI_MAP_SIZE);

	qspi->irq = platform_get_irq(pdev, 0);
	if (qspi->irq <= 0) {
		ret = -ENXIO;
		goto clk_dis_pclk;
	}
	ret = devm_request_irq(&pdev->dev, qspi->irq, wokoo_qspi_irq,
			       0, pdev->name, qspi);
	if (ret != 0) {
		ret = -ENXIO;
		dev_err(&pdev->dev, "request_irq failed\n");
		goto clk_dis_pclk;
	}

	ret = wokoo_qspi_dma_init(pdev, qspi);

	ctlr->num_chipselect = 1;
	ctlr->mode_bits =  SPI_RX_DUAL | SPI_RX_QUAD |
				SPI_TX_DUAL | SPI_TX_QUAD;
	ctlr->mem_ops = &wokoo_qspi_mem_ops;
	ctlr->setup   = wokoo_qspi_setup;
	ctlr->dev.of_node = np;

	ret = devm_spi_register_controller(&pdev->dev, ctlr);
	if (ret) {
		dev_err(&pdev->dev, "spi_register_master failed\n");
		goto clk_dis_pclk;
	}

	return ret;

clk_dis_pclk:
	clk_disable_unprepare(qspi->pclk);
remove_master:
	spi_controller_put(ctlr);

	return ret;
}

/**
 * wokoo_qspi_remove - Remove method for the QSPI driver
 * @pdev:	Pointer to the platform_device structure
 *
 * This function is called if a device is physically removed from the system or
 * if the driver module is being unloaded. It frees all resources allocated to
 * the device.
 *
 * Return:	0 on success and error value on failure
 */
static int wokoo_qspi_remove(struct platform_device *pdev)
{
	struct wokoo_qspi *qspi = platform_get_drvdata(pdev);

	clk_disable_unprepare(qspi->pclk);

	if (qspi->tx_chan) {
		dma_free_coherent(&pdev->dev,
						  QSPI_DMA_BUFFER_SIZE, qspi->tx_bb_addr,
						  qspi->tx_bb_dma_addr);
		dma_release_channel(qspi->tx_chan);
	}

	if (qspi->rx_chan) {
		dma_free_coherent(&pdev->dev,
						  QSPI_DMA_BUFFER_SIZE, qspi->rx_bb_addr,
						  qspi->rx_bb_dma_addr);
		dma_release_channel(qspi->rx_chan);
	}

	return 0;
}

#ifdef CONFIG_PM
static int wokoo_qspi_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct wokoo_qspi *qspi = platform_get_drvdata(pdev);

	clk_disable_unprepare(qspi->pclk);
	return 0;
}

static int wokoo_qspi_resume(struct platform_device *pdev)
{
	int  ret = -1;
	struct wokoo_qspi *qspi = platform_get_drvdata(pdev);

	ret = clk_prepare_enable(qspi->pclk);
	if (ret != 0) {
		dev_err(&pdev->dev, "failed to enable qspi clock: %d\n", ret);
		return ret;
	}

	return ret;
}
#endif

static struct platform_driver wokoo_qspi_driver = {
	.probe   = wokoo_qspi_probe,
	.remove  = wokoo_qspi_remove,
#ifdef CONFIG_PM
	.suspend = wokoo_qspi_suspend,
	.resume  = wokoo_qspi_resume,
#endif
	.driver = {
		.name = "wokoo-qspi",
		.of_match_table = wokoo_qspi_match,
	}
};

module_platform_driver(wokoo_qspi_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("WOKOO QSPI controller driver");
MODULE_ALIAS("platform:wokoo-qspi");

