/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __WOKOO_KEY_H__
#define __WOKOO_KEY_H__

/************************************************************************/
#define AES_START_REG			0x00

#define AES_CTL_REG				0x04
#define AES_CTL_IN_TYPE			0x0008
#define AES_CTL_ENCRYPT			0x0004
#define AES_CTL_MODE			0x0003

#define AES_INTR_REG			0x08
#define AES_INTE_REG			0x0C
#define AES_INTS_REG			0x10
#define AES_KEY_LEN_REG			0x14
#define AES_KEY_UPDATE_REG		0x18
#define AES_BLK_NUM_REG			0x1C
#define AES_SADDR_REG			0x20
#define AES_CUR_SAR_REG			0x24
#define AES_TADDR_REG			0x30
#define AES_CUR_TAR_REG			0x34
#define AES_IN0_REG				0x40
#define AES_IN1_REG				0x44
#define AES_IN2_REG				0x48
#define AES_IN3_REG				0x4C
#define AES_OUT0_REG			0x50
#define AES_OUT1_REG			0x54
#define AES_OUT2_REG			0x58
#define AES_OUT3_REG			0x5C
#define AES_KEY0_REG			0x60
#define AES_KEY1_REG			0x64
#define AES_KEY2_REG			0x68
#define AES_KEY3_REG			0x6C
#define AES_KEY4_REG			0x70
#define AES_KEY5_REG			0x74
#define AES_KEY6_REG			0x78
#define AES_KEY7_REG			0x7C
#define AES_KEY_MASK0_REG		0x80
#define AES_KEY_MASK1_REG		0x84
#define AES_KEY_MASK2_REG		0x88
#define AES_KEY_MASK3_REG		0x8C
#define AES_KEY_MASK4_REG		0x90
#define AES_KEY_MASK5_REG		0x94
#define AES_KEY_MASK6_REG		0x98
#define AES_KEY_MASK7_REG		0x9C
#define AES_LP_CTRL_REG			0xA0

/************************************************************************/
#define SHA_CTL_REG				0x00
#define SHA_CTL_CAL_STOP		0x0004
#define SHA_CTL_RE_START		0x0002
#define SHA_CTL_CAL_START		0x0001

#define SHA_IN_TYPE_REG			0x04
#define SHA_MOD_REG				0x08
#define SHA_INTR_REG			0x0C
#define SHA_INTE_REG			0x10
#define SHA_INTS_REG			0x14
#define SHA_SADDR_REG			0x18
#define SHA_CUR_SAR_REG			0x1C
#define SHA_LINE_NUM_REG		0x20
#define SHA_W0_REG				0x24
#define SHA_W1_REG				0x28
#define SHA_W2_REG				0x2C
#define SHA_W3_REG				0x30
#define SHA_W4_REG				0x34
#define SHA_W5_REG				0x38
#define SHA_W6_REG				0x3C
#define SHA_W7_REG				0x40
#define SHA_W8_REG				0x44
#define SHA_W9_REG				0x48
#define SHA_W10_REG				0x4C
#define SHA_W11_REG				0x50
#define SHA_W12_REG				0x54
#define SHA_W13_REG				0x58
#define SHA_W14_REG				0x5C
#define SHA_W15_REG				0x60
#define SHA_H0_REG				0x64
#define SHA_H1_REG				0x68
#define SHA_H2_REG				0x6C
#define SHA_H3_REG				0x70
#define SHA_H4_REG				0x74
#define SHA_H5_REG				0x78
#define SHA_H6_REG				0x7C
#define SHA_H7_REG				0x80
#define SHA_LP_CTRL_REG			0x84

/************************************************************************/
#define UAC_REG_OFFSET  0

/************************************************************************/
#define HRNG_REG_OFFSET  0//0x8000

#define HRNG_CTRL_REG	0x00 + HRNG_REG_OFFSET
#define HRNG_CTRL_FIX_TEST_EN3  0x8000
#define HRNG_CTRL_FIX_TEST_EN2	0x4000
#define HRNG_CTRL_FIX_TEST_EN1	0x2000
#define HRNG_CTRL_FIX_TEST_EN0	0x1000
#define HRNG_CTRL_LOOP_TEST_EN3	0x0800
#define HRNG_CTRL_LOOP_TEST_EN2	0x0400
#define HRNG_CTRL_LOOP_TEST_EN1	0x0200
#define HRNG_CTRL_LOOP_TEST_EN0	0x0100
#define HRNG_CTRL_FREQ_TEST_EN	0x0080
#define HRNG_CTRL_SCLK_SEL		0x0040
#define HRNG_CTRL_RNG_EN3		0x0008
#define HRNG_CTRL_RNG_EN2		0x0004
#define HRNG_CTRL_RNG_EN1		0x0002
#define HRNG_CTRL_RNG_EN0		0x0001

#define HRNG_LFSR_REG	0x04 + HRNG_REG_OFFSET
#define HRNG_CMPRES_REG	0x08 + HRNG_REG_OFFSET

#define HRNG_STATUS_REG	0x0C + HRNG_REG_OFFSET
#define HRNG_STATUS_FIX_TEST_FAIL3	0x80000
#define HRNG_STATUS_FIX_TEST_FAIL2	0x40000
#define HRNG_STATUS_FIX_TEST_FAIL1	0x20000
#define HRNG_STATUS_FIX_TEST_FAIL0	0x10000
#define HRNG_STATUS_LOOP_TEST_FAIL3	0x08000
#define HRNG_STATUS_LOOP_TEST_FAIL2	0x04000
#define HRNG_STATUS_LOOP_TEST_FAIL1	0x02000
#define HRNG_STATUS_LOOP_TEST_FAIL0	0x01000
#define HRNG_STATUS_FIX_TEST_DONE3	0x00800
#define HRNG_STATUS_FIX_TEST_DONE2	0x00400
#define HRNG_STATUS_FIX_TEST_DONE1	0x00200
#define HRNG_STATUS_FIX_TEST_DONE0	0x00100
#define HRNG_STATUS_LOOP_TEST_DONE3	0x00080
#define HRNG_STATUS_LOOP_TEST_DONE2	0x00040
#define HRNG_STATUS_LOOP_TEST_DONE1	0x00020
#define HRNG_STATUS_LOOP_TEST_DONE0	0x00010
#define HRNG_STATUS_FREQ_TEST_FAIL	0x00004
#define HRNG_STATUS_FIFO_FULL		0x00002
#define HRNG_STATUS_FIFO_NOT_EMPTY	0x00001

/************************************************************************/
#define SM2_DIR_KEY		0x9C
#define SM2_DIR_ENCRYPT 1
#define SM2_DIR_DECRYPT 0
#define SM2_DIR_SIGN    0x9A
#define SM2_DIR_VERIFY  0x9B

#define PKI_REG_OFFSET  0//0x10000

#define PKI_ASRAM_REG		0x00 + PKI_REG_OFFSET
#define PKI_ASRAM_RST_REG	0x80 + PKI_REG_OFFSET
#define PKI_BSRAM_REG		0x04 + PKI_REG_OFFSET
#define PKI_BSRAM_RST_REG	0x84 + PKI_REG_OFFSET
#define PKI_NSRAM_REG		0x08 + PKI_REG_OFFSET
#define PKI_NSRAM_RST_REG	0x88 + PKI_REG_OFFSET
#define PKI_RSRAM_REG		0x0C + PKI_REG_OFFSET
#define PKI_RSRAM_RST_REG	0x8C + PKI_REG_OFFSET
#define PKI_TSRAM_REG		0x10 + PKI_REG_OFFSET
#define PKI_TSRAM_RST_REG	0x90 + PKI_REG_OFFSET
#define PKI_X0SRAM_REG		0x14 + PKI_REG_OFFSET
#define PKI_X0SRAM_RST_REG	0x94 + PKI_REG_OFFSET
#define PKI_Y0SRAM_REG		0x18 + PKI_REG_OFFSET
#define PKI_Y0SRAM_RST_REG	0x98 + PKI_REG_OFFSET
#define PKI_X1SRAM_REG		0x1C + PKI_REG_OFFSET
#define PKI_X1SRAM_RST_REG	0x9C + PKI_REG_OFFSET
#define PKI_Y1SRAM_REG		0x20 + PKI_REG_OFFSET
#define PKI_Y1SRAM_RST_REG	0xA0 + PKI_REG_OFFSET
#define PKI_aSRAM_REG		0x24 + PKI_REG_OFFSET
#define PKI_aSRAM_RST_REG	0xA4 + PKI_REG_OFFSET
#define PKI_kSRAM_REG		0x28 + PKI_REG_OFFSET
#define PKI_kSRAM_RST_REG	0xA8 + PKI_REG_OFFSET
#define PKI_bSRAM_REG		0x2C + PKI_REG_OFFSET
#define PKI_bSRAM_RST_REG	0xAC + PKI_REG_OFFSET

#define PKI_CMD_REG		0x30 + PKI_REG_OFFSET
#define PKI_CMD_INTER_EN	0x0100
#define PKI_CMD_PARITY_EN	0x0080

#define PKI_STATUS_REG	0x34 + PKI_REG_OFFSET
#define PKI_STATUS_FA_ERROR	0x0010	
#define PKI_STATUS_JUDGE	0x0008
#define PKI_STATUS_PC_ERROR	0x0004
#define PKI_STATUS_EVEN		0x0002
#define PKI_STATUS_DONE		0x0001

#define PKI_OPERATION_REG	0x38 + PKI_REG_OFFSET
#define PKI_RSA_REG			0x40 + PKI_REG_OFFSET
#define PKI_ECCX_REG		0x44 + PKI_REG_OFFSET
#define PKI_ECCY_REG		0x48 + PKI_REG_OFFSET
#define PKI_MOD_REG			0x4C + PKI_REG_OFFSET
#define PKI_ECCZ_REG		0x50 + PKI_REG_OFFSET
#define PKI_ECCX1_REG		0x54 + PKI_REG_OFFSET
#define PKI_ECCY1_REG		0x58 + PKI_REG_OFFSET
#define PKI_ECCZ1_REG		0x5C + PKI_REG_OFFSET
#define PKI_Z0SRAM_REG		0x60 + PKI_REG_OFFSET
#define PKI_Z0RST_REG		0xE0 + PKI_REG_OFFSET
#define PKI_Y2SRAM_REG		0x64 + PKI_REG_OFFSET
#define PKI_Y2RST_REG		0xE4 + PKI_REG_OFFSET
#define PKI_Z1SRAM_REG		0x68 + PKI_REG_OFFSET
#define PKI_Z1RST_REG		0xE8 + PKI_REG_OFFSET
#define PKI_T2SRAM_REG		0x6C + PKI_REG_OFFSET
#define PKI_T2RST_REG		0xEC + PKI_REG_OFFSET

/************************************************************************/
#define DIV_REG_OFFSET  0//0x18000

/************************************************************************/
#define SM3_REG_OFFSET  0//0x30000

#define SM3_DATAIN_REG	0x00 + SM3_REG_OFFSET
#define SM3_MIDDATA_REG	0x04 + SM3_REG_OFFSET
#define SM3_CTRL_REG	0x08 + SM3_REG_OFFSET
#define SM3_DATAOUT_REG	0x0C + SM3_REG_OFFSET

/************************************************************************/
#define GMALG_SM4_ECB 0x00000401
#define GMALG_SM4_CBC 0x00000402

#define SM4_DIR_ENCRYPT 1
#define SM4_DIR_DECRYPT 0

#define SM4_REG_OFFSET  0//0x38000

#define SM4_DATAIN_REG	0x00 + SM4_REG_OFFSET
#define SM4_KEYIN_REG	0x04 + SM4_REG_OFFSET
#define SM4_IVIN_REG	0x08 + SM4_REG_OFFSET
#define SM4_DATAOUT_REG	0x14 + SM4_REG_OFFSET

#define SM4_CONTROL_REG	0x0C + SM4_REG_OFFSET
#define SM4_CONTROL_MASK_EN		0x0100
#define SM4_CONTROL_SBox_INIT	0x0080
#define SM4_CONTROL_VSM4_EN		0x0040
#define SM4_CONTROL_CBC			0x0020
#define SM4_CONTROL_SWAP		0x0010
#define SM4_CONTROL_INT_EN		0x0008
#define SM4_CONTROL_CRYPT		0x0004
#define SM4_CONTROL_KEY_START	0x0002
#define SM4_CONTROL_CRYPT_START	0x0001

#define SM4_STATE_REG	0x10 + SM4_REG_OFFSET
#define SM4_STATE_SBOX_DONE		0x0008
#define SM4_STATE_CBC_DONE		0x0004
#define SM4_STATE_KEY_DONE		0x0002
#define SM4_STATE_CRYPT_DONE	0x0001

#define SM4_DATAOUT_REG	0x14 + SM4_REG_OFFSET

/************************************************************************/
#define BP147_SR_SIZE_REG		0x000
#define BP147_DMAG_STAT_REG		0x818
#define BP147_DMAG_SET_REG		0x81C
#define BP147_DMAG_CLR_REG		0x820

#define PKI_CMD_FUNC_CLEAR_INTERRUPT					0x00
#define PKI_CMD_FUNC_MODULAR_MULTIPLE					0x01
#define PKI_CMD_FUNC_MODULAR_ADDITIOn					0x02
#define PKI_CMD_FUNC_MODULAR_SQUARE						0x03
#define PKI_CMD_FUNC_MODULAR_SUBTRACTION				0x04

#define PKI_CMD_FUNC_MODULAR_INVERSION					0x10
#define PKI_CMD_FUNC_A_AXA								0x12
#define PKI_CMD_FUNC_R_AXA								0x13
#define PKI_CMD_FUNC_A_A_R								0x14
#define PKI_CMD_FUNC_B_A_B								0x15
#define PKI_CMD_FUNC_R_R_R								0x16
#define PKI_CMD_FUNC_B_R_B								0x17
#define PKI_CMD_FUNC_A_CONVERT_TO_NORMAL_NUMBER			0x18
#define PKI_CMD_FUNC_B_CONVERT_TO_NORMAL_NUMBER			0x19
#define PKI_CMD_FUNC_R_CONVERT_TO_NORMAL_NUMBER			0x1A
#define PKI_CMD_FUNC_CONVERT_TO_MONTGOMERY_NUMBER		0x1C

#define PKI_CMD_FUNC_PRE_CALCULATION					0x20
#define PKI_CMD_FUNC_POINT_SCALAR_MULTIPLE				0x21
#define PKI_CMD_FUNC_POINT_DOUBLE						0x22
#define PKI_CMD_FUNC_POINT_ADDITION						0x24
#define PKI_CMD_FUNC_POINT_SUBTRACTION					0x28
#define PKI_CMD_FUNC_POINT_JUDGE						0x2A

#define PKI_ECC_REG_STATUS_DONE							(0x1UL << 0)
#define PKI_ECC_REG_STATUS_EVEN							(0x1UL << 1)
#define PKI_ECC_REG_STATUS_PC_ERROR						(0x1UL << 2)
#define PKI_ECC_REG_STATUS_JUDGE						(0x1UL << 3)
#define PKI_ECC_REG_STATUS_FA_ERROR						(0x1UL << 4)



#define SEC_CF_SR										(0x1UL << 0)
#define SEC_CF_SA										(0x1UL << 1)
#define SEC_CF_ST										(0x1UL << 2)
#define SEC_CF_TPZ_LOCK									(0x1UL << 3)
#define SEC_CF_SW										(0x1UL << 4)
#define SEC_CF_MOD_RD_LOCK								(0x1UL << 5)
#define SEC_CF_KEY_RD_LOCK								(0x1UL << 6)

#define SEC_EFUSE_CTRL_LOAD_EN							(0x1UL << 0)
#define SEC_EFUSE_CTRL_UPDATE							(0x1UL << 1)

#define SEC_LPC_CTRL_APB_LPC_EN							(0x1UL << 0)
#define SEC_LPC_CTRL_EFUSE_LPC_EN						(0x1UL << 1)
#define SEC_LPC_CTRL_EFUSE_IDLE_TIME_POS				(4)
#define SEC_LPC_CTRL_EFUSE_IDLE_TIME_MSK				(0xFUL << SEC_LPC_CTRL_EFUSE_IDLE_TIME_POS)
#define SEC_LPC_CTRL_AHB_IDLE_TIME_POS					(8)
#define SEC_LPC_CTRL_AHB_IDLE_TIME_MSK					(0xFFUL << SEC_LPC_CTRL_AHB_IDLE_TIME_POS)

#define SEC_EFUSE_LOAD_INT								(0x1UL << 0)
#define SEC_NON_SECS_INT								(0x1UL << 1)
#define SEC_CONV_S0_INT									(0x1UL << 2)
#define SEC_SECS_INT									(0x1UL << 3)
#define SEC_CONV_S1_INT									(0x1UL << 4)
#define SEC_SEC_SW_PUSH_INT								(0x1UL << 5)
#define SEC_SEC_SW_POP_INT								(0x1UL << 6)

#define SEC_DBG_ACC_A7_CFGSDIS							(0x1UL << 0)
#define SEC_DBG_ACC_A7_CP15SDIS							(0x1UL << 1)
#define SEC_DBG_ACC_JTAG_SOFT_MASK						(0x1UL << 5)
#define SEC_DBG_ACC_A7_DBGEN							(0x1UL << 8)
#define SEC_DBG_ACC_A7_NIDEN							(0x1UL << 12)
#define SEC_DBG_ACC_CS_DBGEN							(0x1UL << 16)
#define SEC_DBG_ACC_CS_NIDEN							(0x1UL << 17)
#define SEC_DBG_ACC_CS_SPIDEN							(0x1UL << 18)
#define SEC_DBG_ACC_CS_SPNIDEN							(0x1UL << 19)
#define SEC_DBG_ACC_CS_A7_SPIDEN						(0x1UL << 20)
#define SEC_DBG_ACC_CS_A7_SPNIDEN						(0x1UL << 24)

#define SEC_FF_SE										(0x1UL << 0)
#define SEC_FF_ST										(0x1UL << 1)
#define SEC_FF_FL_POS									(2)
#define SEC_FF_FL_MSK									(0x3UL << SEC_FF_FL_POS)
#define SEC_FF_A7_DBG									(0x1UL << 4)


#define SEC_REG_OFFSET  0

#define CF				0x000 + SEC_REG_OFFSET
#define EFUSE_CTRL 		0x004 + SEC_REG_OFFSET	
#define MODE 			0x008 + SEC_REG_OFFSET	
#define CONV_S0_CTR		0x00C + SEC_REG_OFFSET	
#define CONV_S1_CTR		0x010 + SEC_REG_OFFSET	
#define LPC_CTRL 		0x014 + SEC_REG_OFFSET	
#define INTRAW			0x018 + SEC_REG_OFFSET	
#define INTE 			0x01C + SEC_REG_OFFSET	
#define INTS 			0x020 + SEC_REG_OFFSET	
#define DBG_ACC			0x024 + SEC_REG_OFFSET	
#define Reserved0		0x028 + SEC_REG_OFFSET	
#define APPWR_PROTECT	0x02C + SEC_REG_OFFSET	
#define CHIP_ID_0		0x080 + SEC_REG_OFFSET	
#define CHIP_ID_1		0x084 + SEC_REG_OFFSET	
#define CHIP_ID_2		0x088 + SEC_REG_OFFSET
#define Reserved2		0x08C + SEC_REG_OFFSET	
#define SEC_KEY_0		0x090 + SEC_REG_OFFSET	
#define SEC_KEY_1		0x094 + SEC_REG_OFFSET	
#define SEC_KEY_2		0x098 + SEC_REG_OFFSET	
#define SEC_KEY_3		0x09C + SEC_REG_OFFSET	
#define FF				0x0D0 + SEC_REG_OFFSET	









struct wokoo_crypto_info {
	struct device	*dev;
	struct clk 		*hclk;
	struct clk 		*sm4_clk;
	struct clk 		*sm3_clk;
	struct clk 		*pki_clk;
	struct clk 		*hrng_clk;
	struct clk 		*alg_clk;
	struct clk 		*sclk;
	int				irq;
};

void SM3_Hash(uint32_t *pDataIn, uint32_t DataLen, uint32_t *pDigest);
uint8_t get_hrng(void);

//sm2_API
void set_sm2_mode(unsigned int cmd);
void get_sm2_key(void);
void write_sm2_data(unsigned char *buffer);
void read_sm2_data(unsigned char *buffer, unsigned int size);

//sm3_API
void write_sm3_data(unsigned char *buffer, unsigned int count);
void read_sm3_data(unsigned char *buffer);

//sm4_API
void write_sm4_data(unsigned char *buffer);
void set_sm4_mode(unsigned int cmd);
void read_sm4_data(unsigned char *buffer);

#endif
