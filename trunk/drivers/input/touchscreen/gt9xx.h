#ifndef GT9XX_H
#define GT9XX_H

#define MAX_SUPPORT_POINTS		5			/* 5点触摸 */
//#define GT9XXADDRESS    0XBA
#define GT9XXADDRESS    0X5D

#define COMMAND_REG	  0x8040        //复位寄存器
#define GT_CFGS_REG	  0x8047        //寄存器组起始地址
#define GT_CRC_REG    0x80FF        //CRC寄存器
#define GT_Fresh_REG  0x8100        //
#define GT_PID_REG    0x8140        //产品ID寄存器

#define GTP_COOR_REG	0x814E  //触摸点寄存器


#endif