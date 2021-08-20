// SPDX-License-Identifier: GPL-2.0-only
/*
 * wokoo_lcdc.c - config lcdc 
 */

#include <linux/err.h>
#include <linux/gpio.h> 
#include <linux/gpio/consumer.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <dt-bindings/gpio/gpio.h>

#define ASIZE(a)  sizeof(a)/sizeof(a[0])
enum spi_write_type_e
{
	TYPE_CMD = 0X0,
	TYPE_DATA
};
struct wokoo_lcdc_info
{
	int reset;
	int cs;
	int mosi;

	int clk;
	enum spi_write_type_e type;
};

struct spi_write_ctrl
{
  enum spi_write_type_e type;
  char data;
  int delay;
};
 
 struct spi_write_ctrl spi_ctrl[] = 
 {
	{TYPE_CMD, 0x11,40},{TYPE_CMD,0xB0,0},{TYPE_DATA,0x04,0},{TYPE_CMD, 0xB3,0},{TYPE_DATA, 0x10,0},
    {TYPE_DATA,0x00,0},{TYPE_DATA,0x00,0},{TYPE_CMD, 0xB6,0},{TYPE_DATA,0x52,0},{TYPE_DATA, 0x83,0},
	{TYPE_CMD, 0xB7,0},{TYPE_DATA,0x80,0},{TYPE_DATA,0x72,0},{TYPE_DATA,0x11,0},{TYPE_DATA, 0x25,0},
	{TYPE_CMD, 0xB8,0},{TYPE_DATA,0x00,0},{TYPE_DATA,0x0F,0},{TYPE_DATA,0x0F,0},{TYPE_DATA, 0xFF,0},
	{TYPE_DATA,0xFF,0},{TYPE_DATA,0xC8,0},{TYPE_DATA,0xC8,0},{TYPE_DATA,0x02,0},{TYPE_DATA, 0x18,0},
	{TYPE_DATA,0x10,0},{TYPE_DATA,0x10,0},{TYPE_DATA,0x37,0},{TYPE_DATA,0x5A,0},{TYPE_DATA, 0x87,0},
    {TYPE_DATA,0xBE,0},{TYPE_DATA,0xFF,0},{TYPE_DATA,0x00,0},{TYPE_DATA,0x00,0},{TYPE_DATA, 0x00,0},
	{TYPE_DATA,0x00,0},{TYPE_CMD, 0xB9,0},{TYPE_DATA,0x00,0},{TYPE_DATA,0x00,0},{TYPE_DATA, 0x00,0},
	{TYPE_DATA,0x00,0},{TYPE_CMD, 0xBD,0},{TYPE_DATA,0x00,0},{TYPE_CMD, 0xC0,0},{TYPE_DATA, 0x02,0},
	{TYPE_DATA,0x76,0},{TYPE_CMD, 0xC1,0},{TYPE_DATA,0x63,0},{TYPE_DATA,0x31,0},{TYPE_DATA, 0x00,0},
	{TYPE_DATA,0x27,0},{TYPE_DATA,0x27,0},{TYPE_DATA,0x32,0},{TYPE_DATA,0x12,0},{TYPE_DATA, 0x28,0},
	{TYPE_DATA,0x4E,0},{TYPE_DATA,0x10,0},{TYPE_DATA,0xA5,0},{TYPE_DATA,0x0F,0},{TYPE_DATA, 0x58,0},
	{TYPE_DATA,0x21,0},{TYPE_DATA,0x01,0},{TYPE_CMD, 0xC2,0},{TYPE_DATA,0x28,0},{TYPE_DATA, 0x06,0},
	{TYPE_DATA,0x01,0},{TYPE_DATA,0x03,0},{TYPE_DATA,0x00,0},{TYPE_CMD, 0xC3,0},{TYPE_DATA, 0x40,0},
	{TYPE_DATA,0x00,0},{TYPE_DATA,0x03,0},{TYPE_CMD, 0xC4,0},{TYPE_DATA,0x00,0},{TYPE_DATA, 0x01,0},
	{TYPE_CMD, 0xC6,0},{TYPE_DATA,0x00,0},{TYPE_DATA,0x00,0},{TYPE_CMD, 0xC7,0},{TYPE_DATA, 0x11,0},
	{TYPE_DATA,0x8D,0},{TYPE_DATA,0xA0,0},{TYPE_DATA,0xF5,0},{TYPE_DATA,0x27,0},{TYPE_CMD,  0xC8,0},
	{TYPE_DATA,0x02,0},{TYPE_DATA,0x13,0},{TYPE_DATA,0x18,0},{TYPE_DATA,0x25,0},{TYPE_DATA, 0x34,0},
	{TYPE_DATA,0x4E,0},{TYPE_DATA,0x36,0},{TYPE_DATA,0x23,0},{TYPE_DATA,0x17,0},{TYPE_DATA, 0x0E,0},
	{TYPE_DATA,0x0C,0},{TYPE_DATA,0x02,0},{TYPE_DATA,0x02,0},{TYPE_DATA,0x13,0},{TYPE_DATA, 0x18,0},
	{TYPE_DATA,0x25,0},{TYPE_DATA,0x34,0},{TYPE_DATA,0x4E,0},{TYPE_DATA,0x36,0},{TYPE_DATA, 0x23,0},
	{TYPE_DATA,0x17,0},{TYPE_DATA,0x0E,0},{TYPE_DATA,0x0C,0},{TYPE_DATA,0x02,0},{TYPE_CMD,  0xC9,0},
	{TYPE_DATA,0x02,0},{TYPE_DATA,0x13,0},{TYPE_DATA,0x18,0},{TYPE_DATA,0x25,0},{TYPE_DATA, 0x34,0},
	{TYPE_DATA,0x4E,0},{TYPE_DATA,0x36,0},{TYPE_DATA,0x23,0},{TYPE_DATA,0x17,0},{TYPE_DATA, 0x0E,0},
	{TYPE_DATA,0x0C,0},{TYPE_DATA,0x02,0},{TYPE_DATA,0x02,0},{TYPE_DATA,0x13,0},{TYPE_DATA, 0x18,0},
	{TYPE_DATA,0x25,0},{TYPE_DATA,0x34,0},{TYPE_DATA,0x4E,0},{TYPE_DATA,0x36,0},{TYPE_DATA, 0x23,0},
	{TYPE_DATA,0x17,0},{TYPE_DATA,0x0E,0},{TYPE_DATA,0x0C,0},{TYPE_DATA,0x02,0},{TYPE_CMD,  0xCA,0},	
	{TYPE_DATA,0x02,0},{TYPE_DATA,0x13,0},{TYPE_DATA,0x18,0},{TYPE_DATA,0x25,0},{TYPE_DATA, 0x34,0},
	{TYPE_DATA,0x4E,0},{TYPE_DATA,0x36,0},{TYPE_DATA,0x23,0},{TYPE_DATA,0x17,0},{TYPE_DATA, 0x0E,0},
	{TYPE_DATA,0x0C,0},{TYPE_DATA,0x02,0},{TYPE_DATA,0x02,0},{TYPE_DATA,0x13,0},{TYPE_DATA, 0x18,0},
	{TYPE_DATA,0x25,0},{TYPE_DATA,0x34,0},{TYPE_DATA,0x4E,0},{TYPE_DATA,0x36,0},{TYPE_DATA, 0x23,0},
	{TYPE_DATA,0x17,0},{TYPE_DATA,0x0E,0},{TYPE_DATA,0x0C,0},{TYPE_DATA,0x02,0},{TYPE_CMD,  0xD0,0},	
	{TYPE_DATA,0xA9,0},{TYPE_DATA,0x03,0},{TYPE_DATA,0xCC,0},{TYPE_DATA,0xA5,0},{TYPE_DATA, 0x00,0},
	{TYPE_DATA,0x53,0},{TYPE_DATA,0x20,0},{TYPE_DATA,0x10,0},{TYPE_DATA,0x01,0},{TYPE_DATA, 0x00,0},
	{TYPE_DATA,0x01,0},{TYPE_DATA,0x01,0},{TYPE_DATA,0x00,0},{TYPE_DATA,0x03,0},{TYPE_DATA, 0x01,0},
	{TYPE_DATA,0x00,0},{TYPE_CMD, 0xD1,0},{TYPE_DATA,0x18,0},{TYPE_DATA,0x0C,0},{TYPE_DATA, 0x23,0},
	{TYPE_DATA,0x03,0},{TYPE_DATA,0x75,0},{TYPE_DATA,0x02,0},{TYPE_DATA,0x50,0},{TYPE_CMD,  0xD3,0},
	{TYPE_DATA,0x33,0},{TYPE_CMD, 0xD5,0},{TYPE_DATA,0x2a,0},{TYPE_DATA,0x2a,0},{TYPE_CMD,  0xD6,0},
	{TYPE_DATA,0x28,0},{TYPE_CMD, 0xD7,0},{TYPE_DATA,0x01,0},{TYPE_DATA,0x00,0},{TYPE_DATA, 0xAA,0},
	{TYPE_DATA,0xC0,0},{TYPE_DATA,0x2A,0},{TYPE_DATA,0x2C,0},{TYPE_DATA,0x22,0},{TYPE_DATA, 0x12,0},
	{TYPE_DATA,0x71,0},{TYPE_DATA,0x0A,0},{TYPE_DATA,0x12,0},{TYPE_DATA,0x00,0},{TYPE_DATA, 0xA0,0},
	{TYPE_DATA,0x00,0},{TYPE_DATA,0x03,0},{TYPE_CMD, 0xD8,0},{TYPE_DATA,0x44,0},{TYPE_DATA, 0x44,0},
	{TYPE_DATA,0x22,0},{TYPE_DATA,0x44,0},{TYPE_DATA,0x21,0},{TYPE_DATA,0x46,0},{TYPE_DATA, 0x42,0},
	{TYPE_DATA,0x40,0},{TYPE_CMD, 0xD9,0},{TYPE_DATA,0xCF,0},{TYPE_DATA,0x2D,0},{TYPE_DATA, 0x51,0},
	{TYPE_CMD, 0xDA,0},{TYPE_DATA,0x01,0},{TYPE_CMD, 0xDE,0},{TYPE_DATA,0x01,0},{TYPE_DATA, 0x51,0},
	{TYPE_CMD, 0xE1,0},{TYPE_DATA,0x00,0},{TYPE_DATA,0x00,0},{TYPE_DATA,0x00,0},{TYPE_DATA, 0x00,0},
	{TYPE_DATA,0x00,0},{TYPE_DATA,0x00,0},{TYPE_CMD, 0xE6,0},{TYPE_DATA,0x55,0},{TYPE_CMD,  0xF3,0},
	{TYPE_DATA,0x06,0},{TYPE_DATA,0x00,0},{TYPE_DATA,0x00,0},{TYPE_DATA,0x24,0},{TYPE_DATA, 0x00,0},
	{TYPE_CMD, 0xF8,0},{TYPE_DATA,0x00,0},{TYPE_CMD, 0xFA,0},{TYPE_DATA,0x01,0},{TYPE_CMD,  0xFB,0},
	{TYPE_DATA,0x00,0},{TYPE_DATA,0x00,0},{TYPE_DATA,0x00,0},{TYPE_CMD, 0xFC,0},{TYPE_DATA, 0x00,0},
	{TYPE_DATA,0x00,0},{TYPE_DATA,0x00,0},{TYPE_DATA,0x00,0},{TYPE_DATA,0x00,0},{TYPE_CMD,  0xFD,0},
	{TYPE_DATA,0x00,0},{TYPE_DATA,0x00,0},{TYPE_DATA,0x70,0},{TYPE_DATA,0x00,0},{TYPE_DATA, 0x72,0},
	{TYPE_DATA,0x31,0},{TYPE_DATA,0x37,0},{TYPE_DATA,0x70,0},{TYPE_DATA,0x32,0},{TYPE_DATA, 0x31,0},
	{TYPE_DATA,0x07,0},{TYPE_DATA,0x00,0},{TYPE_DATA,0x00,0},{TYPE_CMD, 0xFE,0},{TYPE_DATA, 0x00,0},
	{TYPE_DATA,0x00,0},{TYPE_DATA,0x00,0},{TYPE_DATA,0x00,0},{TYPE_DATA,0x20,0},{TYPE_CMD,  0xB0,0},
	{TYPE_DATA,0x04,40},{TYPE_CMD,0x35,0},{TYPE_DATA,0x00,0},{TYPE_CMD, 0x44,0},{TYPE_DATA, 0x00,0},
	{TYPE_CMD, 0x36,0},{TYPE_DATA,0x00,0},{TYPE_CMD, 0x3A,0},{TYPE_DATA,0x77,0},{TYPE_CMD,  0x2A,0},
	{TYPE_DATA,0x00,0},{TYPE_DATA,0x00,0},{TYPE_DATA,0x01,0},{TYPE_DATA,0xDF,0},{TYPE_CMD,  0x2B,0},
	{TYPE_DATA,0x00,0},{TYPE_DATA,0x00,0},{TYPE_DATA,0x03,0},{TYPE_DATA,0x1F,0},{TYPE_CMD,  0x29,10},
	{TYPE_CMD, 0x2C,10},{TYPE_CMD,0x36,0},{TYPE_DATA,0x68,0},{TYPE_CMD, 0x2c,0} //0x68
 };
 
 
 
int gpio_reset,gpio_bl,gpio_sda,gpio_dclk,gpio_cs;

#define LCD_SPI_CS(a)  \
						if (a)	\
						gpio_set_value(gpio_cs, 1); 	\
						else		\
						gpio_set_value(gpio_cs, 0);
						
#define SPI_DCLK(a)	\
						if (a)	\
						gpio_set_value(gpio_dclk, 1); 	\
						else		\
						gpio_set_value(gpio_dclk, 0);
						
#define SPI_SDA(a)	\
						if (a)	\
						gpio_set_value(gpio_sda, 1);	\
						else		\
						gpio_set_value(gpio_sda, 0);


volatile void LCD_delay(volatile int time)
{
	volatile unsigned int i;	
	while(time--)
	for(i=500;i>0;i--);
}



void lcd_reset(void)
{
	gpio_set_value(gpio_reset, 0);   //pull down

	LCD_delay(300);

	gpio_set_value(gpio_reset, 1);  //pull up

	LCD_delay(300);

}

void LCD_WriteByteSPI(unsigned char byte)
{
    unsigned char n;

    for(n=0; n<8; n++)
    {
        if(byte&0x80) SPI_SDA(1)
        else SPI_SDA(0)
        byte<<= 1;
        SPI_DCLK(0);
        SPI_DCLK(1);
    }
}
void SPI_WriteComm(unsigned     int CMD)  //3线9bit 串行接口
{
	LCD_SPI_CS(0);
	SPI_SDA(0);
	SPI_DCLK(0);
	SPI_DCLK(1);
	LCD_WriteByteSPI(CMD);
	LCD_SPI_CS(1);
}
void SPI_WriteData(unsigned     int tem_data)
{
	LCD_SPI_CS(0);
	SPI_SDA(1);
	SPI_DCLK(0);
	SPI_DCLK(1);
	LCD_WriteByteSPI(tem_data);
	LCD_SPI_CS(1);
}

static void Lcd_Initialize_l(void){
	LCD_SPI_CS(1);
	LCD_delay(20);
	LCD_SPI_CS(0);
	lcd_reset();

	SPI_WriteComm(0x11);
	LCD_delay(40);
	SPI_WriteComm(0xB0);
	SPI_WriteData(0x04);

	SPI_WriteComm(0xB3);
	SPI_WriteData(0x10);//RGB=0x10;MCU=0x00;  0x02
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);

	SPI_WriteComm(0xB6);
	SPI_WriteData(0x52);
	SPI_WriteData(0x83);

	SPI_WriteComm(0xB7);
	SPI_WriteData(0x80);
	SPI_WriteData(0x72);
	SPI_WriteData(0x11);
	SPI_WriteData(0x25);

	SPI_WriteComm(0xB8);
	SPI_WriteData(0x00);
	SPI_WriteData(0x0F);
	SPI_WriteData(0x0F);
	SPI_WriteData(0xFF);
	SPI_WriteData(0xFF);
	SPI_WriteData(0xC8);
	SPI_WriteData(0xC8);
	SPI_WriteData(0x02);
	SPI_WriteData(0x18);
	SPI_WriteData(0x10);
	SPI_WriteData(0x10);
	SPI_WriteData(0x37);
	SPI_WriteData(0x5A);
	SPI_WriteData(0x87);
	SPI_WriteData(0xBE);
	SPI_WriteData(0xFF);
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);
	SPI_WriteComm(0xB9);
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);
	SPI_WriteComm(0xBD);
	SPI_WriteData(0x00);

	SPI_WriteComm(0xC0);
	SPI_WriteData(0x02);
	SPI_WriteData(0x76);

	SPI_WriteComm(0xC1);
	SPI_WriteData(0x63);
	SPI_WriteData(0x31);
	SPI_WriteData(0x00);
	SPI_WriteData(0x27);
	SPI_WriteData(0x27);
	SPI_WriteData(0x32);
	SPI_WriteData(0x12);
	SPI_WriteData(0x28);
	SPI_WriteData(0x4E);
	SPI_WriteData(0x10);
	SPI_WriteData(0xA5);
	SPI_WriteData(0x0F);
	SPI_WriteData(0x58);
	SPI_WriteData(0x21);
	SPI_WriteData(0x01);

	SPI_WriteComm(0xC2);
	SPI_WriteData(0x28);
	SPI_WriteData(0x06);
	SPI_WriteData(0x06);
	SPI_WriteData(0x01);
	SPI_WriteData(0x03);
	SPI_WriteData(0x00);

	SPI_WriteComm(0xC3);
	SPI_WriteData(0x40);
	SPI_WriteData(0x00);
	SPI_WriteData(0x03);
	SPI_WriteComm(0xC4);
	SPI_WriteData(0x00);
	SPI_WriteData(0x01);
	SPI_WriteComm(0xC6);
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);
	SPI_WriteComm(0xC7);
	SPI_WriteData(0x11);
	SPI_WriteData(0x8D);
	SPI_WriteData(0xA0);
	SPI_WriteData(0xF5);
	SPI_WriteData(0x27);
	SPI_WriteComm(0xC8);
	SPI_WriteData(0x02);
	SPI_WriteData(0x13);
	SPI_WriteData(0x18);
	SPI_WriteData(0x25);
	SPI_WriteData(0x34);
	SPI_WriteData(0x4E);
	SPI_WriteData(0x36);
	SPI_WriteData(0x23);
	SPI_WriteData(0x17);
	SPI_WriteData(0x0E);
	SPI_WriteData(0x0C);
	SPI_WriteData(0x02);
	SPI_WriteData(0x02);
	SPI_WriteData(0x13);
	SPI_WriteData(0x18);
	SPI_WriteData(0x25);
	SPI_WriteData(0x34);
	SPI_WriteData(0x4E);
	SPI_WriteData(0x36);
	SPI_WriteData(0x23);
	SPI_WriteData(0x17);
	SPI_WriteData(0x0E);
	SPI_WriteData(0x0C);
	SPI_WriteData(0x02);
	SPI_WriteComm(0xC9);
	SPI_WriteData(0x02);
	SPI_WriteData(0x13);
	SPI_WriteData(0x18);
	SPI_WriteData(0x25);
	SPI_WriteData(0x34);
	SPI_WriteData(0x4E);
	SPI_WriteData(0x36);
	SPI_WriteData(0x23);
	SPI_WriteData(0x17);
	SPI_WriteData(0x0E);
	SPI_WriteData(0x0C);
	SPI_WriteData(0x02);
	SPI_WriteData(0x02);
	SPI_WriteData(0x13);
	SPI_WriteData(0x18);
	SPI_WriteData(0x25);
	SPI_WriteData(0x34);
	SPI_WriteData(0x4E);
	SPI_WriteData(0x36);
	SPI_WriteData(0x23);
	SPI_WriteData(0x17);
	SPI_WriteData(0x0E);
	SPI_WriteData(0x0C);
	SPI_WriteData(0x02);
	SPI_WriteComm(0xCA);
	SPI_WriteData(0x02);
	SPI_WriteData(0x13);
	SPI_WriteData(0x18);
	SPI_WriteData(0x25);
	SPI_WriteData(0x34);
	SPI_WriteData(0x4E);
	SPI_WriteData(0x36);
	SPI_WriteData(0x23);
	SPI_WriteData(0x17);
	SPI_WriteData(0x0E);
	SPI_WriteData(0x0C);
	SPI_WriteData(0x02);
	SPI_WriteData(0x02);
	SPI_WriteData(0x13);
	SPI_WriteData(0x18);
	SPI_WriteData(0x25);
	SPI_WriteData(0x34);
	SPI_WriteData(0x4E);
	SPI_WriteData(0x36);
	SPI_WriteData(0x23);
	SPI_WriteData(0x17);
	SPI_WriteData(0x0E);
	SPI_WriteData(0x0C);
	SPI_WriteData(0x02);
	SPI_WriteComm(0xD0);
	SPI_WriteData(0xA9);
	SPI_WriteData(0x03);
	SPI_WriteData(0xCC);
	SPI_WriteData(0xA5);
	SPI_WriteData(0x00);
	SPI_WriteData(0x53);
	SPI_WriteData(0x20);
	SPI_WriteData(0x10);
	SPI_WriteData(0x01);
	SPI_WriteData(0x00);
	SPI_WriteData(0x01);
	SPI_WriteData(0x01);
	SPI_WriteData(0x00);
	SPI_WriteData(0x03);
	SPI_WriteData(0x01);
	SPI_WriteData(0x00);
	SPI_WriteComm(0xD1);
	SPI_WriteData(0x18);
	SPI_WriteData(0x0C);
	SPI_WriteData(0x23);
	SPI_WriteData(0x03);
	SPI_WriteData(0x75);
	SPI_WriteData(0x02);
	SPI_WriteData(0x50);
	SPI_WriteComm(0xD3);
	SPI_WriteData(0x33);
	SPI_WriteComm(0xD5);
	SPI_WriteData(0x2a);
	SPI_WriteData(0x2a);
	SPI_WriteComm(0xD6);
	SPI_WriteData(0x28);//a8
	SPI_WriteComm(0xD7);
	SPI_WriteData(0x01);
	SPI_WriteData(0x00);
	SPI_WriteData(0xAA);
	SPI_WriteData(0xC0);
	SPI_WriteData(0x2A);
	SPI_WriteData(0x2C);
	SPI_WriteData(0x22);
	SPI_WriteData(0x12);
	SPI_WriteData(0x71);
	SPI_WriteData(0x0A);
	SPI_WriteData(0x12);
	SPI_WriteData(0x00);
	SPI_WriteData(0xA0);
	SPI_WriteData(0x00);
	SPI_WriteData(0x03);
	SPI_WriteComm(0xD8);
	SPI_WriteData(0x44);
	SPI_WriteData(0x44);
	SPI_WriteData(0x22);
	SPI_WriteData(0x44);
	SPI_WriteData(0x21);
	SPI_WriteData(0x46);
	SPI_WriteData(0x42);
	SPI_WriteData(0x40);
	SPI_WriteComm(0xD9);
	SPI_WriteData(0xCF);
	SPI_WriteData(0x2D);
	SPI_WriteData(0x51);
	SPI_WriteComm(0xDA);
	SPI_WriteData(0x01);
	SPI_WriteComm(0xDE);
	SPI_WriteData(0x01);
	SPI_WriteData(0x51);//58
	SPI_WriteComm(0xE1);
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);
	SPI_WriteComm(0xE6);
	SPI_WriteData(0x55);//58
	SPI_WriteComm(0xF3);
	SPI_WriteData(0x06);
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);
	SPI_WriteData(0x24);
	SPI_WriteData(0x00);
	SPI_WriteComm(0xF8);
	SPI_WriteData(0x00);
	SPI_WriteComm(0xFA);
	SPI_WriteData(0x01);
	SPI_WriteComm(0xFB);
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);
	SPI_WriteComm(0xFC);
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);
	SPI_WriteComm(0xFD);
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);
	SPI_WriteData(0x70);
	SPI_WriteData(0x00);
	SPI_WriteData(0x72);
	SPI_WriteData(0x31);
	SPI_WriteData(0x37);
	SPI_WriteData(0x70);
	SPI_WriteData(0x32);
	SPI_WriteData(0x31);
	SPI_WriteData(0x07);
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);
	SPI_WriteComm(0xFE);
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);
	SPI_WriteData(0x20);
	SPI_WriteComm(0xB0);
	SPI_WriteData(0x04); //04
	LCD_delay(40);
	SPI_WriteComm(0x35);
	SPI_WriteData(0x00);
	SPI_WriteComm(0x44);
	SPI_WriteData(0x00);
	SPI_WriteComm(0x36);
	SPI_WriteData(0x00);
	SPI_WriteComm(0x3A);
	SPI_WriteData(0x77);
	SPI_WriteComm(0x2A);
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);
	SPI_WriteData(0x01);
	SPI_WriteData(0xDF);
	SPI_WriteComm(0x2B);
	SPI_WriteData(0x00);
	SPI_WriteData(0x00);
	SPI_WriteData(0x03);
	SPI_WriteData(0x1F);
	SPI_WriteComm(0x29);
	LCD_delay(10);
	SPI_WriteComm(0x2C);
	LCD_delay(10);

	SPI_WriteComm(0x36);
	SPI_WriteData(0x28);
	SPI_WriteComm(0x2C);
}
 
 
 

static struct of_device_id wokoo_lcdc_of_match[] = {
	{ .compatible = "wokoo-lcdc" },
	{ /* sentinel */ }
};


static void wokoo_lcdc_delay(int time)
{
	volatile unsigned int i;	
	while(time--)
	for(i=500;i>0;i--);
}



static void wokoo_lcdc_reset(struct wokoo_lcdc_info *lcdci)
{
	gpio_set_value(lcdci->reset, 0);   //pull down

	wokoo_lcdc_delay(300);

	gpio_set_value(lcdci->reset, 1);  //pull up

	wokoo_lcdc_delay(300);
}

static void wokoo_lcdc_writedatatospi(struct wokoo_lcdc_info *lcdci,unsigned char byte)
{
    unsigned char n;

    for(n = 0; n < 8; n++)
    {
        if(byte&0x80) 
		{
			gpio_set_value(lcdci->mosi, 1);  //pull up	
		}
        else 
		{
			gpio_set_value(lcdci->mosi, 0);  //pull up	
		}
        byte <<= 1;
		gpio_set_value(lcdci->clk, 0);  //pull up	
		gpio_set_value(lcdci->clk, 1);  //pull up	
    }
}

static void wokoo_lcdc_writeorcom(struct wokoo_lcdc_info *lcdci,unsigned char byte) 
{
	gpio_set_value(lcdci->cs, 0);  //pull up	
	if (TYPE_DATA == lcdci ->type)
	{
		gpio_set_value(lcdci->mosi, 1);  //pull up
	}
	else
	{
		gpio_set_value(lcdci->mosi, 0);  //pull up
	}
	gpio_set_value(lcdci->clk, 0);  //pull up	
	gpio_set_value(lcdci->clk, 1);  //pull up	
	wokoo_lcdc_writedatatospi(lcdci,byte);
	gpio_set_value(lcdci->cs, 1);  //pull up	
}

static void wokoo_lcdc_init(struct wokoo_lcdc_info *lcdci)
{
	int i =0;
    gpio_set_value(lcdci->cs, 1);  //pull up	
    wokoo_lcdc_delay(20);
    gpio_set_value(lcdci->cs, 0);  //pull up	
	wokoo_lcdc_reset(lcdci);

	for (i = 0; i < ASIZE(spi_ctrl); i ++)
	{
		lcdci ->type = spi_ctrl[i].type;
		wokoo_lcdc_writeorcom(lcdci,spi_ctrl[i].data);
		if (spi_ctrl[i].delay > 0)
		{
			wokoo_lcdc_delay(spi_ctrl[i].delay);
		}
	}

	
}

#ifdef CONFIG_PM_SLEEP
static int wokoo_lcdc_suspend(struct device *dev)
{
  struct wokoo_lcdc_info	*lcdci = dev_get_drvdata(dev);
	return 0;
}

static int wokoo_lcdc_resume(struct device *dev)
{
	struct wokoo_lcdc_info  *lcdci = dev_get_drvdata(dev);
	//Lcd_Initialize_l();
	//wokoo_lcdc_init(lcdci);
	return 0;
}
#endif /* CONFIG_PM_SLEEP */

static const struct dev_pm_ops wokoo_lcdc_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(wokoo_lcdc_suspend, wokoo_lcdc_resume)
};
static int wokoo_lcdc_dt(struct device *dev, struct wokoo_lcdc_info *lcdci)
{
	
	struct device_node *np = dev->of_node;
    int ret ;
	enum of_gpio_flags  flags;
	lcdci->reset = of_get_named_gpio_flags(np, "reset-gpios", 0, &flags);
	if (gpio_is_valid(lcdci->reset)) 
	{
		#if 0
		ret = devm_gpio_request(dev, lcdci->reset,"spi_reset");
		if (ret)
		{
			lcdci->reset = -1;
			return -ENODEV;
		}
		#endif
		gpio_direction_output(lcdci->reset, (flags == GPIO_ACTIVE_HIGH) ? 1:0);
	} 
	else
	{
		lcdci->reset  = -1;
		pr_err("gpio_reset request fail\n");
		return -ENODEV;
	}
	
	lcdci->mosi = of_get_named_gpio_flags(np, "sda-gpios", 0, &flags);
	if (gpio_is_valid(lcdci->mosi)) 
	{

		ret = devm_gpio_request(dev, lcdci->mosi,"spi_sda");
		if (ret)
		{
			lcdci->mosi = -1;
			return -ENODEV;
		}
	
		gpio_direction_output(lcdci->mosi, (flags == GPIO_ACTIVE_HIGH) ? 1:0);
	} 
	else
	{
		lcdci->mosi = -1;
		pr_err("gpio_reset request fail\n");
		return -ENODEV;
	}


	lcdci->clk = of_get_named_gpio_flags(np, "clk-gpios", 0, &flags);
	if (gpio_is_valid(lcdci->clk)) 
	{
	
	    ret = devm_gpio_request(dev, lcdci->clk,"spi_clk");
		if (ret)
		{
			lcdci->clk = -1;
			return -ENODEV;
		}
	
		gpio_direction_output(lcdci->clk, (flags == GPIO_ACTIVE_HIGH) ? 1:0);
	}	
	else
	{
		lcdci->clk = -1;
		pr_err("gpio_reset request fail\n");
		return -ENODEV;
	}


    lcdci->cs = of_get_named_gpio_flags(np, "cs-gpios", 0, &flags);
	if (gpio_is_valid(lcdci->cs)) 
	{

		ret = devm_gpio_request(dev, lcdci->cs,"spi_cs");
		if (ret)
		{
			lcdci->cs = -1;
			return -ENODEV;
		}
		
		gpio_direction_output(lcdci->cs, (flags == GPIO_ACTIVE_HIGH) ? 1:0);
	} 	
	else
	{
		 lcdci->cs = -1;
		pr_err("gpio_reset request fail\n");
		return -ENODEV;
	}
   return  0;
}
static int wokoo_lcdc_probe(struct platform_device *pdev)
{
	
	
	 enum of_gpio_flags  flags;
		gpio_reset = of_get_named_gpio_flags(pdev->dev.of_node, "reset-gpios", 0, &flags);
		if (gpio_is_valid(gpio_reset)) {
			gpio_direction_output(gpio_reset, (flags == GPIO_ACTIVE_HIGH) ? 1:0);
//			gpio_set_value(gpio_reset, (flags == GPIO_ACTIVE_HIGH) ? 1:0);
		} else
			pr_err("gpio_reset request fail\n");



	gpio_sda = of_get_named_gpio_flags(pdev->dev.of_node, "sda-gpios", 0, &flags);
		if (gpio_is_valid(gpio_sda)) {
			gpio_direction_output(gpio_sda, (flags == GPIO_ACTIVE_HIGH) ? 1:0);
//			gpio_set_value(gpio_sda, (flags == GPIO_ACTIVE_HIGH) ? 1:0);

		} else
			pr_err("gpio_sda request fail\n");

	gpio_dclk = of_get_named_gpio_flags(pdev->dev.of_node, "clk-gpios", 0, &flags);
		if (gpio_is_valid(gpio_dclk)) {
			
			gpio_direction_output(gpio_dclk, (flags == GPIO_ACTIVE_HIGH) ? 1:0);
//			gpio_set_value(gpio_dclk, (flags == GPIO_ACTIVE_HIGH) ? 1:0);

		} else
			pr_err("gpio_dclk request fail\n");

	  gpio_cs = of_get_named_gpio_flags(pdev->dev.of_node, "cs-gpios", 0, &flags);
		if (gpio_is_valid(gpio_cs)) {
			gpio_direction_output(gpio_cs, (flags == GPIO_ACTIVE_HIGH) ? 1:0);
//			gpio_set_value(gpio_cs, (flags == GPIO_ACTIVE_HIGH) ? 1:0);

		} else
			pr_err("gpio_cs request fail\n");

	//init spi screen  
	
	//Lcd_Initialize_l();
	//msleep(100);
	
	#if 0
	struct wokoo_lcdc_info *lcdci;

	int ret;
	lcdci = devm_kmalloc(&pdev->dev,sizeof(struct wokoo_lcdc_info),GFP_KERNEL);
	if (NULL == lcdci)
	{
		dev_err(&pdev->dev, "lcdc malloc failed\n");
		return -ENOMEM;
	}
		
	
	if (pdev->dev.of_node)
	{
		ret = wokoo_lcdc_dt(&pdev->dev,lcdci);
		if (ret)
		{
			return -ENODEV;
		}
	}
	
	wokoo_lcdc_init(lcdci);
	platform_set_drvdata(pdev, lcdci);
	#endif
	pr_info("[%s]:lcd init success\n",__FUNCTION__);
	
   return 0;
}


MODULE_DEVICE_TABLE(of, wokoo_lcdc_of_match);

static struct platform_driver  wokoo_lcdc_driver = {
	.driver		= {
		.name		= "wokoo-lcdc",
		.of_match_table = wokoo_lcdc_of_match,
		.pm = &wokoo_lcdc_pm_ops,
	},
	.probe		= wokoo_lcdc_probe,
};

module_platform_driver(wokoo_lcdc_driver);

MODULE_AUTHOR("linsh");
MODULE_DESCRIPTION("wokoo-lcdc Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:wokoo-lcdc");