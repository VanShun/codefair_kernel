#include <linux/module.h>

#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/miscdevice.h>
#include <linux/kernel.h>
#include <linux/major.h>
#include <linux/mutex.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/stat.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/tty.h>
#include <linux/kmod.h>
#include <linux/gfp.h>
#include <asm/io.h>
#include <linux/ide.h>
#include <linux/gpio.h>
#include <asm/mach/map.h>
#include <linux/platform_device.h>
#include <linux/of_gpio.h>
#include <linux/of_gpio.h>
#include <linux/of.h>
#include <linux/of_address.h>

#include <linux/input.h>
#include <asm/uaccess.h>
#include <linux/irq.h>
#include <linux/of_irq.h>
#include <linux/timer.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/debugfs.h>
#include <linux/ratelimit.h>
#include "gt9xx.h"


//定义AP3216设备结构体
struct gt9xx_dev {
    struct device_node *nd;         //设备节点
    int irqPin;                     //中断引脚
    int resetPin;                   //复位引脚
    int irqnum;                     //中断号
    void *private_data;             //私有数据
    struct input_dev *input;        //input 结构体
    struct i2c_client *client;      //I2C客户端
    int irq;
};
static struct gt9xx_dev pgt9xx_dev;
//gt9xx读寄存器
static int gt9xx_read_regs(struct gt9xx_dev *dev, uint16_t reg, uint8_t *val, int len)
{
    uint8_t addr[2];
	int ret;
	struct i2c_msg msg[2];
	struct i2c_client *client = (struct i2c_client *)dev->client;

    addr[0] = (reg>>8) & 0xFF;					/* 寄存器高8位地址 */
    addr[1] = reg & 0xFF;             /* 寄存器低8位地址 */
	/* msg[0]为发送要读取的首地址 */
	msg[0].addr = client->addr;			/* ft5x06地址 */
	msg[0].flags = 0;					/* 标记为发送数据 */
    msg[0].buf = addr;

	msg[0].len = 2;						/* reg长度*/
	/* msg[1]读取数据 */
	msg[1].addr = client->addr;			/* ft5x06地址 */
	msg[1].flags = I2C_M_RD;			/* 标记为读取数据*/
	msg[1].buf = val;					/* 读取数据缓冲区 */
	msg[1].len = len;					/* 要读取的数据长度*/
	ret = i2c_transfer(client->adapter, msg, 2);
	if(ret == 2) {
		ret = 0;
	} else {
        printk("b[0] %d b[1]%d\r\n",msg[0].buf[0], msg[0].buf[1]);
        printk("i2c rd failed=%d reg=%06x len=%d\r\n",ret, reg, len);
		ret = -EREMOTEIO;
	}
	return ret;
}
//gt9xx读一个寄存器
static unsigned char gt9xx_read_reg(struct gt9xx_dev *dev, uint16_t reg)
{
    uint8_t data = 0;
    gt9xx_read_regs(dev, reg, &data, 1);
    return data;
}
//gt9xx写寄存器
static s32 gt9xx_write_regs(struct gt9xx_dev *dev, uint16_t reg, uint8_t *buf, uint8_t len)
{
    uint8_t b[256];
	struct i2c_msg msg;
	struct i2c_client *client = (struct i2c_client *)dev->client;
	int ret;

    msg.addr = client->addr;			/* ft5x06地址 */
	msg.flags = 0;					/* 标记为发送数据 */

    b[0] = (reg>>8) & 0xFF;					/* 寄存器高8位地址 */
    b[1] = reg & 0xFF;             /* 寄存器低8位地址 */

    memcpy(&b[2],buf,len);

    msg.buf = b;				/* 要写入的数据缓冲区 */
	msg.len = len + 2;			/* 要写入的数据长度 */
    ret = i2c_transfer(client->adapter, &msg, 1);
	printk("gt9xx_write_regs ret = %d",ret);
	return ret;
}
//gt9xx写单个寄存器
static void gt9xx_write_reg(struct gt9xx_dev *dev, uint16_t reg, uint8_t data)
{
	uint8_t buf = 0;
	buf = data;
	gt9xx_write_regs(dev, reg, &buf, 1);
}
//gt9xx中断
static irqreturn_t gt9xx_handler(int irq, void *dev_id)
{
    struct gt9xx_dev *multidata = (struct gt9xx_dev *)dev_id;
    uint8_t rdbuf[40] = {0};
    int i, x, y, id;
    //printk("cd %s\r\n",__FUNCTION__);
    //清除
    memset(rdbuf, 0, sizeof(rdbuf));
    //读取坐标点寄存器
    gt9xx_read_regs(multidata,GTP_COOR_REG,rdbuf,40);
    // * bit7  Buffer status，1 表示坐标（或按键）已经准备好，主控可以读取；0 表示未就绪，数据无效。
    // Bit4: HaveKey, 1 表示有按键，0 表示无按键（已经松键）。
    // Bit3~0: Number of touch points, 屏上的坐标点个数
    //上报触摸点坐标
    uint8_t status = rdbuf[0] >> 7;
    uint8_t HaveKey = (rdbuf[0] >> 4)& 0x01;
    uint8_t keynum = rdbuf[0] & 0x0F;
    printk("status:%d  HaveKey:%d keynum:%d\r\n",status,HaveKey,keynum);
    if( status) {
        for (i = 0; i < MAX_SUPPORT_POINTS; i++) {
            uint8_t *buf = &rdbuf[i * 8];
            id = buf[1];
            x = ((buf[3] << 8) | buf[2]) & 0x0fff;
            y = ((buf[5] << 8) | buf[4]) & 0x0fff;
            // x= x*5/4;
            // y= y*5/4;
            input_mt_slot(multidata->input, i);
            if (i < keynum) {
                input_mt_report_slot_state(multidata->input, MT_TOOL_FINGER, true);
            } else {
                input_mt_report_slot_state(multidata->input, MT_TOOL_FINGER, false);
            }

            input_report_abs(multidata->input, ABS_MT_POSITION_X, x);
            input_report_abs(multidata->input, ABS_MT_POSITION_Y, y);
            printk("id:%d  X:%d Y:%d\r\n",id,x,y);
        }
        input_mt_report_pointer_emulation(multidata->input, true);
        input_sync(multidata->input);
        //状态寄存器清零
        gt9xx_write_reg(multidata,GTP_COOR_REG,0);
    }
    return IRQ_HANDLED;
}
//复位gt9xx
static int gtp_setaddr(struct i2c_client *client,struct gt9xx_dev *dev,uint8_t addr)
{
    int ret = 0;
    printk("cd %s\r\n",__FUNCTION__);
    if (gpio_is_valid(dev->irqPin)) { // 检查 IO 是否有效
        ret = devm_gpio_request_one(&client->dev,dev->irqPin,
                                    GPIOF_OUT_INIT_LOW,
                                    "gt9xx irq init");
        if (ret) {
            dev_err(&client->dev,"Failed to request irq GPIO %d, error %d\n",dev->irqPin, ret);
            return ret;
        }
        msleep(5);
        gpio_set_value(dev->irqPin, 0); //输出低电平
        msleep(100);
    } else {
        printk("irqPin is valid!\r\n");
    }
	/*
    if (gpio_is_valid(dev->resetPin)) { // 检查 IO 是否有效
        ret = devm_gpio_request_one(&client->dev,dev->resetPin,
                                    GPIOF_OUT_INIT_LOW,
                                    "gt9xx reset init");
        if (ret) {
            return ret;
        }
        msleep(5);
        gpio_set_value(dev->resetPin, 0); //输出低电平
        msleep(100);
    } else {
        printk("resetPin is valid!\r\n");
    }

    */
    if (addr == 0X5D) {
        printk("GT9XX ADDR :0X5D\r\n");
        msleep(10);
        gpio_set_value(dev->irqPin, 1); //输出高电平
        msleep(10);
        //gpio_set_value(dev->resetPin, 1); //输出高电平
        //msleep(10);
    } else if (addr == 0XBA) {
        printk("GT9XX ADDR :0XBA\r\n");
        msleep(10);
        //gpio_set_value(dev->resetPin, 1); //输出高电平
        msleep(300);
        //gpio_set_value(dev->irqPin, 1);
    } else {
        printk("GT9XX ADDR SET ERROR !!! MUST 0X5D OR 0XBA\r\n");
        ret = -1;
        return ret;
    }
    return 0;
}

//初始化中断
static int gtp_irq(struct i2c_client *client,struct gt9xx_dev *dev)
{
    int ret = 0;
    printk("cd %s\r\n",__FUNCTION__);
    gpio_free(dev->irqPin);
    //msleep(10);
    
    ret = devm_gpio_request_one(&client->dev,dev->irqPin,
                                GPIOF_IN,"gt9xx irq");


    if (ret) {
        dev_err(&client->dev,"Failed to request irq GPIO %d, error %d\n",dev->irqPin, ret);
        return ret;
    }
	dev->irq = gpio_to_irq(dev->irqPin);   
	//devm_gpiod_get_optional()
	//gpiod_to_irq();
	
	if (dev->irq) {
        client->irq = dev->irq;
        ret = devm_request_threaded_irq(&client->dev,
                                        client->irq,
                                        NULL,
                                        gt9xx_handler,
                                        IRQF_TRIGGER_RISING | IRQF_ONESHOT,
                                        client->name,
                                        &pgt9xx_dev);
        if (ret) {
            dev_err(&client->dev,"Unable to request touchscreen IRQ.\n");
            return ret;
        }
        //msleep(100);
        printk("gt9xx_handler irq number:%d\r\n",client->irq);
	}
    return 0;
}


static void gt9xx_init(void)
{
    uint8_t regdata[186] = {0};
    int ret = 0;
    uint8_t i= 0;
    uint8_t sofeVersion = 0;
    uint8_t gt9xx_id[6] = {0};
    uint8_t iqrmode = 0;
    uint8_t crc = 0;
    printk("cd %s\r\n",__FUNCTION__);
    //读软件版本号
    gt9xx_read_regs(&pgt9xx_dev, GT_CFGS_REG ,&sofeVersion,1);
    printk("Soft Version :%d\r\n",sofeVersion);
    //1读取产品IDs
    gt9xx_read_regs(&pgt9xx_dev, GT_PID_REG ,gt9xx_id,6);
    printk("CTP ID :");
    for (i = 0; i < 6; i++) {
        printk("%d ",gt9xx_id[i]);
    }
    printk("\r\n");
    //读取中断触发方式
    gt9xx_read_regs(&pgt9xx_dev, 0x804D ,&iqrmode,1);
    printk("IRQMODE :%d\r\n",iqrmode);
    //读取184个寄存器
    gt9xx_read_regs(&pgt9xx_dev, GT_CFGS_REG ,regdata,186);
    for(i = 0; i < 186; i++){
        printk("%#X ", regdata[i]);
        if( i < 184){
            crc += regdata[i];
        }
    }
    printk("\r\n");
    crc = (~crc) + 1;
    printk("crc:%d\r\n",crc);
    //2.软件复位
    gt9xx_write_reg(&pgt9xx_dev,COMMAND_REG,2);
    //3.配置186个寄存器
    ret = of_property_read_u8_array(pgt9xx_dev.client->dev.of_node, "goodix,cfg-group0", regdata, 186);
    if (ret < 0) {
        printk("goodix,cfg-group0 property read failed\r\n");
    } else {
        printk("reg data:\r\n");
        for(i = 0; i < 186; i++){
            printk("%X ", regdata[i]);
        }
        printk("\r\n");
    }
	printk("sizeof(regdata) = %d",sizeof(regdata));
    gt9xx_write_regs(&pgt9xx_dev,GT_CFGS_REG,regdata,sizeof(regdata));
     msleep(100);
	 //enter red coordinate state
    gt9xx_write_reg(&pgt9xx_dev,COMMAND_REG,0);
    msleep(100);
}

static int gtp_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int ret = 0;
    printk("cd %s\r\n",__FUNCTION__);

    pgt9xx_dev.client = client;

    //获取中断引脚和复位引脚
    pgt9xx_dev.irqPin = of_get_named_gpio(client->dev.of_node,"irq-gpios",0);
    pgt9xx_dev.resetPin = of_get_named_gpio(client->dev.of_node,"reset-gpios",0);
    //设置gt9xx地址
    ret = gtp_setaddr(client, &pgt9xx_dev,GT9XXADDRESS);
        if(ret < 0) {
        goto fail;
    }
    //初始化gt9xx
    gt9xx_init();
    //初始化中断
    ret = gtp_irq(pgt9xx_dev.client, &pgt9xx_dev);
        if(ret < 0) {
        goto fail;
    }

    //申请input设备
    pgt9xx_dev.input = devm_input_allocate_device(&client->dev);
    if (!pgt9xx_dev.input) {
        ret = -ENOMEM;
        goto fail;
    }
    //初始化input
    pgt9xx_dev.input->name = client->name;
    pgt9xx_dev.input->id.bustype = BUS_I2C;
    pgt9xx_dev.input->dev.parent = &client->dev;
    //设置input设备需要上报事件类型和按键值
    __set_bit(EV_KEY, pgt9xx_dev.input->evbit);
    __set_bit(EV_ABS, pgt9xx_dev.input->evbit);
    __set_bit(BTN_TOUCH, pgt9xx_dev.input->keybit);
    //设置input设备 需要上报的绝对坐标
    input_set_abs_params(pgt9xx_dev.input, ABS_X, 0, 800, 0, 0);
    input_set_abs_params(pgt9xx_dev.input, ABS_Y, 0, 480, 0, 0);
    input_set_abs_params(pgt9xx_dev.input, ABS_MT_POSITION_X,0, 800, 0, 0);
    input_set_abs_params(pgt9xx_dev.input, ABS_MT_POSITION_Y,0, 480, 0, 0);
    //初始化多点电容触摸的 slots
    ret = input_mt_init_slots(pgt9xx_dev.input, MAX_SUPPORT_POINTS, 0);
    if (ret) {
        goto fail;
    }
    //注册input
    ret = input_register_device(pgt9xx_dev.input);
    if (ret)
        goto fail;

    return 0;
fail:
    return ret;
}

static int gtp_drv_remove(struct i2c_client *client)
{
    //释放输入设备
    input_unregister_device(pgt9xx_dev.input);
    return 0;
}
static const struct i2c_device_id gtp_device_id[] = {
    {"goodix-ts", 0,},
    {}
};

static const struct of_device_id gtp_match_table[] = {
    { .compatible = "goodix,gt9xx", },
    { /* Sentinel */ }
};

//IIC驱动结构体
static struct i2c_driver goodix_ts_driver = {
    .driver = {
        .owner = THIS_MODULE,
        .name = "goodix-ts",
        .of_match_table = of_match_ptr(gtp_match_table),
    },
    .id_table = gtp_device_id,
    .probe = gtp_probe,
    .remove = gtp_drv_remove,
};



static int __init gtp_init(void)
{
    int ret = 0;
    printk("cd %s\r\n",__FUNCTION__);
    //注册i2c_driver结构体
    ret = i2c_add_driver(&goodix_ts_driver);
    return ret;
}

static void __exit gtp_exit(void)
{
    printk("cd %s\r\n",__FUNCTION__);
    //注销i2c_driver结构体
    i2c_del_driver(&goodix_ts_driver);
}




module_init(gtp_init);
module_exit(gtp_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("liuxiang");