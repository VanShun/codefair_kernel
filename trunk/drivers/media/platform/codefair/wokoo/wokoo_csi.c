/* SPDX-License-Identifier: GPL-2.0
 *
 * Copyright (c) 2020 Codefair Semiconductor Technology Co., Ltd.
 *		http://www.codefairsemi.com 
 *
 */

#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>
#include <linux/videodev2.h>

#include <media/v4l2-dev.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mediabus.h>

#include <media/videobuf2-core.h>
#include <media/videobuf2-dma-contig.h>

#include <asm/page.h>
#include <linux/mm.h>

#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/omap-dma.h>



#include "wokoo_csi.h"
//#include "isp_api.h"
//#include "isp_example.h"
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/init.h>
#include <linux/serio.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/miscdevice.h>
#include <linux/gpio.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <linux/uaccess.h>
//#include <mach/regs-clock.h>
//#include <plat/regs-timer.h>   
//#include <mach/regs-gpio.h>
#include <linux/cdev.h>
#include <linux/device.h>

// extern struct fasync_struct *async;
// #define BITS_PER_BYTE 8
// #define BITS_PER_LONG 32

// #define BIT(nr)		(1UL << (nr))
// #define BIT_MASK(nr)	(1UL << ((nr) & (BITS_PER_LONG - 1)))
// #define BIT_WORD(nr)	((nr) / BITS_PER_LONG)
#define SZ_3M               0x00300000
#define BASE_ADDR_ISP_REG	    (0x0UL)	

#define REG_ISP_CTRL0	(BASE_ADDR_ISP_REG + 0)
#define REG_ISP_CTRL1	(BASE_ADDR_ISP_REG + 0x04)
#define REG_ISP_CTRL2	(BASE_ADDR_ISP_REG + 0x08)
#define REG_SECTION1_DP_CTRL0	(BASE_ADDR_ISP_REG + 0x10)

#define REG_SECTION1_CTRL	(BASE_ADDR_ISP_REG + 0x14)
// #define REG_S1R0_CMD_REG0	(BASE_ADDR_ISP_REG + 0x100)
// #define REG_S1R0_CMD_REG1	(BASE_ADDR_ISP_REG + 0x104)
// #define REG_S1R0_CMD_REG2	(BASE_ADDR_ISP_REG + 0x108)
// #define REG_S1R0_CMD_REG3	(BASE_ADDR_ISP_REG + 0x10C)

// #define REG_S1W0_CMD_REG0	(BASE_ADDR_ISP_REG + 0x110)
// #define REG_S1W0_CMD_REG1	(BASE_ADDR_ISP_REG + 0x114)
// #define REG_S1W0_CMD_REG2	(BASE_ADDR_ISP_REG + 0x118)
// #define REG_S1W0_CMD_REG3	(BASE_ADDR_ISP_REG + 0x11C)

#define REG_SECTION2_DP_CTRL0	(BASE_ADDR_ISP_REG + 0x20)
#define REG_SECTION2_DP_CTRL1	(BASE_ADDR_ISP_REG + 0x24)

/*REG_SECTION2_DP_CTRL2*/
#define REG_SECTION2_DP_CTRL2	(BASE_ADDR_ISP_REG + 0x28)


/*REG_SECTION2_CTRL*/
#define REG_SECTION2_CTRL	(BASE_ADDR_ISP_REG + 0x2C)


// #define REG_S2R0_CMD_REG0	(BASE_ADDR_ISP_REG + 0x120)
// #define REG_S2R0_CMD_REG1	(BASE_ADDR_ISP_REG + 0x124)
// #define REG_S2R0_CMD_REG2	(BASE_ADDR_ISP_REG + 0x128)
// #define REG_S2R0_CMD_REG3	(BASE_ADDR_ISP_REG + 0x12C)

// #define REG_S2R1_CMD_REG0	(BASE_ADDR_ISP_REG + 0x130)
// #define REG_S2R1_CMD_REG1	(BASE_ADDR_ISP_REG + 0x134)
// #define REG_S2R1_CMD_REG2	(BASE_ADDR_ISP_REG + 0x138)
// #define REG_S2R1_CMD_REG3	(BASE_ADDR_ISP_REG + 0x13C)


// #define REG_S2W1_CMD_REG0	(BASE_ADDR_ISP_REG + 0x190)
// #define REG_S2W1_CMD_REG1	(BASE_ADDR_ISP_REG + 0x194)
// #define REG_S2W1_CMD_REG2	(BASE_ADDR_ISP_REG + 0x198)
// #define REG_S2W1_CMD_REG3	(BASE_ADDR_ISP_REG + 0x19C)



// #define REG_S2W5_CMD_REG0	(BASE_ADDR_ISP_REG + 0x1D0)
// #define REG_S2W5_CMD_REG1	(BASE_ADDR_ISP_REG + 0x1D4)
// #define REG_S2W5_CMD_REG2	(BASE_ADDR_ISP_REG + 0x1D8)
// #define REG_S2W5_CMD_REG3	(BASE_ADDR_ISP_REG + 0x1DC)

#define REG_ISP_INTR_MASK	(BASE_ADDR_ISP_REG + 0x800)
#define REG_ISP_INTR_CLEAR	(BASE_ADDR_ISP_REG + 0x804)
#define REG_ISP_INTR_STATUS	(BASE_ADDR_ISP_REG + 0x808)


// #define MIPI_FLAG
// #define debug_isp
// struct fasync_struct *async;

unsigned int phy_address_img;
unsigned int phy_address_regs;

struct ioctl_data {		
	unsigned int reg;
	unsigned int value;
	unsigned char bit;

};



struct wokoo_csi *global_csi;
char *img_buff;
char *reg_buff;

unsigned int reg_count=0;


#define ISP_MAGIC 'k'
#define IOCTL_ISP_REGW _IOW (ISP_MAGIC, 1, int)
#define IOCTL_ISP_REGWB _IOW (ISP_MAGIC, 2, int)
#define IOCTL_ISP_REGR _IOW (ISP_MAGIC, 3, int)
#define IOCTL_ISP_SETBIT _IOW (ISP_MAGIC, 4, int)
#define IOCTL_ISP_CLRBIT _IOW (ISP_MAGIC, 5, int)
#define IOCTL_ISP_INIT _IOW (ISP_MAGIC, 6, int)
#define IOCTL_ISP_GETIMGADD _IOW (ISP_MAGIC, 7, int)
#define IOCTL_ISP_GETREGS _IOW (ISP_MAGIC, 8, int)
#define IOCTL_ISP_INTFLAG _IOW (ISP_MAGIC, 9, int)
#define IOCTL_ISP_LCDFLAG _IOW (ISP_MAGIC, 10, int)
#define IOCTL_ISP_FLAGCLRLCD _IOW (ISP_MAGIC, 11, int)
#define IOCTL_ISP_FLAGCLRISP _IOW (ISP_MAGIC, 12, int)


#define CTL_CPHY0_RSTZ	0x10
#define CTL_CPHY0_CTRL0	0x18
#define CTL_CPHY0_CTRL1	0x1C
#define CTL_CPHY0_MODE	0x28


#define BITS_PER_BYTE 8
#define BITS_PER_LONG 32


static int isp_fasync(int fd, struct file *filp, int on)  
{  
    int ret=0;
    struct wokoo_csi *dev = filp->private_data;
	// struct wokoo_csi *dev = (struct wokoo_csi *)filp->private_data;
	// struct wokoo_csi *dev = filp->private_data;
	
	ret=fasync_helper(fd, filp, on, &dev->async_isp);
	// ret=fasync_helper(fd, filp, on, &global_csi->async_isp);
    printk("ISP app fasync!\n"); 
    if(ret>0){
    return 0;	
    }
    else{
    	return -1;
    }
    return 0;
}  
  
static int isp_release(struct inode *inode, struct file *filp)  
{   
	// struct wokoo_csi *dev = (struct wokoo_csi *)filp->private_data;
	// struct wokoo_csi *dev = filp->private_data;
    printk("isp app close!\n");  
    // fasync_helper(-1, filp, 0,&dev->async_isp); 
    isp_fasync(-1, filp, 0);
    return 0; 
} 
#if 0 
static inline void set_bit(int nr, volatile unsigned long *addr)
{
	unsigned long mask = BIT_MASK(nr);
	volatile unsigned long *p = ((unsigned long *)addr) + BIT_WORD(nr);

	*p |= mask;
}

static inline void clear_bit(int nr, volatile unsigned long *addr)
{
	unsigned long mask = BIT_MASK(nr);
	volatile unsigned long *p = ((unsigned long *)addr) + BIT_WORD(nr);

	*p &= ~mask;
}
#endif
#if 0 
static inline void set_bit_w(int nr, volatile unsigned long *addr)
{
	unsigned long mask = BIT_MASK(nr);
	unsigned long value;
	//volatile unsigned long *p = ((unsigned long *)addr) + BIT_WORD(nr);
	value = readl((unsigned long *)addr) + BIT_WORD(nr);

	value |= mask;

	writel(value,(unsigned long *)addr);
}

static inline void clear_bit_w(int nr, volatile unsigned long *addr)
{
	unsigned long mask = BIT_MASK(nr);
	unsigned long value;
	//volatile unsigned long *p = ((unsigned long *)addr) + BIT_WORD(nr);

	//*p &= ~mask;
	value = readl((unsigned long *)addr) + BIT_WORD(nr);

	value &= ~mask;

	writel(value,(unsigned long *)addr);
}
#endif
#if 0 
static inline void set_bit_w(int nr, volatile unsigned long *addr)
{
	unsigned long mask = BIT_MASK(nr);
	unsigned long value;
	//volatile unsigned long *p = ((unsigned long *)addr) + BIT_WORD(nr);
	value = readl((unsigned long *)addr) + BIT_WORD(nr);

	value |= mask;

	writel(value,(unsigned long *)addr);
}

static inline void clear_bit_w(int nr, volatile unsigned long *addr)
{
	unsigned long mask = BIT_MASK(nr);
	unsigned long value;
	//volatile unsigned long *p = ((unsigned long *)addr) + BIT_WORD(nr);

	//*p &= ~mask;
	value = readl((unsigned long *)addr) + BIT_WORD(nr);

	value &= ~mask;

	writel(value,(unsigned long *)addr);
}
#endif
#if 1 
void delay_me(volatile int delay)
{
	volatile int i=0;
	for(i=0; i<delay; i++);
}
static inline void set_bit_w(int nr, volatile unsigned long *addr)
{
	unsigned long mask = BIT_MASK(nr);
	volatile unsigned long *p = ((unsigned long *)addr) + BIT_WORD(nr);

	*p |= mask;
}

static inline void clear_bit_w(int nr, volatile unsigned long *addr)
{
	unsigned long mask = BIT_MASK(nr);
	volatile unsigned long *p = ((unsigned long *)addr) + BIT_WORD(nr);
	*p &= ~mask;
}
#endif

void init_phy(struct wokoo_csi *csi) {

	printk("init_phy=============\n");
	writel(0x01f,csi->ctl_regs +CTL_CPHY0_RSTZ);
	writel(0x01c,csi->ctl_regs +CTL_CPHY0_RSTZ);
	udelay(5);
	writel(0x01d,csi->ctl_regs +CTL_CPHY0_RSTZ);
	delay_me(3);
	writel(0x01f,csi->ctl_regs +CTL_CPHY0_RSTZ);
	printk("CTL_CPHY0_RSTZ  = %p\n",csi->ctl_regs +CTL_CPHY0_RSTZ);
	delay_me(3);
	writel(0X02,csi->ctl_regs +CTL_CPHY0_MODE);
  
}

int mipi_ctrl_init(struct wokoo_csi *csi, int num)
{
	//unsigned int data;

	printk("mipi_ctrl_init=============\n");
	delay_me(5);
	writel(0x00000000,csi->ctl_regs +CTL_CPHY0_CTRL1 );
	delay_me(5);
	delay_me(5);
	writel(0x00000001,csi->ctl_regs +CTL_CPHY0_CTRL0);
	delay_me(5);
	delay_me(5);
	delay_me(5);
	writel(0xffffffff,csi->regs +0x20040);
	delay_me(5);
	delay_me(5);
	writel(0xffffffff,csi->regs +0x20044);
	delay_me(5);
	delay_me(5);
	writel(0xffffffff, csi->regs +0x20008);
	delay_me(5);
	writel(0x00000000,csi->ctl_regs +CTL_CPHY0_CTRL0);
	delay_me(5);
	delay_me(5);
	writel(0x00000000, csi->ctl_regs +CTL_CPHY0_CTRL0);
	delay_me(5);
	delay_me(5);
	writel(0x00010034,csi->ctl_regs +CTL_CPHY0_CTRL1);
	delay_me(5);
	delay_me(5);
	writel(0x00000002,csi->ctl_regs +CTL_CPHY0_CTRL0);
	delay_me(5);
	delay_me(5);
	writel(0x00000000,csi->ctl_regs +CTL_CPHY0_CTRL0);
	delay_me(5);
	delay_me(5);
	writel(0x00000014,csi->ctl_regs +CTL_CPHY0_CTRL1);
	delay_me(5);
	delay_me(5);
	writel(0x00000002,csi->ctl_regs +CTL_CPHY0_CTRL0);
	delay_me(5);
	delay_me(5);
	writel(0x00000000,csi->ctl_regs +CTL_CPHY0_CTRL0);
	delay_me(5);
	delay_me(5);
	writel(0x00010044,csi->ctl_regs +CTL_CPHY0_CTRL1);
	delay_me(5);
	delay_me(5);
	writel(0x00000002,csi->ctl_regs +CTL_CPHY0_CTRL0);
	delay_me(5);
	delay_me(5);
	writel(0x00000000,csi->ctl_regs +CTL_CPHY0_CTRL0);
	delay_me(5);
	delay_me(5);
	if(num==2)
		writel(0x0000000c,csi->ctl_regs +CTL_CPHY0_CTRL1);   //modify by liuyz 2lane
	else
		writel(0x00000014,csi->ctl_regs +CTL_CPHY0_CTRL1);   //modify by liuyz 1lane

	delay_me(5);
	delay_me(5);
	writel(0x00000002,csi->ctl_regs +CTL_CPHY0_CTRL0);
	delay_me(5);
	delay_me(5);
	writel(0x00000000,csi->ctl_regs +CTL_CPHY0_CTRL0);
	delay_me(50);	
	writel(num-1,csi->regs +0x20004);   //REG_W(0x20004,0x00000001);

	printk("mipi_ctrl_init end=======\n");
	return 0;
}

void isp_module_init(struct wokoo_csi *csi)
{
// #ifdef MIPI_FLAG

 if (!strcmp(csi->type, "mipi")){
 	init_phy(csi);
	mipi_ctrl_init(csi,1); 
	printk("mipi init ok\n");   	
 }
  if (!strcmp(csi->type, "mipi2")){
 	init_phy(csi);
	mipi_ctrl_init(csi,2);  	
 }
 if (!strcmp(csi->type, "dvp")){
   printk("dvp init ok\n"); 
 return;
 }
return;
}



static int isp_open(struct inode *inode, struct file *filp)
{  
	// struct wokoo_csi *dev = (struct wokoo_csi *)filp->private_data// 
	filp->private_data = global_csi;
     // memset(img_buff, 0, 1024*1024);
	// delay_me(10);
    // isp_module_init(global_csi);
    return 0;
}



static int isp_map(struct file *filp, struct vm_area_struct *vma)
{    
    unsigned long page;
    // unsigned char i;
    unsigned long start = (unsigned long)vma->vm_start;
    //unsigned long end =  (unsigned long)vma->vm_end;
    unsigned long size = (unsigned long)(vma->vm_end - vma->vm_start);
    // filp->private_data = &global_csi;
 	//printk("kernel enter my map \n");
 	//page = virt_to_phys(buffer);    
    page = phy_address_img;    
  
    if(remap_pfn_range(vma,start,page>>PAGE_SHIFT,size,PAGE_SHARED))
    {    
		printk("wokoo isp map error\n");
		return -1;

	}

	printk("wokoo isp map success\n");
    return 0;

}


static ssize_t isp_write(struct file *filp, const char __user *user_buf, size_t count, loff_t *ppos)
{  
	// struct wokoo_csi *dev = (struct wokoo_csi *)filp->private_data;
	// filp->private_data = &global_csi;
	// struct wokoo_csi *dev = (struct wokoo_csi *)filp->private_data;
    memset(img_buff+0x100000, 0, 1024*1024);
    if (copy_from_user(img_buff+0x100000, user_buf, count))
    {
    	printk("kernel copy error\n");
        return -EFAULT;
    }
	printk("kernel copy end\n");
	return 0;
}

static long isp_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret;
	struct ioctl_data ispcmd_data;

	if(copy_from_user(&ispcmd_data, (struct ioctl_data *)arg, sizeof(struct ioctl_data))){
		ret = -1;
		printk("isp arg is error!\n");
	}


    switch(cmd) {
		 case IOCTL_ISP_INIT:
			isp_module_init(global_csi);
           break;
			
        case IOCTL_ISP_REGW:
           	writel( ispcmd_data.value, global_csi->regs + ispcmd_data.reg);
           	#ifdef debug_isp
		    printk("REGW AFTER %x=%x\n", 0xa0180000+ispcmd_data.reg,readl(global_csi->regs + ispcmd_data.reg));
		    #endif
			break;

        case IOCTL_ISP_REGWB:
			if(ispcmd_data.value == 1)
			{
				set_bit_w(ispcmd_data.bit,global_csi->regs + ispcmd_data.reg);
				#ifdef debug_isp
				printk("REGWB  %xBIT%x=1\n", ispcmd_data.reg,ispcmd_data.bit);
				#endif
			}
			else
			{
				clear_bit_w(ispcmd_data.bit,global_csi->regs + ispcmd_data.reg);
				#ifdef debug_isp
				printk("REGWB  %xBIT%x=0\n", ispcmd_data.reg,ispcmd_data.bit);
				#endif
			}
           	break;
            
        case IOCTL_ISP_REGR:
            ispcmd_data.value =0;			 
			ispcmd_data.value = readl(global_csi->regs + ispcmd_data.reg);
			if(copy_to_user( (struct ioctl_data *)arg, &ispcmd_data, sizeof(struct ioctl_data))){
				ret = -1;
			 	printk("ISP REGR arg is error!\n");
			}	
			break;


		case IOCTL_ISP_GETIMGADD:
			ispcmd_data.value = phy_address_img;
			#ifdef debug_isp
			printk(" ISP GETIMGADD =%x \n", ispcmd_data.value);
			#endif
			if(copy_to_user( (struct ioctl_data *)arg, &ispcmd_data, sizeof(struct ioctl_data))){
				ret = -1;
			 	printk("ISP IOCTL_ISP_GETIMGADD arg is error!\n");
			}	
            break;

		case IOCTL_ISP_GETREGS:
			for(reg_count=0; reg_count<ispcmd_data.bit*4; reg_count=reg_count+4 )
			{
			#ifdef debug_isp
		    printk("%x=%x\n", global_csi->regs + ispcmd_data.reg +reg_count,readl(global_csi->regs + ispcmd_data.reg +reg_count));
			printk("IOCTL_ISP_GETREGS cpy end\n");
			#endif
		    }
			break;
      	case IOCTL_ISP_INTFLAG:
			ispcmd_data.value =global_csi->ispint;
			printk("ctlval=%x \n", ispcmd_data.value);
			if(copy_to_user( (struct ioctl_data *)arg, &ispcmd_data, sizeof(struct ioctl_data))){
				ret = -1;
			 	printk("ISP IOCTL_ISP_GETIMGADD arg is error!\n");
			}
			// global_csi->ispint=0xf;	
			break;
		case IOCTL_ISP_LCDFLAG:
		    ispcmd_data.value =global_csi->ispdataflag;
           	#ifdef debug_isp
		    printk("ctlval=%x \n", ispcmd_data.value);
		    #endif
			if(copy_to_user( (struct ioctl_data *)arg, &ispcmd_data, sizeof(struct ioctl_data))){
				ret = -1;
			 	printk("ISP IOCTL_ISP_GETIMGADD arg is error!\n");
			}      
		    break;
        default:
            printk("ISP in the default!!\n");
            // return -EINVAL;
            break;
    }
    return 0;
}


static struct file_operations isp_fops = {
    .owner  =   THIS_MODULE,
    .open   =   isp_open,    
    .release = isp_release, 
    .unlocked_ioctl  =  isp_ioctl,
    .mmap   = isp_map,
    .write  = isp_write,
    .fasync = isp_fasync, 
};
#if 1
volatile u32 k=0;
static irqreturn_t wokoo_isp_irq(int irq, void *devid)
{
	u32 sta=0;
	// u32 i=0;
	// struct wokoo_csi *isp = devid;
	struct wokoo_csi *isp = global_csi;
    sta = readl(isp->regs +REG_ISP_INTR_STATUS);
    // kill_fasync(&isp->async_isp, SIGIO, POLL_IN);
    writel(0x3030363,isp->regs +REG_ISP_INTR_MASK);
    writel(sta,isp->regs +REG_ISP_INTR_CLEAR);
    if(global_csi->async_isp){
    kill_fasync(&global_csi->async_isp, SIGIO, POLL_IN);	
    }
    // printk("isp sta: %x\n", sta);
 return IRQ_HANDLED;
}
#endif

#if 1
static int isp_con_init(struct wokoo_csi *csi)
{

	// struct wokoo_csi *csi;
	char dev_name[]="isp";
	int ISP_CNT=1;
	if (csi->major) {
		csi->devid = MKDEV(csi->major, 0);
		register_chrdev_region(csi->devid, ISP_CNT, dev_name);
	} else {
		alloc_chrdev_region(&csi->devid, 0, ISP_CNT, dev_name);
		csi->major = MAJOR(csi->devid);
	}

	/* cdev */
	cdev_init(&csi->IspDevs, &isp_fops);
	cdev_add(&csi->IspDevs, csi->devid, ISP_CNT);

	/* class */
	csi->class = class_create(THIS_MODULE, dev_name);
	if (IS_ERR(csi->class)) {
		return PTR_ERR(csi->class);
	}

	/* device */
	csi->dev = device_create(csi->class, NULL, csi->devid, NULL, dev_name);
	if (IS_ERR(csi->dev)) {
		return PTR_ERR(csi->dev);
	}
    // global_csi->private_data = csi; 
	// csi->private_data = client;
	// ov5648_detect(&ov5648dev);
	printk("Isp device installed, with major %d\n", csi->major);
    printk("The device name is: %s\n", dev_name);	
	return 0;
}

static void  isp_exit(struct wokoo_csi *csi)
{
  	cdev_del(&csi->IspDevs);
	unregister_chrdev_region(csi->devid, 1);

	/* device class */
	device_destroy(csi->class, csi->devid);
	class_destroy(csi->class);
    printk("Isp device uninstalled\n");
    return ;
}
#endif
static int wokoo_csi_get_ofdata(struct wokoo_csi *csi)
{
	uint32_t model=0;
	struct device *dev = csi->dev;
	struct device_node *node = dev->of_node;
	int ret;

	ret = of_property_read_u32(node, "senser-name", &model);
	if (ret) {
		dev_warn(dev, "No csi  selected, using 5648\n");
		// csi->type = 5648;
	}
	printk("senser name=%d\n",model);
    csi->stype=model;
	return 0;
}
static int wokoo_csi_get_ofdata_str_type(struct wokoo_csi *csi)
{
	// const char *type=csi->nd->name;
	 const char *type;
	// const char *name;
	struct device *dev = csi->dev;
	struct device_node *node = dev->of_node;
	// struct device_node *node = csi->nd;
	int ret;

	ret = of_property_read_string(node, "senser-type", &type);
	if (ret) {
		dev_warn(dev, "No csi  selected, using 5648\n");
		// csi->type = 5648;
	}
    csi->type=type;
    ret = of_property_read_string(node, "memsize", &type);
    if (ret) {
    dev_warn(dev, " no memsize\n");
        // csi->type = 5648;
    } 
    if (!strcmp(type, "1M")) {
     printk("1M\n");
     }
     else if (!strcmp(type, "2M")) {
     printk("2M\n");   
     }
     else if (!strcmp(type, "3M")) {
     printk("3M\n"); 
     }
     else{
        return -EINVAL;
     }
     csi->memsize=type;  
	return 0;
}
#if 0
static void isp_setup_cdev(struct cdev *dev, int minor,
        struct file_operations *fops)
{
    int err, devno = MKDEV(isp_major, minor);
    
    cdev_init(dev, fops);
    dev->owner = THIS_MODULE;
    dev->ops = fops;
    err = cdev_add (dev, devno, 1);
    /* Fail gracefully if need be */
    if (err)
        printk (KERN_NOTICE "Error %d adding Isp%d", err, minor);
}
static int isp_con_init(void)

{
    int result;
    dev_t dev = MKDEV(isp_major, 0);
    char dev_name[]="isp"; 

    /* Figure out our device number. */
    if (isp_major)
        result = register_chrdev_region(dev, 1, dev_name);
    else {
        result = alloc_chrdev_region(&dev, 0, 1, dev_name);
        isp_major = MAJOR(dev);
    }
    if (result < 0) {
        printk(KERN_WARNING "isp: unable to get major %d\n", isp_major);
        return result;
    }
    if (isp_major == 0)
        isp_major = result;

    /* Now set up cdev. */
    isp_setup_cdev(&IspDevs, 0, &isp_fops);


    isp_class = class_create(THIS_MODULE, "isp_class");/* /sys/class/~*/
    device_create(isp_class, NULL, IspDevs.dev,  dev_name,  dev_name);/* /dev/$DEVICE_NAME*///?
    
    printk("Isp device installed, with major %d\n", isp_major);
    printk("The device name is: %s\n", dev_name);
    return 0;

}

/*
 * ”rmmod isp”
 */
static void __exit isp_exit(void)
{
  
    cdev_del(&IspDevs);
    unregister_chrdev_region(MKDEV(isp_major, 0), 1);
    printk("Isp device uninstalled\n");
}
#endif
static int wokoo_csi_probe(struct platform_device *pdev)
{
	struct wokoo_csi *csi;
	struct resource *res;
	struct device *dev = &pdev->dev;
	int irq;
	int ret;
	const char *stype;
    const char *memtype;


    // isp_con_init(csi);	
	csi = devm_kzalloc(&pdev->dev, sizeof(*csi), GFP_KERNEL);
	if (!csi)
		return -ENOMEM;
    csi->dev = dev;
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	csi->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(csi->regs))
		return PTR_ERR(csi->regs);
	 printk("a018=%x\n",csi->regs);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	csi->ctl_regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(csi->ctl_regs))
		return PTR_ERR(csi->ctl_regs);
    wokoo_csi_get_ofdata_str_type(csi);
    stype=csi->type;
    memtype=csi->memsize;
    csi->clk_out_bus = devm_clk_get(&pdev->dev, "clkout2_bus_clk");
    csi->clk_out_bus_1= devm_clk_get(&pdev->dev, "clkout1_bus_clk");
	if (IS_ERR(csi->clk_out_bus)) {
		dev_err(&pdev->dev, "can't get csi clk_out_bus clock\n");
		return PTR_ERR(csi->clk_out_bus);
	}
	clk_set_rate(csi->clk_out_bus, 24000000);
    clk_set_rate(csi->clk_out_bus_1, 24000000);
	ret = clk_prepare_enable(csi->clk_out_bus);
    ret = clk_prepare_enable(csi->clk_out_bus_1);

	csi->isp_axi_clk = devm_clk_get(&pdev->dev, "axiclk");
	if (IS_ERR(csi->isp_axi_clk)) {
		dev_err(&pdev->dev, "can't get csi isp_axi_clk clock\n");
		return PTR_ERR(csi->isp_axi_clk);
	}
	clk_set_rate(csi->isp_axi_clk, 192000000);
	ret = clk_prepare_enable(csi->isp_axi_clk);
	// printk("isp_axi_clk= %lx\n", clk_get_rate(csi->isp_axi_clk));
	if (ret) {
		dev_err(&pdev->dev, "can't enable csi isp_axi_clk clock\n");
		return ret;
	}
	csi->cphy_cfg_clk = devm_clk_get(&pdev->dev, "phyclk");
	if (IS_ERR(csi->cphy_cfg_clk)) {
		dev_err(&pdev->dev, "can't get csi cphy_cfg_clk clock\n");
		return PTR_ERR(csi->cphy_cfg_clk);
	}
	ret = clk_prepare_enable(csi->cphy_cfg_clk);
	if (ret) {
		dev_err(&pdev->dev, "can't enable csi cphy_cfg_clk clock\n");
		return ret;
	}
	csi->isp_hclk = devm_clk_get(&pdev->dev, "hclk");
	if (IS_ERR(csi->isp_hclk)) {
		dev_err(&pdev->dev, "can't get csi isp_hclk clock\n");
		return PTR_ERR(csi->isp_hclk);
	}
	ret = clk_prepare_enable(csi->isp_hclk);
	if (ret) {
		dev_err(&pdev->dev, "can't enable csi isp_hclk clock\n");
		return ret;
	}
	csi->isp_p_sclk = devm_clk_get(&pdev->dev, "isppclk");
	if (IS_ERR(csi->isp_p_sclk)) {
		dev_err(&pdev->dev, "can't get csi isp_p_sclk clock\n");
		return PTR_ERR(csi->isp_p_sclk);
	}
	ret = clk_prepare_enable(csi->isp_p_sclk);
	if (ret) {
		dev_err(&pdev->dev, "can't enable csi isp_p_sclk clock\n");
		return ret;
	}
#if 1	
	csi->pclk = devm_clk_get(&pdev->dev, "pclk");
	if (IS_ERR(csi->pclk)) {
		dev_err(&pdev->dev, "can't get csi pclk clock\n");
		return PTR_ERR(csi->pclk);
	}
	ret = clk_prepare_enable(csi->pclk);
	if (ret) {
		dev_err(&pdev->dev, "can't enable csi pclk clock\n");
		return ret;
	}
#endif
	csi->isp_psclk_sclk2 = devm_clk_get(&pdev->dev, "psclk");
	if (IS_ERR(csi->isp_psclk_sclk2)) {
		dev_err(&pdev->dev, "can't get csi isp_psclk_sclk2 clock\n");
		return PTR_ERR(csi->isp_psclk_sclk2);
	}
	clk_set_rate(csi->isp_psclk_sclk2, 192000000);
	ret = clk_prepare_enable(csi->isp_psclk_sclk2);
	if (ret) {
		dev_err(&pdev->dev, "can't enable csi isp_psclk_sclk2 clock\n");
		return ret;
	}

	if(!strcmp(stype, "dvp")){
		csi->lvds_idsp_sclk = devm_clk_get(&pdev->dev, "idspclk");
		if (IS_ERR(csi->lvds_idsp_sclk)) {
			dev_err(&pdev->dev, "can't get csi isp_axi_clk clock\n");
			return PTR_ERR(csi->lvds_idsp_sclk);
		}
		ret = clk_prepare_enable(csi->lvds_idsp_sclk);
		if (ret) {
			dev_err(&pdev->dev, "can't enable csi isp_axi_clk clock\n");
			return ret;
		}
    }

	csi->rstp = devm_reset_control_get(&pdev->dev, "resetp");
	if (!IS_ERR(csi->rstp)) {
	reset_control_assert(csi->rstp);
	udelay(100);
	reset_control_deassert(csi->rstp);
	}
	csi->rsta = devm_reset_control_get(&pdev->dev, "resetahb");
	if (!IS_ERR(csi->rsta)) {
		reset_control_assert(csi->rsta);
		udelay(100);
		reset_control_deassert(csi->rsta);
	}
	// isp_con_init(csi);
	global_csi = csi;
	// global_csi = csi;		
    // isp_con_init(csi);
	/* Alloc buffer for DMA pool */
    if(!strcmp(memtype, "1M")){
    img_buff = dma_alloc_coherent(&pdev->dev,SZ_1M,&csi->tx_bb_dma_addr, GFP_DMA);
        if (!img_buff) {
        dev_err(csi->dev, "ISP dma alloc failed\n");
        }
    }
    if(!strcmp(memtype, "2M")){
        img_buff = dma_alloc_coherent(&pdev->dev,SZ_2M,&csi->tx_bb_dma_addr, GFP_DMA);
        if (!img_buff) {
        dev_err(csi->dev, "ISP dma alloc failed\n");
        }
    }
    if(!strcmp(memtype, "3M")){
        img_buff = dma_alloc_coherent(&pdev->dev,SZ_3M,&csi->tx_bb_dma_addr, GFP_DMA);
        if (!img_buff) {
        dev_err(csi->dev, "ISP dma alloc failed\n");
        }
    }
	isp_con_init(global_csi);
	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
	{
	return -1;
	}
	ret = devm_request_irq(&pdev->dev, irq,wokoo_isp_irq, 0,dev_name(&pdev->dev), csi);//IRQF_SHARED
	// ret = devm_request_irq(&pdev->dev, irq,wokoo_isp_irq, IRQF_SHARED,dev_name(&pdev->dev), csi);//IRQF_SHARED
	if (ret) {
		dev_err(&pdev->dev,"request_irq failed with err %d \n", ret);
		return ret;
	}
	platform_set_drvdata(pdev, csi);
	//phy_address_img = virt_to_phys(img_buff);
	printk("ISP vir_address_img  =%x\n",&img_buff);
	phy_address_img =  csi->tx_bb_dma_addr;
	printk("phy_address_img  =%x\n",phy_address_img);
	return 0;
}

static int wokoo_csi_remove(struct platform_device *pdev)
{
	// struct wokoo_csi *dev = filp->private_data;
	isp_exit(global_csi);
	return 0;
}

static const struct of_device_id wokoo_csi_of_match[] = {
	{ .compatible = "wokoo,wokoo-isp" },
	{ /* Sentinel */ }
};
MODULE_DEVICE_TABLE(of, wokoo_csi_of_match);

static int __maybe_unused wokoo_csi_runtime_resume(struct device *dev)
{
	struct wokoo_csi *csi = dev_get_drvdata(dev);
	int ret;
	if (csi ->issup == false)
	{
        return 0;
	}
	printk("runtime_resume\n");
	clk_set_rate(csi->isp_axi_clk, 192000000);
	ret = clk_prepare_enable(csi->isp_axi_clk);
	if (ret) {
	dev_err(dev, "can't enable csi isp_axi_clk clock\n");
	//return ret;
	}
	ret = clk_prepare_enable(csi->cphy_cfg_clk);
	if (ret) {
		dev_err(dev, "can't enable csi cphy_cfg_clk clock\n");
	//	return ret;
	}
	ret = clk_prepare_enable(csi->isp_hclk);
	if (ret) {
		dev_err(dev, "can't enable csi isp_hclk clock\n");
	//	return ret;
	}
	ret = clk_prepare_enable(csi->isp_p_sclk);
	if (ret) {
	dev_err(dev, "can't enable csi isp_p_sclk clock\n");
	//return ret;
	}
	ret = clk_prepare_enable(csi->pclk);
	if (ret) {
	dev_err(dev, "can't enable csi pclk clock\n");
	//return ret;
	}
	clk_set_rate(csi->isp_psclk_sclk2, 192000000);
	ret = clk_prepare_enable(csi->isp_psclk_sclk2);
	if (ret) {
	dev_err(dev, "can't enable csi isp_psclk_sclk2 clock\n");
	//return ret;
	}

	clk_set_rate(csi->clk_out_bus, 24000000);
	ret = clk_prepare_enable(csi->clk_out_bus);
	if (ret) {
	dev_err(dev, "can't enable csi clk_out_bus clock\n");
	//return ret;
	}
	reset_control_assert(csi->rstp);
	udelay(100);
	reset_control_deassert(csi->rstp);
	reset_control_assert(csi->rsta);
	udelay(100);
	reset_control_deassert(csi->rsta);
	csi ->issup = false;
	//printk("wokoo_csi_runtime_resume\n");
	return 0;
}

static int __maybe_unused wokoo_csi_runtime_suspend(struct device *dev)
{
	struct wokoo_csi *csi = dev_get_drvdata(dev);
	if (csi ->issup)
	{
        return 0;
	}
    printk("runtime_suspend\n");
	clk_disable_unprepare(csi->isp_psclk_sclk2);
	clk_disable_unprepare(csi->pclk);
	clk_disable_unprepare(csi->isp_p_sclk);
	clk_disable_unprepare(csi->isp_hclk);
	clk_disable_unprepare(csi->cphy_cfg_clk);
	clk_disable_unprepare(csi->isp_axi_clk);
	clk_disable_unprepare(csi->clk_out_bus);
    printk("wokoo_csi_runtime_suspend\n");
	csi ->issup = true;
	return 0;
}

static const struct dev_pm_ops wokoo_csi_pm_ops = {
	SET_RUNTIME_PM_OPS(wokoo_csi_runtime_suspend,wokoo_csi_runtime_resume,NULL)
	SET_SYSTEM_SLEEP_PM_OPS(wokoo_csi_runtime_suspend,wokoo_csi_runtime_resume)
};

static struct platform_driver wokoo_csi_driver = {
	.probe	= wokoo_csi_probe,
	.remove	= wokoo_csi_remove,
	.driver	= {
		.name		= "wokoo-csi",
		.of_match_table	= wokoo_csi_of_match,
		.pm		= &wokoo_csi_pm_ops,
	},
};

module_platform_driver(wokoo_csi_driver);
MODULE_DESCRIPTION("wokoo Camera Sensor Interface driver");
MODULE_AUTHOR("www.codefairsemi.com");
MODULE_LICENSE("GPL");
