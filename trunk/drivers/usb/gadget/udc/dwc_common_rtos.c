
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/io.h>
#include "dwc_os.h"

void __DWC_ERROR(char *format, ...)
{
}

uint32_t DWC_READ_REG32(uint32_t volatile *reg)
{
	return readl_relaxed(reg);
}

void DWC_WRITE_REG32(uint32_t volatile *reg, uint32_t value)
{
    writel_relaxed(value, reg);
}

void DWC_MODIFY_REG32(uint32_t volatile *reg, uint32_t clear_mask, uint32_t set_mask)
{
    uint32_t value;

	value = readl_relaxed(reg);
	value =(value & ~clear_mask) | set_mask;
	writel_relaxed(value, reg);
}

void DWC_UDELAY(uint32_t usecs)
{
	int i;
	for (i = 0; i < 50; i++);
}

void DWC_MDELAY(uint32_t msecs)
{
	int i;
	for (i = 0; i < msecs; i ++);
}

struct dwc_timer
{
	char *name;
	/*
	struct timer_list *t;
	char *name;
	dwc_timer_callback_t cb;
	void *data;
	uint8_t scheduled;
	dwc_spinlock_t *lock;
	*/
};

void __DWC_FREE(void *addr)
{
}

