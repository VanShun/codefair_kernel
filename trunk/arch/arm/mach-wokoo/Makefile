# SPDX-License-Identifier: GPL-2.0
#
# Copyright (c) 2020 Codefair Semiconductor Technology Co., Ltd.
# 		http://www.codefairsemi.com

ccflags-$(CONFIG_ARCH_MULTIPLATFORM) += -I$(srctree)/arch/arm/plat-wokoo/include

# Core

obj-$(CONFIG_PM_SLEEP)		+= pm.o sleep.o

# machine support

obj-y				+= wokoo.o
