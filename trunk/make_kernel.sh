#/bin/sh

export ARCH=arm
export CROSS_COMPILE=/mnt/data/gcc-arm-8.3-2019.03-x86_64-arm-linux-gnueabihf/bin/arm-linux-gnueabihf-

make uImage LOADADDR=0x8000 -j 8
