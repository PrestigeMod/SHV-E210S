#!/bin/bash

export ARCH=arm
export CROSS_COMPILE=/opt/toolchains/arm-eabi-4.4.3/bin/arm-eabi-
#export CROSS_COMPILE="/home/yyoung.kim/Toolchain/prebuilt/linux-x86/toolchain/arm-eabi-4.4.3/bin/arm-eabi-"
export USE_SEC_FIPS_MODE=true

make c1skt_00_defconfig
make
