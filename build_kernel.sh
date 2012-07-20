#!/bin/bash

export KERNELDIR=`readlink -f .`
export RAMFS_SOURCE=`readlink -f $KERNELDIR/../SHV-E210S/ramdisk`
export PARENT_DIR=`readlink -f ..`
export USE_SEC_FIPS_MODE=true
export CROSS_COMPILE=/home/privilege/prebuilt/linux-x86/toolchain/arm-eabi-4.4.3/bin/arm-eabi-

if [ "${1}" != "" ];then
  export KERNELDIR=`readlink -f ${1}`
fi

RAMFS_TMP="/tmp/ramfs_tmp"

if [ ! -f $KERNELDIR/.config ];
then
  make c1skt_00_defconfig
fi

make c1skt_00_defconfig


. $KERNELDIR/.config

export ARCH=arm

cd $KERNELDIR/
nice -n 10 make -j4 || exit 1

rm -rf $RAMFS_TMP
rm -rf $RAMFS_TMP.cpio
rm -rf $RAMFS_TMP.cpio.gz

cp -ax $RAMFS_SOURCE $RAMFS_TMP
find $RAMFS_TMP -name .git -exec rm -rf {} \;
find $RAMFS_TMP -name EMPTY_DIRECTORY -exec rm -rf {} \;

rm -rf $RAMFS_TMP/tmp/*
#remove mercurial repository
rm -rf $RAMFS_TMP/.hg
#copy modules into ramfs
mkdir -p $INITRAMFS/lib/modules

find -name '*.ko' -exec cp -av {} 
$RAMFS_TMP/lib/modules/ \;
${CROSS_COMPILE}strip --strip-unneeded $RAMFS_TMP/lib/modules/*

cd $RAMFS_TMP
find | fakeroot cpio -H newc -o > $RAMFS_TMP.cpio 2>/dev/null
ls -lh $RAMFS_TMP.cpio
gzip -9 $RAMFS_TMP.cpio
cd -

nice -n 10 make -j4 zImage || exit 1

./mkbootimg --kernel $KERNELDIR/arch/arm/boot/zImage --ramdisk $RAMFS_TMP.cpio.gz --board smdk4x12 --base 0x10000000 --pagesize 2048 --ramdiskaddr 0x11000000 -o $KERNELDIR/boot.img
