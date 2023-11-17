#!/bin/bash

#
# This Script is by Iceows - build phenix kernel (4.9)
# use linaro 7.5 toolchain
#
# 
# Download linearo toolschain > 4.9, for example
#	https://releases.linaro.org/components/toolchain/binaries/7.5-2019.12/aarch64-elf/
# Extract to 
#	~/gcc/
#
#

rm -rf out/arch/arm64/boot/Image.gz
export PATH=$PATH:~/gcc/gcc-linaro-7.5.0-2019.12-x86_64_aarch64-elf/bin
export CROSS_COMPILE=aarch64-elf-

export GCC_COLORS=auto
export ARCH=arm64

if [ ! -d "out" ];
then
	mkdir out
fi
	

#make ARCH=arm64 distclean
make ARCH=arm64 O=out phenix_defconfig
make ARCH=arm64 O=out -j8

rm -rf Phenix_Kirin65x-4.9.337*.img


#BOARD_KERNEL_CMDLINE loglevel=4 coherent_pool=512K page_tracker=on slub_min_objects=12 unmovable_isolate1=2:192M,3:224M,4:256M printktimer=0xfff0a000,0x534,0x538 androidboot.selinux=enforcing buildvariant=user
#BOARD_KERNEL_BASE 0x00478000
#BOARD_NAME
#BOARD_PAGE_SIZE 2048
#BOARD_HASH_TYPE sha1
#BOARD_KERNEL_OFFSET 0x00008000
#BOARD_RAMDISK_OFFSET 0x0ff88000
#BOARD_SECOND_OFFSET 0x00e88000
#BOARD_TAGS_OFFSET 0x07988000
#BOARD_OS_VERSION 9.0.0
#BOARD_OS_PATCH_LEVEL 2020-10
#BOARD_HEADER_VERSION 0


# androidboot.selinux=enforcing or androidboot.selinux=permissive
#  ramoops.mem_adress=0x3480000 ramoops.mem_size=0x100000 ramoops.mem_type=0 ramoops.console_size=0x40000 ramoops.record_size=0x40000 ramoops.ftrace_size=0x40000 ramoops.dump_oops=1 ramoops.ecc=0
./tools/mkbootimg --kernel out/arch/arm64/boot/Image.gz --base 0x00478000 --cmdline "loglevel=4 ramoops.mem_adress=0x3480000 ramoops.mem_size=0x100000 ramoops.mem_type=0 ramoops.console_size=0x40000 ramoops.record_size=0x40000 ramoops.ftrace_size=0x40000 ramoops.dump_oops=1 ramoops.ecc=0 coherent_pool=512K page_tracker=on slub_min_objects=12 unmovable_isolate1=2:192M,3:224M,4:256M printktimer=0xfff0a000,0x534,0x538 androidboot.selinux=permissive buildvariant=user" --kernel_offset 0x00008000 --ramdisk_offset 0x0ff88000 --second_offset 0x00e88000 --tags_offset 0x07988000 --header_version 0 --output Phenix_Kirin65x-4.9.337-permissive.img

./tools/mkbootimg --kernel out/arch/arm64/boot/Image.gz --base 0x00478000 --cmdline "loglevel=4 ramoops.mem_adress=0x3480000 ramoops.mem_size=0x100000 ramoops.mem_type=0 ramoops.console_size=0x40000 ramoops.record_size=0x40000 ramoops.ftrace_size=0x40000 ramoops.dump_oops=1 ramoops.ecc=0 coherent_pool=512K page_tracker=on slub_min_objects=12 unmovable_isolate1=2:192M,3:224M,4:256M printktimer=0xfff0a000,0x534,0x538 androidboot.selinux=enforcing buildvariant=user" --kernel_offset 0x00008000 --ramdisk_offset 0x0ff88000 --second_offset 0x00e88000 --tags_offset 0x07988000 --header_version 0 --output Phenix_Kirin65x-4.9.337-enforcing.img




ls Phenix_Kirin65x*


