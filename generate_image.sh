#!/bin/sh
cp arch/arm/boot/zImage zImage-with-dtb.bin
cat arch/arm/boot/dts/imx6ul-14x14-pne.dtb >> zImage-with-dtb.bin
../uboot/tools/mkimage -n imximage.cfg -T imximage -e 0x80800000 -d zImage-with-dtb.bin zImage-with-dtb.imx
