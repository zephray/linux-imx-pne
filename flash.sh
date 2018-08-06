#!/bin/sh
sudo dd if=zImage-with-dtb.imx of=/dev/mmcblk0 bs=1k seek=1 conv=fsync
sync

