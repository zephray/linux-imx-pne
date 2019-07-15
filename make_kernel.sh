#!/bin/sh
export CPUS=`grep -c processor /proc/cpuinfo`
export ARCH=arm
export CROSS_COMPILE=arm-linux-gnueabihf-
#time make -j${CPUS}
time make
