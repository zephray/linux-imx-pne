#!/bin/sh
export CPUS=`grep -c processor /proc/cpuinfo`
export ARCH=arm
export CROSS_COMPILE=/opt/gcc-linaro-5.5.0-2017.10-x86_64_arm-linux-gnueabihf/bin/arm-linux-gnueabihf-
export PATH=/opt/gcc-linaro-5.5.0-2017.10-x86_64_arm-linux-gnueabihf/bin:$PATH
time make -j${CPUS}
