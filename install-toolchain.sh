#!/bin/bash

if [ ! -d $PWD/gcc-arm-none-eabi-5_4-2016q2/bin ] ; then
    curl --retry 10 --retry-max-time 120 -L "https://launchpad.net/gcc-arm-embedded/5.0/5-2016-q2-update/+download/gcc-arm-none-eabi-5_4-2016q2-20160622-linux.tar.bz2" | tar xfj -
fi
