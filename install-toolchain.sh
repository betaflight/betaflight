#!/bin/bash

if [ ! -d $PWD/gcc-arm-none-eabi-6_2-2016q4/bin ] ; then
    curl --retry 10 --retry-max-time 120 -L "https://armkeil.blob.core.windows.net/developer/Files/downloads/gnu-rm/6-2017q2/gcc-arm-none-eabi-6-2017-q2-update-linux.tar.bz2" | tar xfj -
fi
