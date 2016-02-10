#!/bin/bash

targets=("PUBLISHMETA=True" \
    "RUNTESTS=True" \
    "TARGET=SPRACINGF3MINI" \
    "TARGET=SPRACINGF3" \
    "TARGET=NAZE" \
    "TARGET=CC3D" \
    "TARGET=CJMCU" \
    "TARGET=SPARKY" \
    "TARGET=COLIBRI_RACE" \
    "TARGET=LUX_RACE" \
    "TARGET=MOTOLAB" \
    "TARGET=RMDO" \
    "TARGET=ALIENWIIF3" \
    "TARGET=ALIENWIIF1" \
    "TARGET=STM32F3DISCOVERY" \
    "TARGET=PORT103R" \
    "TARGET=EUSTM32F103RC" \
    "TARGET=CHEBUZZF3" \
    "TARGET=OLIMEXINO" \
    "TARGET=NAZE32PRO" )

#fake a travis build environment
export TRAVIS_BUILD_NUMBER=$(date +%s)
export BUILDNAME=${BUILDNAME:=fake_travis}
export TRAVIS_REPO_SLUG=${TRAVIS_REPO_SLUG:=$USER/simulated}

for target in "${targets[@]}"
do
	unset RUNTESTS PUBLISHMETA TARGET
	eval "export $target"
	make clean
	./.travis.sh
done
