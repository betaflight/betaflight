#!/bin/bash

targets=("PUBLISHMETA=True" \
    "RUNTESTS=True" \
    "TARGET=CC3D" \
    "TARGET=CC3D_OPBL" \
    "TARGET=CHEBUZZF3" \
    "TARGET=COLIBRI_RACE" \
    "TARGET=LUX_RACE" \
    "TARGET=EUSTM32F103RC" \
    "TARGET=SPRACINGF3" \
    "TARGET=SPRACINGF3MINI" \
    "TARGET=NAZE" \
    "TARGET=NAZE32PRO" \
    "TARGET=OLIMEXINO" \
    "TARGET=PORT103R" \
    "TARGET=RMDO" \
    "TARGET=SPARKY" \
    "TARGET=STM32F3DISCOVERY" \
    "TARGET=ALIENFLIGHTF1" \
    "TARGET=ALIENFLIGHTF3")

#fake a travis build environment
export TRAVIS_BUILD_NUMBER=$(date +%s)
export BUILDNAME=${BUILDNAME:=fake_travis}
export TRAVIS_REPO_SLUG=${TRAVIS_REPO_SLUG:=$USER/simulated}

for target in "${targets[@]}"
do
	unset RUNTESTS PUBLISHMETA TARGET
	eval "export $target"
	make -f Makefile clean
	./.travis.sh
done
