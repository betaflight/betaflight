#!/bin/bash

targets=("PUBLISHMETA=True" \
    "RUNTESTS=True" \
    "TARGET=CC3D" \
    "TARGET=CC3D_OPBL" \
    "TARGET=CC3D_BP6" \
    "TARGET=CC3D_OPBL_BP6" \
    "TARGET=IRCFUSIONF3" \
    "TARGET=MOTOLAB" \
    "TARGET=COLIBRI_RACE" \
    "TARGET=SPRACINGF3" \
    "TARGET=NAZE" \
    "TARGET=RMDO" \
    "TARGET=SPARKY" \
    "TARGET=AFROMINI")

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
