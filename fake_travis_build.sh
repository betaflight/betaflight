#!/bin/bash

targets=("PUBLISHMETA=True" \
    "TARGET=CC3D" \
    "TARGET=CC3D_OPBL" \
    "TARGET=COLIBRI_RACE" \
    "TARGET=LUX_RACE" \
    "TARGET=SPRACINGF3" \
    "TARGET=SPRACINGF3EVO" \
    "TARGET=SPRACINGF3MINI" \
    "TARGET=OMNIBUS" \
    "TARGET=NAZE" \
    "TARGET=AFROMINI" \
    "TARGET=BEEBRAIN" \
    "TARGET=RMDO" \
    "TARGET=SPARKY" \
    "TARGET=MOTOLAB" \
    "TARGET=PIKOBLX" \
    "TARGET=IRCFUSIONF3" \
    "TARGET=ALIENFLIGHTF1" \
    "TARGET=ALIENFLIGHTF3" \
    "TARGET=DOGE" \
    "TARGET=SINGULARITY" \
    "TARGET=SIRINFPV" \
    "TARGET=X_RACERSPI" \
    "TARGET=RCEXPLORERF3")


#fake a travis build environment
export TRAVIS_BUILD_NUMBER=$(date +%s)
export BUILDNAME=${BUILDNAME:=fake_travis}
export TRAVIS_REPO_SLUG=${TRAVIS_REPO_SLUG:=$USER/simulated}

for target in "${targets[@]}"
do
	unset RUNTESTS PUBLISHMETA TARGET
	echo
	echo
	echo "BUILDING '$target'"
	eval "export $target"
	make -f Makefile clean
	./.travis.sh
done
