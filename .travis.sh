#!/bin/bash

REVISION=$(git rev-parse --short HEAD)
BRANCH=$(git rev-parse --abbrev-ref HEAD)
REVISION=$(git rev-parse --short HEAD)
LAST_COMMIT_DATE=$(git log -1 --date=short --format="%cd")
TARGET_FILE=obj/cleanflight_${TARGET}
TRAVIS_REPO_SLUG=${TRAVIS_REPO_SLUG:=$USER/undefined}
BUILDNAME=${BUILDNAME:=travis}
TRAVIS_BUILD_NUMBER=${TRAVIS_BUILD_NUMBER:=undefined}

CURL_BASEOPTS=(
	"--retry" "10"
	"--retry-max-time" "120" )

CURL_PUB_BASEOPTS=(
	"--form" "revision=${REVISION}"
	"--form" "branch=${BRANCH}"
	"--form" "travis_build_number=${TRAVIS_BUILD_NUMBER}"
	"--form" "last_commit_date=${LAST_COMMIT_DATE}"
	"--form" "github_repo=${TRAVIS_REPO_SLUG}"
	"--form" "build_name=${BUILDNAME}" )

# A hacky way of running the unit tests at the same time as the normal builds.
if [ $RUNTESTS ] ; then
	cd ./src/test && make test

# A hacky way of building the docs at the same time as the normal builds.
elif [ $PUBLISHDOCS ] ; then
	if [ $PUBLISH_URL ] ; then

		# Patch Gimli to fix underscores_inside_words
		curl -L "${CURL_BASEOPTS[@]}" https://github.com/walle/gimli/archive/v0.5.9.tar.gz | tar zxf -

		sed -i 's/).render(/, :no_intra_emphasis => true).render(/' gimli-0.5.9/ext/github_markup.rb

		cd gimli-0.5.9/
		gem build gimli.gemspec && gem install gimli
		cd ../

		./build_docs.sh

		curl -k "${CURL_BASEOPTS[@]}" "${CURL_PUB_BASEOPTS[@]}" --form "manual=@docs/Manual.pdf" ${PUBLISH_URL} || true
	fi

elif [ $PUBLISHMETA ] ; then
	if [ $PUBLISH_URL ] ; then
		RECENT_COMMITS=$(git shortlog -n25)
		curl -k "${CURL_BASEOPTS[@]}" "${CURL_PUB_BASEOPTS[@]}" --form "recent_commits=${RECENT_COMMITS}" ${PUBLISH_URL} || true
	fi

else
	if [ $PUBLISH_URL ] ; then
		make -j2
		if   [ -f ${TARGET_FILE}.bin ] ; then
			TARGET_FILE=${TARGET_FILE}.bin
		elif [ -f ${TARGET_FILE}.hex ] ; then
			TARGET_FILE=${TARGET_FILE}.hex
		else
			echo "build artifact (hex or bin) for ${TARGET_FILE} not found, aborting";
			exit 1
		fi

		curl -k "${CURL_BASEOPTS[@]}" "${CURL_PUB_BASEOPTS[@]}" --form "file=@${TARGET_FILE}" ${PUBLISH_URL} || true
		exit 0;
	else
		make -j2
	fi
fi
