#!/bin/bash

FC_VER=$(make version)
REVISION=$(git rev-parse --short HEAD)
BRANCH=$(git rev-parse --abbrev-ref HEAD)
REVISION=$(git rev-parse --short HEAD)
LAST_COMMIT_DATE=$(git log -1 --date=short --format="%cd")
TARGET_FILE=obj/betaflight_${FC_VER}_${TARGET}
TRAVIS_REPO_SLUG=${TRAVIS_REPO_SLUG:=$USER/undefined}
BUILDNAME=${BUILDNAME:=travis}
TRAVIS_BUILD_NUMBER=${TRAVIS_BUILD_NUMBER:=undefined}

MAKE="make EXTRA_FLAGS=-Werror"

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

# A hacky way of building the docs at the same time as the normal builds.
if [ $PUBLISHDOCS ] ; then
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

elif [ $TARGET ] ; then
    $MAKE $TARGET || exit $?

	if [ $PUBLISH_URL ] ; then
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
	fi

elif [ $GOAL ] ; then
    if [ "test-all" == "$GOAL" ] ; then
        $MAKE check-target-independence || exit $?
        $MAKE check-fastram-usage-correctness || exit $?
        $MAKE check-platform-included || exit $?
    else
	 export V=0
    fi

    $MAKE $GOAL || exit $?

    if [ $PUBLISHCOV ] ; then
        if [ "test-all" == "$GOAL" ] ; then
            lcov --directory . -b src/test --capture --output-file coverage.info 2>&1 | grep -E ":version '402\*', prefer.*'406\*" --invert-match
            lcov --remove coverage.info 'lib/test/*' 'src/test/*' '/usr/*' --output-file coverage.info # filter out system and test code
            lcov --list coverage.info # debug before upload
            coveralls-lcov coverage.info # uploads to coveralls
        fi
    fi
else
    $MAKE all
fi
