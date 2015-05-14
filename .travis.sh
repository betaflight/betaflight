#!/bin/bash
REVISION=$(git rev-parse --short HEAD)
BRANCH=$(git rev-parse --abbrev-ref HEAD)
REVISION=$(git rev-parse --short HEAD)
LAST_COMMIT_DATE=$(git log -1 --date=short --format="%cd")
TARGET_FILE=obj/cleanflight_${TARGET}

# A hacky way of running the unit tests at the same time as the normal builds.
if [ $RUNTESTS ] ; then
	cd ./src/test && make test

# A hacky way of building the docs at the same time as the normal builds.
elif [ $PUBLISHDOCS ] ; then
	if [ $PUBLISH_URL ] ; then

		sudo apt-get install zlib1g-dev libssl-dev wkhtmltopdf libxml2-dev libxslt-dev #ruby-rvm
		
		# Patch Gimli to fix underscores_inside_words
		curl -L https://github.com/walle/gimli/archive/v0.5.9.tar.gz | tar zxf -
		
		sed -i 's/).render(/, :no_intra_emphasis => true).render(/' gimli-0.5.9/ext/github_markup.rb
		
		cd gimli-0.5.9/
		gem build gimli.gemspec && rvmsudo gem install gimli
		cd ../

		./build_docs.sh

		curl -k \
			--form "manual=@docs/Manual.pdf" \
			--form "revision=${REVISION}" \
			--form "branch=${BRANCH}" \
			--form "last_commit_date=${LAST_COMMIT_DATE}" \
			--form "travis_build_number=${TRAVIS_BUILD_NUMBER}" \
			${PUBLISH_URL}
	fi

elif [ $PUBLISHMETA ] ; then
	if [ $PUBLISH_URL ] ; then
		RECENT_COMMITS=$(git shortlog -n25)
		curl -k \
			--form "recent_commits=${RECENT_COMMITS}" \
			--form "revision=${REVISION}" \
			--form "branch=${BRANCH}" \
			--form "last_commit_date=${LAST_COMMIT_DATE}" \
			--form "travis_build_number=${TRAVIS_BUILD_NUMBER}" \
			${PUBLISH_URL}
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
	
		curl -k \
			--form "file=@${TARGET_FILE}" \
			--form "revision=${REVISION}" \
			--form "branch=${BRANCH}" \
			--form "last_commit_date=${LAST_COMMIT_DATE}" \
			--form "travis_build_number=${TRAVIS_BUILD_NUMBER}" \
			${PUBLISH_URL}
	else
		make -j2
	fi
fi

