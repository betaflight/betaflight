#!/bin/bash
# A hacky way of running the unit tests at the same time as the normal builds.
if [ $RUNTESTS ] ; then
	cd ./src/test && make test
else
	if [ $PUBLISH_URL ] ; then

		make -j2

		TARGET_FILE=obj/cleanflight_${TARGET}

		if [ -f ${TARGET_FILE}.bin ];
                then
                        TARGET_FILE=${TARGET_FILE}.bin
		elif [ -f ${TARGET_FILE}.hex ];
                then
                        TARGET_FILE=${TARGET_FILE}.hex
		else
			echo "build artifact (hex or bin) for ${TARGET_FILE} not found, aborting";
			exit 1
                fi
	
		curl -F file=@${TARGET_FILE} ${PUBLISH_URL}
	else
		make -j2
	fi

fi
