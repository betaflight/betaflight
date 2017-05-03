echo "Building target" $1
docker run --rm -v `pwd`:/home/src/ inav make TARGET=$1