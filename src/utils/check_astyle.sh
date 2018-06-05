#!/bin/sh

./src/utils/astyle.sh

git diff

git diff --quiet HEAD 2>/dev/null

if [ $? -gt 0 ]; then
     echo "Style error! Run astyle: ./src/utils/astyle.sh"

     git reset --hard 2>/dev/null 1>&2
     git clean -f 2>/dev/null 1>&2

     exit 1
fi;
