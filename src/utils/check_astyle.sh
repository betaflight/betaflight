#!/bin/sh

git diff-files --quiet 2>/dev/null
UNSTAGED=$?

git diff-index --cached --quiet HEAD 2>/dev/null
UNCOMMITTED=$?

if [ ${UNSTAGED} -gt 0 -o ${UNCOMMITTED} -gt 0 ]; then
    echo "Unstaged / uncommitted changes, aborting!"

    exit 2
fi

./src/utils/astyle.sh

git diff

git diff --quiet HEAD 2>/dev/null
DIFF=$?

if [ ${DIFF} -gt 0 ]; then
     echo "Style error! Run astyle: ./src/utils/astyle.sh"

     git reset --hard 2>/dev/null 1>&2
     git clean -f 2>/dev/null 1>&2

     exit 1
fi;
