#!/bin/sh

astyle --style=kr --recursive $(pwd)/src/main/\*.h $(pwd)/src/main/\*.c --indent=spaces=4 --indent-after-parens
