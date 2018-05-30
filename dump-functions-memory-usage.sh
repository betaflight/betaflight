#!/bin/sh

if [ $# -ne 1 ]; then
	echo "USAGE: $0 <filename.elf>";
	exit;
fi

$(make arm_sdk_prefix)objdump -t $1 |
awk '{ if ($3 == "F") print $NF " 0x" $5 }' |
sed 's/.lto_priv.[0-9]*//;s/.constprop.[0-9]*//' |
sort;
