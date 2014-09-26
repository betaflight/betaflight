#!/bin/bash

filename=Manual
doc_files=( 'Configuration.md'
	'Board - CC3D.md'
	'Board - Naze32.md'
	'Rx.md'
	'Serial.md'
	'Failsafe.md'
	'Battery.md'
	'Gps.md'
	'Rssi.md'
	'Telemetry.md'
	'LedStrip.md'
	'Display.md'
	'Buzzer.md'
	'Sonar.md'
	'Autotune.md'
	'Migrating from baseflight.md')

if which gimli >/dev/null; then
	echo "Building ${filename}.pdf"
	pushd . >/dev/null
	cd docs
	
	rm -f ${filename}.md
	for i in "${doc_files[@]}"
	do
		cat "$i" >> ${filename}.md
	done
	rm -f ${filename}.pdf
	gimli -f ${filename}.md
	rm ${filename}.md 
	popd >/dev/null
else
	echo -e "\nFAILED"
	echo "Install Gimli to build the PDF documentation"
	echo -e "https://github.com/walle/gimli\n"
	exit 1
fi




