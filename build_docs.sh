#!/bin/bash

filename=Manual
doc_files=(
	'Introduction.md'
	'Getting Started.md'
	'Safety.md'
	'Installation.md'
	'Configuration.md'
	'Cli.md'
	'Serial.md'
	'Rx.md'
	'Spektrum bind.md'
	'Failsafe.md'
	'Battery.md'
	'Gps.md'
	'Rssi.md'
	'Telemetry.md'
	'LedStrip.md'
	'Display.md'
	'Buzzer.md'
	'Sonar.md'
	'Profiles.md'
	'Modes.md'
	'Inflight Adjustments.md'
	'Controls.md'
	'Autotune.md'
	'Blackbox.md'
	'Migrating from baseflight.md'
	'Boards.md'
	'Board - AlienWii32.md'
	'Board - CC3D.md'
	'Board - CJMCU.md'
	'Board - Naze32.md'
	'Board - Sparky.md'
	'Board - Olimexino.md'
	'Board - ChebuzzF3.md'
)

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
	gimli -f ${filename}.md -stylesheet override.css \
	  -w '--toc --title "Cleanflight Manual" --footer-right "[page]" --toc-depth 1'
	rm ${filename}.md
	popd >/dev/null
else
	echo -e "\nFAILED"
	echo "Install Gimli to build the PDF documentation"
	echo -e "https://github.com/walle/gimli\n"
	exit 1
fi
