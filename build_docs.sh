#!/bin/bash



doc_files=( 'oasdf ne' 'two' 'three' )
for i in "${doc_files[@]}"
do
	echo $i
done

exit;

filename=Manual
if which gimli >/dev/null; then
	echo "Building ${filename}.pdf"
	find docs -name "*.md" -exec cat {} > ${filename}.md \;
	mv ${filename}.md docs
	pushd . >/dev/null
	cd docs
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




