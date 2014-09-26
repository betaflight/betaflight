#!/bin/bash


if which gimli >/dev/null; then
	find docs -name "*.md" -exec cat {} > documentation.md \;
	gimli -f documentation.md
	mv documentation.pdf docs
	rm documentation.md    
else
    echo -e "\nFAILED"
	echo "Install Gimli to build the PDF documentation"
	echo -e "https://github.com/walle/gimli\n"
	exit 1
fi




