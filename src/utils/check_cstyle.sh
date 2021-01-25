#!/bin/bash

FILE_TO_CHECK=${1}
RC=0

# Make sure file exists and has proper suffix.
if [[ ! -f "${FILE_TO_CHECK}" ]] || [[ ! "${FILE_TO_CHECK##*\.}" =~ ^[c,h,C,H]$ ]]; then
    exit ${RC}
fi

TMP_FILE_DIFF=$(mktemp /tmp/betaflight.diff.XXXXXXXXX)
TMP_FILE_ASTYLE=$(mktemp /tmp/betaflight.astyle.XXXXXXXXX)
TMP_FILE_WARNINGS=$(mktemp /tmp/betaflight.warning.XXXXXXXXX)
ASTYLE_OPTIONS="--style=kr
		--indent=spaces=4
		--min-conditional-indent=0
		--max-instatement-indent=80
		--pad-header
		--pad-oper
		--align-pointer=name
		--align-reference=name
		--max-code-length=120
		--convert-tabs
		--preserve-date
		--suffix=none
		--mode=c"

git diff --function-context --unified=1 HEAD ${FILE_TO_CHECK} | sed -e '/diff --git/,+3d;/^@@/d;/^-/d;s/^+/ /' | cut -d' ' -f2- > ${TMP_FILE_DIFF}
astyle ${ASTYLE_OPTIONS} -n < ${TMP_FILE_DIFF} > ${TMP_FILE_ASTYLE}
diff -u ${TMP_FILE_DIFF} ${TMP_FILE_ASTYLE} | tail -n +3 | sed -e '/^-/d;/^@@/d;s/^+/[STYLE WARNING] /' > ${TMP_FILE_WARNINGS}

if [[ -s ${TMP_FILE_WARNINGS} ]]; then
    cat ${TMP_FILE_WARNINGS}
    RC=1
fi

if [[ ${RC} -eq 1 ]]; then
    echo -e "File ${FILE_TO_CHECK} has style warning(s), see" \
	 "https://github.com/betaflight/betaflight/blob/master/docs/development/CodingStyle.md"
fi

rm -f ${TMP_FILE_DIFF} ${TMP_FILE_ASTYLE} ${TMP_FILE_WARNINGS}

exit ${RC}
