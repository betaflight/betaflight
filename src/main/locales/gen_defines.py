#!/usr/bin/env python

import sys
import json

# do this


def wr_bf_header(outFile, filename):
    with open(filename, "r") as fr:
        outFile.write(fr.read())

def wr_define(outFile, id, mes, desc, error):
    if error == "":
        newline = "#define " + id + " " * max(1, 30 - len(id)) + mes + " " * max(1, 38 - len(mes)) + "\t// " + desc
    else:
        newline = id + ": " + error + " " * max(1, 30 - len(id)) + mes + " " * max(1, 38 - len(mes)) + "\t// " + desc + '\n'
    print(newline, file=outFile)
    # debug print(newline)

hdKey = "_HD"
lenSearch = "Max length:"
lenDelimiter = ";"

language = sys.argv[1]
from_filename = sys.argv[2]
to_filename = language + '/bf_locale.h'

print(language + ': ' + from_filename + ' -> '+ to_filename)

fw = open(to_filename, 'w')
wr_bf_header(fw, "bf_header")

with open(from_filename, "r") as fr:
    # Converting JSON encoded data into Python dictionary
    translations = json.load(fr)
    prevKey = ""
    prevMessage = prevDescription = ""

    for key, value in translations.items():
        currentKey = key
        # get translation and description
        message = value.get('message')
        description = value.get('description')
        # get number of maximum lenght in "Max length: 10; description of text"
        start = description.find(lenSearch) + len(lenSearch)
        end = description.find(lenDelimiter, start)
        maxLen = int(description[start:end].strip())
        # debug print(" currentKey: ", currentKey, ": prevKey: ", prevKey, " maxLen: ", maxLen)

        if len(message) > maxLen and maxLen > 0:
            errorMess = "ERROR - maximum length " + str(maxLen) + " exceed for: " + message + '/' + str(len(message))
            print(errorMess)
        else:
            errorMess = ""

        if currentKey == prevKey + hdKey:
            mess = 'TR2("' + prevMessage + '", "' + message + '")'
            wr_define(fw, prevKey, mess, prevDescription + "; HD> " + description, errorMess)
        else:
            # debug print("2 currentKey: ", currentKey, ": prevKey: ", prevKey)
            if prevKey != "" and not prevKey.find(hdKey) > 0:
                wr_define(fw, prevKey, '"' + prevMessage + '"', prevDescription, errorMess)
        prevKey = currentKey
        prevMessage = message
        prevDescription = description
    # for

    print("Done reading json file\n")
# with

fr.close()
fw.close()
