#!/usr/bin/env python

import sys
import xml.etree.ElementTree as ET

# do this
# generate language specific file from language/bf_locale.json to language/bf_locale.h 
# py gen_defines.py source language 

def wr_bf_header(outFile, filename):
    with open(filename, "r") as fr:
        outFile.write(fr.read())

def wr_define(outFile, id, mes, desc, error):
    if error == "":
        newline = "#define " + id + " " * max(1, 35 - len(id)) + mes + " " * max(1, 43 - len(mes)) + "\t// " + desc
    else:
        newline = id + ": " + error + " " * max(1, 35 - len(id)) + mes + " " * max(1, 43 - len(mes)) + "\t// " + desc + '\n'
    print(newline, file=outFile)

hdKey = "_HD"

source   = sys.argv[1]
language = sys.argv[2]
from_filename  = source + '/' + language + '/bf_locale.xml'
to_filename    = source + '/' + language + '/bf_locale.h'
bf_header_name = source + '/bf_header'

print(sys.argv[0] + ': GENERATE >' + language + '< ' + from_filename + ' to '+ to_filename)

fw = open(to_filename, 'w')
wr_bf_header(fw, bf_header_name)

prevKey = ""
prevMessage = prevDescription = ""

tree = ET.parse(from_filename)
root = tree.getroot()

for string_element in root.findall('string'):
    currentKey = string_element.get('name')
    maxLen = int(string_element.get('maxLength'))
    description = string_element.get('comment')
    message = string_element.text

    if len(message) > maxLen and maxLen > 0:
        errorMess = "ERROR - maximum length " + str(maxLen) + " exceed for: " + message + '/' + str(len(message))
        print(errorMess)
    else:
        errorMess = ""

    if currentKey == prevKey + hdKey:
        mess = 'TR2("' + prevMessage + '", "' + message + '")'
        wr_define(fw, prevKey, mess, prevDescription + "; " + description, errorMess)
    else:
        if prevKey != "" and not prevKey.find(hdKey) > 0:
            wr_define(fw, prevKey, '"' + prevMessage + '"', prevDescription, errorMess)
    prevKey = currentKey
    prevMessage = message
    prevDescription = description
# for
wr_define(fw, currentKey, '"' + message + '"', description, errorMess)

print(sys.argv[0] + ': FINISH')

fw.close()
