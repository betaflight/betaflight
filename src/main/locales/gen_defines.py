#!/usr/bin/env python

# * This file is part of Betaflight.
# *
# * Betaflight is free software. You can redistribute this software
# * and/or modify this software under the terms of the GNU General
# * Public License as published by the Free Software Foundation,
# * either version 3 of the License, or (at your option) any later
# * version.
# *
# * Betaflight is distributed in the hope that it will be useful,
# * but WITHOUT ANY WARRANTY; without even the implied warranty of
# * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# *
# * See the GNU General Public License for more details.
# *
# * You should have received a copy of the GNU General Public
# * License along with this software.
# *
# * If not, see <http://www.gnu.org/licenses/>.

import sys
import xml.etree.ElementTree as ET

# do this
# generate language specific file from locale/bf_locale.xml to locale/bf_locale.h 
# py gen_defines.py source locale 

def wr_bf_header(outFile, filename):
    with open(filename, "r") as fr:
        outFile.write(fr.read())

def wr_bf_notice(outFile, locale):
    outFile.write("/*\n")
    outFile.write("\tNOTICE !\n\tNOTICE !\n\tNOTICE, this header file for LOCALE '" + locale + "' are generated from " + locale + "/bf_locale.xml\n")
    outFile.write("\tChanges to translation ie. "  + locale + "/bf_locale.h must be done in "  + locale + "/bf_locale.xml\n\n")
    outFile.write("\tTo generate use 'make xx LOCALE=" + locale + "'")
    outFile.write("\n\n")
    outFile.write("\tWant to USE_EXTENDED_HD, ie. have #define TXT TR2( \"short\", \"long\"), x, y), \n")
    outFile.write("\twrite string section for TXT with 'short' followed by string section TXT_HD with 'long'\n\n")
    outFile.write("\tTo generate use 'make xx LOCALE=" + locale + " USE_EXTENDED_HD=1'\n")
    outFile.write("*/\n\n")

def wr_define(outFile, id, mes, desc, error):
    if error == "":
        newline = "#define " + id + " " * max(1, 35 - len(id)) + mes + " " * max(1, 43 - len(mes)) + "\t// " + desc
    else:
        newline = id + ": " + error + " " * max(1, 35 - len(id)) + mes + " " * max(1, 43 - len(mes)) + "\t// " + desc + '\n'
    print(newline, file=outFile)

hdKey = "_HD"

source   = sys.argv[1]
locale = sys.argv[2]
from_filename  = source + '/' + locale + '/bf_locale.xml'
to_filename    = source + '/' + locale + '/bf_locale.h'
bf_header_name = source + '/bf_header'

print(sys.argv[0] + ': GENERATE >' + locale + '< ' + from_filename + ' to '+ to_filename)

fw = open(to_filename, 'w')
wr_bf_header(fw, bf_header_name)
wr_bf_notice(fw, locale)

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
