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

import os
import sys

#import defusedxml      // suggested by coderabbitai, but then need to install defusedxml
# try:
#   from defusedxml import ElementTree as ET  # safer XML parsing
# except ImportError:
import xml.etree.ElementTree as ET

# do this
# generate language specific file from locale/bf_locale.xml to locale/bf_locale.h 
# also generate fallback if new defines not yet are translated (ie. defined)
# 
# syntax: py gen_defines.py TASK source locale 
#         TASK: BF ->locale/bf_header.h
#         TASK: UT ->untranslated.h

hdKey = "_HD"

def wr_bf_header(outFile, filename):
    with open(filename, "r", encoding="utf-8") as fr:
        outFile.write(fr.read())

def wr_bf_notice_bf(outFile, locale):
    outFile.write("/*\n")
    outFile.write("\tNOTICE!\n\tNOTICE: this header file for LOCALE '" + locale + "' is generated from " + locale + "/bf_locale.xml\n")
    outFile.write("\tChanges to translations (i.e., "  + locale + "/bf_locale.h) must be made in "  + locale + "/bf_locale.xml\n\n")   outFile.write("\tTo enable TR2 (HD extended text), write TXT then TXT_HD and build with USE_HD_EXTENDED=1\n")
    outFile.write("\tTo build use 'make <target> LOCALE=" + locale + "'\n")
    outFile.write("*/\n\n")

def wr_bf_notice_ut(outFile):
    outFile.write("/*\n")
    outFile.write("\tNOTICE !\n\tNOTICE, this header file is generated from en/bf_locale.xml\n")
    outFile.write("\tChanges to translation ie. en/bf_locale.h must be made in en/bf_locale.xml\n\n")
    outFile.write("\tThis file provides fallback defines for strings not yet translated in a locale, allowing targets to compile.\n")
    outFile.write("*/\n\n")

def wr_define_bf(outFile, id, mes, desc, error):
    if error == "":
        newline = "#define " + id + " " * max(1, 35 - len(id)) + mes + " " * max(1, 43 - len(mes)) + "\t// " + desc
    else:
        newline = id + ": " + error + " " * max(1, 35 - len(id)) + mes + " " * max(1, 43 - len(mes)) + "\t// " + desc + '\n'
        sys.exit(1)
    print(newline, file=outFile)

def wr_define_ut(outFile, id, mes, error):
    if error == "":
        newline = "#ifndef " + id + "\n   #define " + id + " " * max(1, 35 - len(id)) + mes + "\n#endif\n"
    print(newline, file=outFile)

if os.getenv('V1') == '@':
    verbose = False
else:
    verbose = True

if len(sys.argv) < 4:
    print("Usage: gen_defines.py TASK(BF|UT) <source_dir> <locale>", file=sys.stderr)
    sys.exit(2)
task   = sys.argv[1]
source = sys.argv[2]
locale = sys.argv[3]

if task not in ("BF", "UT"):
    print(f"Unknown TASK '{task}'; expected BF or UT", file=sys.stderr)
    sys.exit(2)

from_filename    = source + '/' + locale + '/bf_locale.xml'
to_filename      = source + '/' + locale + '/bf_locale.h'
untrans_filename = source + '/untranslated.h' 
bf_header_name   = source + '/bf_header'

if verbose:
    print(sys.argv[0] + ': GENERATE ' + task + ' >' + locale + '< ' + from_filename + ' to '+ to_filename)

if task == 'BF':
    os.makedirs(os.path.dirname(to_filename), exist_ok=True)
    fw_to = open(to_filename, 'w', encoding='utf-8', newline='\n')
    wr_bf_header(fw_to, bf_header_name)
    wr_bf_notice_bf(fw_to, locale)

if task == 'UT':
    os.makedirs(os.path.dirname(untrans_filename), exist_ok=True)
    fw_ut = open(untrans_filename, 'w', encoding='utf-8', newline='\n')
    wr_bf_header(fw_ut, bf_header_name)
    wr_bf_notice_ut(fw_ut)

prevKey = ""
prevMessage = prevDescription = ""
prevError = ""

tree = ET.parse(from_filename)
root = tree.getroot()

for string_element in root.findall('string'):
    currentKey = string_element.get('name')
    maxLen = int(string_element.get('maxLength') or 0)
    description = string_element.get('comment') or ""
    message = (string_element.text or "")

    if len(message) > maxLen and maxLen > 0:
        errorMess = "ERROR - define " + currentKey + " with maxLength of " + str(maxLen) + " exceed with text: " + message + " / " + str(len(message))
        print(errorMess, file=sys.stderr)
    else:
        errorMess = ""

    if currentKey == prevKey + hdKey:
        messTR2 = 'TR2("' + prevMessage + '", "' + message + '")'
        mess    = '"' + prevMessage + '"'
        if task == 'BF':
            wr_define_bf(fw_to, prevKey, messTR2, prevDescription + "; " + description, errorMess)
        if task == 'UT':
            wr_define_ut(fw_ut, prevKey, mess, errorMess)
    else:
        if prevKey != "" and not prevKey.find(hdKey) > 0:
            mess = '"' + prevMessage + '"'
            if task == 'BF':
                wr_define_bf(fw_to, prevKey, mess, prevDescription, errorMess)
            if task == 'UT':
                wr_define_ut(fw_ut, prevKey, mess, errorMess)
    prevKey = currentKey
    prevMessage = message
    prevDescription = description
# for
mess = '"' + message + '"'
if task == 'BF':
    wr_define_bf(fw_to, currentKey, mess, description, errorMess)
if task == 'UT':
    wr_define_ut(fw_ut, currentKey, mess, errorMess)

if task == 'BF':
    fw_to.close()
if task == 'UT':
    fw_ut.close()

if verbose:
    print(sys.argv[0] + ': FINISH')
