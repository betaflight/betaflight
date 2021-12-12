#!/usr/bin/env python3

import sys
import re
import csv

if len(sys.argv) != 3:
    sys.exit('Must provide two size report file arguments')

def read_size_report(file):
    targets = {}
    with open(file, newline='') as file:
        reader = csv.DictReader(file, delimiter=' ', skipinitialspace=True)
        for row in reader:
            targetName = re.findall('_([^/]+).elf', row['filename'])[0]
            targets[targetName] = target = {}
            for key in ['text', 'data', 'bss']:
                target[key] = int(row[key])

    return targets

reports = [read_size_report(sys.argv[1]), read_size_report(sys.argv[2])]

def print_size_comparison(target):
    print('| Region | Before | After | Difference |')
    print('| ------ | ------ | ----- | ---------- |')
    regionsA = reports[0][target]
    regionsB = reports[1][target]
    for region in regionsA:
        if region not in regionsB:
            continue

        sizeA = regionsA[region]
        sizeB = regionsB[region]
        diff = sizeB - sizeA

        items = [region, sizeA, sizeB, signed(diff)]
        items = [str(item) for item in items]

        print('|', ' | '.join(items), '|')

def total_size_difference(target):
    return sum(reports[1][target].values()) - sum(reports[0][target].values())

def signed(int):
    return f'{int:+d}' if int != 0 else 0

def b(text):
    return '**' + str(text) + '**'

short_target = 'STM32F411'

if short_target in reports[0] and short_target in reports[1]:
    diff = total_size_difference(short_target)
    print('Size for target', b(short_target), 'changed by:', b(signed(diff)), 'bytes')
    print()

print('<details>')
print('<summary>Show all targets</summary>')
print()

for target in reports[0]:
    if target not in reports[1]:
        continue

    diff = total_size_difference(target)
    print(b(target), 'changed by', b(signed(diff)), 'bytes')
    print()

    if diff != 0:
        print_size_comparison(target)
        print()

print('</details>')
