# Minimal ESP-IDF project used only to build the second-stage bootloader from
# source (`idf.py bootloader`). The Betaflight Makefile invokes this; the main
# app is the real Betaflight build, not this stub.
cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(bf_bootloader)
