#!/usr/bin/env bash
# Start an OpenOCD SWD gdb-server for STM32H5 / STM32C5 (Cortex-M33) targets.
#
# OpenOCD 0.12 (shipped in this container) has no stm32h5x.cfg / stm32c5x.cfg,
# and on these parts the CPU core debug sits on AP1 (the APB-AP), not AP0. So we
# reuse the DAP that target/stm32h7x.cfg creates and attach a cortex_m target on
# AP1. (Using AP0 by mistake shows up as: DPIDR/DBGMCU read fine but
# "Failed to read memory at 0xe000ed00".)
#
# We defer-examine the targets stm32h7x.cfg brings up (they are H7-specific and
# would fail to examine on an H5/C5), so only h5core is examined at init, and we
# pin its gdb port explicitly.
#
# The gdb-server listens on :3334. Connect with the pre-installed gdb-multiarch:
#   gdb-multiarch obj/main/betaflight_<TARGET>.elf \
#       -ex 'target extended-remote localhost:3334' -ex 'monitor halt' -ex bt
#
# Requires an ST-Link the container can open (see the udev rule in README.md).
set -euo pipefail

exec openocd \
    -f interface/stlink-dap.cfg \
    -c "transport select dapdirect_swd" \
    -c "adapter speed 2000" \
    -f target/stm32h7x.cfg \
    -c "foreach t [target names] { \$t configure -defer-examine }" \
    -c "target create h5core cortex_m -dap stm32h7x.dap -ap-num 1 -gdb-port 3334" \
    -c "init" \
    -c "targets h5core"
