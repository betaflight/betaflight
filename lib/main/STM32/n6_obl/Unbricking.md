# Unbricking the STM32N6 (TSV + system DFU)

Use this when OBL or BF in XSPI flash is broken and the board no longer
boots into a usable state — `bl rom` from the BF CLI, OBL DFU on a
crashed BF, and SWD-load workflows are all unavailable. The boot ROM's
built-in system DFU is always reachable, but it can't write the on-board
XSPI flash on its own; CubeProgrammer drives it through a TSV that
stages two helper binaries into RAM first.

## Why the on-chip DFU can't just write XSPI

The N6 boot ROM exposes a system DFU bootloader (`0483:DF11` with the
ST descriptor "STM32 BOOTLOADER", reached via BOOT0=DEV) but the alts
it offers are RAM- and OTP-shaped:

```
@FSBL        /0x34180400/01*512Kg     -- staged FSBL → AXISRAM2 secure
@RSSE_FW     ...
@RSSE_BLOB   ...
@RSSE_PLUGIN ...
@virtual     ...
```

There's no `@nor0` alt and no XSPI alt — the ROM has no XSPI driver. To
write nor0 you have to:

1. Upload a signed FSBL into RAM via the `@FSBL` alt. The ROM verifies
   it, copies it to AXISRAM2, and runs it.
2. The FSBL you upload is itself a DFU loader (ST's "OpenBootloader")
   that exposes `@nor0` plus an "external memory loader" alt for the
   chosen flash chip's driver.
3. Upload the matching XSPI flash driver into RAM via the `@External
   memory loader` alt; the OpenBootloader links it in and the rest of
   the DFU session can read/write nor0.
4. Now write your application binary to nor0 at the right offset.

The TSV (Target Sequence Vector) file is just a script for
`STM32_Programmer_CLI -d <FlashLayout.tsv>` that orchestrates these
four steps in order so it looks like a single flash operation from the
host's point of view.

## When to use this

- `bl rom` from BF CLI / OBL DFU `:leave` doesn't reboot the chip into
  a working state.
- BF crashes early enough that the BF↔OBL IWDG-recovery path doesn't
  trigger (e.g., a fault before OBL's iwdg_start runs, or OBL itself is
  corrupt).
- You want to nuke and re-flash both OBL and BF from a clean slate.

If the chip is reachable via SWD (BOOT1 in the position that bypasses
the FSBL on the dev kit, or BF still alive enough that OBL DFU comes
up), prefer the SWD-load or OBL-DFU paths — they don't require touching
hardware switches.

## What you need

- **STM32CubeProgrammer 2.18+ with the N6 patches installed.** The
  Linux build prior to 2.22 has bugs around the N6 external-loader
  workflow; current ST tooling works.
- **`OpenBootloader_STM32N6570-DK-trusted.stm32`** — ST's signed
  OpenBootloader image. Ships in CubeN6 at
  `lib/main/STM32N6/Projects/STM32N6570-DK/Applications/OpenBootloader/Binaries/NOR_Binary/`.
- **`MX66UW1G45G_STM32N6570-DK.bin`** — XSPI flash driver matching the
  on-board MX66UW1G45G. Same directory in CubeN6.
- The two binaries you want in nor0:
  - `obl_mx66uw1g45g_signed.stm32` — our OBL, output of
    `make -C lib/main/STM32/n6_obl signed`.
  - `betaflight_<version>_STM32N657_<config>.bin` — your BF build.

## TSV layout

Save as `Unbricking.tsv` next to `OpenBootloader_STM32N6570-DK-trusted.stm32`
and `MX66UW1G45G_STM32N6570-DK.bin`, then edit the two
`<...your-path...>` lines to point at your OBL and BF artefacts.

```tsv
#Opt	Id	Name	Type	IP	Offset	Binary
P	0x1	fsbl-openbl	Binary	none	0x0	OpenBootloader_STM32N6570-DK-trusted.stm32
P	0x3	fsbl-extfl	Binary	none	0x0	MX66UW1G45G_STM32N6570-DK.bin
P	0x2	obl-app	Binary	nor0	0x0	<path/to>/obl_mx66uw1g45g_signed.stm32
P	0x4	bf-app	Binary	nor0	0x100000	<path/to>/obj/betaflight_<version>_STM32N657_<config>.bin
```

Columns:

- `Opt`: `P` = program (write); `PE` = program + erase first.
- `Id`: ST's partition ID. The OpenBootloader recognises:
  - `0x1` → @FSBL: stage a signed FSBL image to AXISRAM2.
  - `0x2` → nor0 offset 0 (OBL slot, signed `.stm32` artefact).
  - `0x3` → @External memory loader: stage the XSPI driver to RAM.
  - `0x4` → nor0 offset 0x100000 (BF slot).
- `Name`: free-text label, shown in CubeProg output.
- `Type`: `Binary` for raw bytes; `Force not used` to skip a row.
- `IP`: target memory — `none` means "load to RAM" (used by Id 0x1
  and 0x3); `nor0` is the on-board XSPI flash.
- `Offset`: byte offset inside the target memory.
- `Binary`: path (relative to the TSV) of the file to write.

## Procedure

1. Power cycle the board.
2. Slide **BOOT0** to **DEV** (toward the edge of the board on the
   N6570-DK; the boot ROM samples this pin at reset and enters system
   DFU when high).
3. Press the reset button. `lsusb` should now show:

   ```
   Bus xxx Device xxx: ID 0483:df11 STMicroelectronics STM Device in DFU Mode
   ```
4. Run CubeProg against the TSV:

   ```bash
   ~/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/STM32_Programmer_CLI \
       -c port=USB1 -d Unbricking.tsv
   ```

   You'll see four phases scroll past — `fsbl-openbl` upload, then a
   re-enumeration as the OpenBootloader starts, then `fsbl-extfl`
   upload, then `obl-app` and `bf-app` writes to nor0.
5. Slide **BOOT0** back to **USER**.
6. Press reset. The chip boots from XSPI into our OBL → BF.

## Verifying

After the reset in step 6:

```bash
lsusb | grep 0483
# Expect:
#   Bus xxx Device xxx: ID 0483:3754 STMicroelectronics STLINK-V3
#   Bus xxx Device xxx: ID 0483:5740 STMicroelectronics Virtual COM Port
```

`/dev/ttyACM1` is the BF VCP. Send `#` + LF to escape MSP into the BF
CLI; `version` should report the build you just flashed.

## Footnotes

- `STM32_Programmer_CLI -el .stldr -w` with the `.stldr` form of the
  XSPI driver SIGSEGVs on Linux at least up to v2.22.0; the TSV path
  works around that.
- `port=USB1` selects the first USB DFU device; if you have several DFU
  devices attached, add `sn=<serial>` to disambiguate. List with
  `STM32_Programmer_CLI --list usb`.
- The TSV's relative-path resolution is done from the TSV's own
  directory, which is why ST's two helper binaries are kept alongside
  it. OBL and BF artefact paths can be absolute or relative to that
  same directory.
