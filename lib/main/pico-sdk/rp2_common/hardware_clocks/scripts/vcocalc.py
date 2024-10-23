#!/usr/bin/env python3

import argparse
import sys

# Fixed hardware parameters
fbdiv_range = range(16, 320 + 1)
postdiv_range = range(1, 7 + 1)
ref_min = 5
refdiv_min = 1
refdiv_max = 63

def validRefdiv(string):
    if ((int(string) < refdiv_min) or (int(string) > refdiv_max)):
        raise ValueError("REFDIV must be in the range {} to {}".format(refdiv_min, refdiv_max))
    return int(string)

parser = argparse.ArgumentParser(description="PLL parameter calculator")
parser.add_argument("--input", "-i", default=12, help="Input (reference) frequency. Default 12 MHz", type=float)
parser.add_argument("--ref-min", default=5, help="Override minimum reference frequency. Default 5 MHz", type=float)
parser.add_argument("--vco-max", default=1600, help="Override maximum VCO frequency. Default 1600 MHz", type=float)
parser.add_argument("--vco-min", default=750, help="Override minimum VCO frequency. Default 750 MHz", type=float)
parser.add_argument("--cmake", action="store_true", help="Print out a CMake snippet to apply the selected PLL parameters to your program")
parser.add_argument("--cmake-only", action="store_true", help="Same as --cmake, but do not print anything other than the CMake output")
parser.add_argument("--cmake-executable-name", default="<program>", help="Set the executable name to use in the generated CMake output")
parser.add_argument("--lock-refdiv", help="Lock REFDIV to specified number in the range {} to {}".format(refdiv_min, refdiv_max), type=validRefdiv)
parser.add_argument("--low-vco", "-l", action="store_true", help="Use a lower VCO frequency when possible. This reduces power consumption, at the cost of increased jitter")
parser.add_argument("output", help="Output frequency in MHz.", type=float)
args = parser.parse_args()

refdiv_range = range(refdiv_min, max(refdiv_min, min(refdiv_max, int(args.input / args.ref_min))) + 1)
if args.lock_refdiv:
	print("Locking REFDIV to", args.lock_refdiv)
	refdiv_range = [args.lock_refdiv]

best = (0, 0, 0, 0, 0, 0)
best_margin = args.output

for refdiv in refdiv_range:
	for fbdiv in fbdiv_range:
		vco = args.input / refdiv * fbdiv
		if vco < args.vco_min or vco > args.vco_max:
			continue
		# pd1 is inner loop so that we prefer higher ratios of pd1:pd2
		for pd2 in postdiv_range:
			for pd1 in postdiv_range:
				out = vco / pd1 / pd2
				margin = abs(out - args.output)
				vco_is_better = vco < best[5] if args.low_vco else vco > best[5]
				if ((vco * 1000) % (pd1 * pd2)):
					continue
				if margin < best_margin or (abs(margin - best_margin) < 1e-9 and vco_is_better):
					best = (out, fbdiv, pd1, pd2, refdiv, vco)
					best_margin = margin

best_out, best_fbdiv, best_pd1, best_pd2, best_refdiv, best_vco = best

if best[0] > 0:
	cmake_output = \
f"""target_compile_definitions({args.cmake_executable_name} PRIVATE
	PLL_SYS_REFDIV={best_refdiv}
	PLL_SYS_VCO_FREQ_HZ={int((args.input * 1_000_000) / best_refdiv * best_fbdiv)}
	PLL_SYS_POSTDIV1={best_pd1}
	PLL_SYS_POSTDIV2={best_pd2}
)
"""
	if not args.cmake_only:
		print("Requested: {} MHz".format(args.output))
		print("Achieved:  {} MHz".format(best_out))
		print("REFDIV:    {}".format(best_refdiv))
		print("FBDIV:     {} (VCO = {} MHz)".format(best_fbdiv, args.input / best_refdiv * best_fbdiv))
		print("PD1:       {}".format(best_pd1))
		print("PD2:       {}".format(best_pd2))
		if best_refdiv != 1:
			print(
				"\nThis requires a non-default REFDIV value.\n"
				"Add the following to your CMakeLists.txt to apply the REFDIV:\n"
			)
		elif args.cmake or args.cmake_only:
			print("")
	if args.cmake or args.cmake_only or best_refdiv != 1:
		print(cmake_output)
else:
	sys.exit("No solution found")
