/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

// Provides: cliPrintDebug... functions for displaying debugging information in the CLI
//
// Usage: Make sure USE_CLI_DEBUG_PRINT is defined
//        Include this header in your code
//        Add cliDebugPrint... statements as needed in your code
//        Use the CLI to see the output of the debugging statements
//
// Cautions: Be sure to include rate limiting logic to your debug printing
//           if needed otherwise you can flood the output.
//
//           Be sure to reverse the Usage steps above to remove the debugging
//           elements before submitting final code.

#include "platform.h"

#ifdef USE_CLI_DEBUG_PRINT

#include "cli/cli.h"

// Commands to print debugging information to the CLI
#define cliDebugPrintLinefeed cliPrintLinefeed
#define cliDebugPrintLinef cliPrintLinef
#define cliDebugPrintLine cliPrintLine
#define cliDebugPrintf cliPrintf
#define cliDebugPrint cliPrint

#else
#error "Do not #include cli_debug_print.h unless you intend to do debugging and also define USE_CLI_DEBUG_PRINT"
#endif
