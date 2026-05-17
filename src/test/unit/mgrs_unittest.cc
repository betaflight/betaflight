/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software. If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * Unit tests for the MGRS forward encoder (lib/main/mgrs/).
 *
 * Test vectors are derived directly from the upstream proj4js/mgrs test
 * suite (test/test.js). Each fixture below cites the matching proj4js
 * expected string so the port stays verifiable against the same
 * gold-standard outputs as the source library.
 *
 * The proj4js outputs are bare strings like "33UXP0500444997". Our port
 * emits the OSD-friendly form "33UXP 05004 44997" (zero-padded 2-digit
 * zone + space + 5-digit easting + space + 5-digit northing) — the
 * fixture strings below are the proj4js outputs after that fixed
 * transformation.
 *
 * Latitude / longitude are passed in as int32 scaled by 1e7 to match
 * gpsLocation_t in src/main/io/gps.h.
 */

#include <stdint.h>
#include <string.h>

extern "C" {
    #include "mgrs.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

// --- Helpers -----------------------------------------------------------------

// Convert a decimal-degree literal to the int32 1e-7 scale used by
// gpsLocation_t. Wrapped in a function (not a macro) so floating-point
// constants in the test file are evaluated at compile time.
static int32_t deg_to_e7(double deg)
{
    // round-half-away-from-zero to match how a GPS receiver reports.
    return (int32_t)(deg * 1.0e7 + (deg >= 0 ? 0.5 : -0.5));
}

// --- Mid-zone fixtures (proj4js test.js: "First MGRS set" — Vienna) ----------

// proj4js: mgrs.toPoint('33UXP04')  -> [16.41450, 48.24949]
// proj4js: mgrs.forward([16.41450, 48.24949])     -> '33UXP0500444997'
// proj4js: mgrs.forward([16.41450, 48.24949], 1)  -> '33UXP04'
// proj4js: mgrs.forward([16.41450, 48.24949], 0)  -> '33UXP'

TEST(MgrsForward, ViennaPrecision5_OneMeter)
{
    char buf[MGRS_BUFFER_SIZE] = {0};
    const bool ok = mgrsEncode(deg_to_e7(48.24949), deg_to_e7(16.41450), 5, buf);
    EXPECT_TRUE(ok);
    EXPECT_STREQ("33UXP 05004 44997", buf);
}

TEST(MgrsForward, ViennaPrecision1_TenKilometers)
{
    char buf[MGRS_BUFFER_SIZE] = {0};
    const bool ok = mgrsEncode(deg_to_e7(48.24949), deg_to_e7(16.41450), 1, buf);
    EXPECT_TRUE(ok);
    EXPECT_STREQ("33UXP 0 4", buf);
}

TEST(MgrsForward, ViennaPrecision0_GridSquareOnly)
{
    char buf[MGRS_BUFFER_SIZE] = {0};
    const bool ok = mgrsEncode(deg_to_e7(48.24949), deg_to_e7(16.41450), 0, buf);
    EXPECT_TRUE(ok);
    EXPECT_STREQ("33UXP", buf);
}

// --- Equator / Prime Meridian (proj4js test.js: "Second MGRS set") -----------

// proj4js: mgrs.forward([0, 0], 5)        -> '31NAA6602100000'
// proj4js: mgrs.forward([0, 0.00001], 5)  -> '31NAA6602100001'

TEST(MgrsForward, EquatorPrimeMeridian_Precision5)
{
    char buf[MGRS_BUFFER_SIZE] = {0};
    const bool ok = mgrsEncode(0, 0, 5, buf);
    EXPECT_TRUE(ok);
    EXPECT_STREQ("31NAA 66021 00000", buf);
}

TEST(MgrsForward, JustNorthOfEquator_Precision5)
{
    char buf[MGRS_BUFFER_SIZE] = {0};
    const bool ok = mgrsEncode(deg_to_e7(0.00001), 0, 5, buf);
    EXPECT_TRUE(ok);
    EXPECT_STREQ("31NAA 66021 00001", buf);
}

// --- Western hemisphere mid-zone (proj4js test.js: "third mgrs set" — Las Vegas) ---

// proj4js: mgrs.forward([-115.0820944, 36.2361322])     -> '11SPA7234911844'
// proj4js: mgrs.forward([-115.0820944, 36.2361322], 0)  -> '11SPA'

TEST(MgrsForward, LasVegasPrecision5_OneMeter)
{
    char buf[MGRS_BUFFER_SIZE] = {0};
    const bool ok = mgrsEncode(deg_to_e7(36.2361322), deg_to_e7(-115.0820944), 5, buf);
    EXPECT_TRUE(ok);
    EXPECT_STREQ("11SPA 72349 11844", buf);
}

TEST(MgrsForward, LasVegasPrecision0_GridSquareOnly)
{
    char buf[MGRS_BUFFER_SIZE] = {0};
    const bool ok = mgrsEncode(deg_to_e7(36.2361322), deg_to_e7(-115.0820944), 0, buf);
    EXPECT_TRUE(ok);
    EXPECT_STREQ("11SPA", buf);
}

// --- Norwegian zone-32V exception ------------------------------------------
// MGRS extends zone 32V westward to cover south-west Norway (lat 56..64, lon 3..12).
// Without the exception, longitudes 3..6 would land in zone 31; the exception
// reassigns the strip to 32. The reference strings below come from running
// proj4js mgrs.forward() (the upstream library we ported) on the same inputs.

TEST(MgrsForward, NorwayException_Lat60Lon5_IsZone32V)
{
    // proj4js: mgrs.forward([5, 60]) -> '32VKM7697958157'
    char buf[MGRS_BUFFER_SIZE] = {0};
    const bool ok = mgrsEncode(deg_to_e7(60.0), deg_to_e7(5.0), 5, buf);
    EXPECT_TRUE(ok);
    EXPECT_STREQ("32VKM 76979 58157", buf);
}

TEST(MgrsForward, NorwayException_Lat63Lon11_IsZone32V)
{
    // Cross-checks the upper-right corner of the Norway exception box.
    // proj4js: mgrs.forward([11, 63]) -> '32VPQ0129387164'
    char buf[MGRS_BUFFER_SIZE] = {0};
    const bool ok = mgrsEncode(deg_to_e7(63.0), deg_to_e7(11.0), 5, buf);
    EXPECT_TRUE(ok);
    EXPECT_STREQ("32VPQ 01293 87164", buf);
}

TEST(MgrsForward, NorwayException_JustOutside_FallsToDefaultZone)
{
    // Sanity check: lat 55 is below the Norway exception band (which starts
    // at 56), so longitude 5 stays in the default zone 31U. If we accidentally
    // widened the exception, this test catches it.
    // proj4js: mgrs.forward([5, 55]) -> '31UFA2792896620'
    char buf[MGRS_BUFFER_SIZE] = {0};
    const bool ok = mgrsEncode(deg_to_e7(55.0), deg_to_e7(5.0), 5, buf);
    EXPECT_TRUE(ok);
    EXPECT_STREQ("31UFA 27928 96620", buf);
}

// --- Svalbard zone exceptions ----------------------------------------------
// Above 72 deg N, MGRS uses zones 31X/33X/35X/37X (each twice the normal width)
// instead of the default 6-degree zones. These tests pin each special-case
// branch separately so a regression in one cannot hide behind another.

TEST(MgrsForward, SvalbardException_Lat78Lon8_IsZone31X)
{
    // Default zone for lon=8 would be 32; exception forces 31.
    // proj4js: mgrs.forward([8, 78]) -> '31XFG1591463320'
    char buf[MGRS_BUFFER_SIZE] = {0};
    const bool ok = mgrsEncode(deg_to_e7(78.0), deg_to_e7(8.0), 5, buf);
    EXPECT_TRUE(ok);
    EXPECT_STREQ("31XFG 15914 63320", buf);
}

TEST(MgrsForward, SvalbardException_Lat78Lon10_IsZone33X)
{
    // Default zone for lon=10 would be 32; exception forces 33.
    // proj4js: mgrs.forward([10, 78]) -> '33XUG8408563320'
    char buf[MGRS_BUFFER_SIZE] = {0};
    const bool ok = mgrsEncode(deg_to_e7(78.0), deg_to_e7(10.0), 5, buf);
    EXPECT_TRUE(ok);
    EXPECT_STREQ("33XUG 84085 63320", buf);
}

TEST(MgrsForward, SvalbardException_Lat78Lon35_IsZone37X)
{
    // Default zone for lon=35 would be 36; exception forces 37.
    // proj4js: mgrs.forward([35, 78]) -> '37XDG0722961538'
    char buf[MGRS_BUFFER_SIZE] = {0};
    const bool ok = mgrsEncode(deg_to_e7(78.0), deg_to_e7(35.0), 5, buf);
    EXPECT_TRUE(ok);
    EXPECT_STREQ("37XDG 07229 61538", buf);
}

// --- Polar bounds (proj4js test.js: "forward throws an error when latitude is near pole") ---

// proj4js raises for latitudes outside [-80, 84]. Our port returns false
// and writes a placeholder, since flight controllers can't throw.

TEST(MgrsForward, NorthOfCoverage_ReturnsFalse)
{
    char buf[MGRS_BUFFER_SIZE] = {0};
    const bool ok = mgrsEncode(deg_to_e7(88.0), deg_to_e7(45.0), 5, buf);
    EXPECT_FALSE(ok);
}

TEST(MgrsForward, SouthOfCoverage_ReturnsFalse)
{
    char buf[MGRS_BUFFER_SIZE] = {0};
    const bool ok = mgrsEncode(deg_to_e7(-88.0), deg_to_e7(45.0), 5, buf);
    EXPECT_FALSE(ok);
}

// --- Buffer / size sanity ----------------------------------------------------

// At MGRS_PRECISION_MAX (5), the longest output is 17 chars + NUL =
// MGRS_BUFFER_SIZE bytes. Make sure the encoder doesn't write past that.
// (We trust EXPECT_STREQ above to verify the *content*; this test pins the
// invariant that the buffer-size constant is correct for what the encoder
// produces.)

TEST(MgrsForward, OutputFitsInDeclaredBufferSize)
{
    char buf[MGRS_BUFFER_SIZE] = {0};
    const bool ok = mgrsEncode(deg_to_e7(48.24949), deg_to_e7(16.41450), MGRS_PRECISION_MAX, buf);
    EXPECT_TRUE(ok);
    // Strlen including NUL must not exceed our published buffer-size constant.
    EXPECT_LE(strlen(buf) + 1, (size_t)MGRS_BUFFER_SIZE);
}
