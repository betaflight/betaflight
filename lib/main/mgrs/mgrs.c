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
 * Military Grid Reference System (MGRS) — forward encoder.
 *
 * Ported from proj4js/mgrs (MIT licensed, see LICENSE in this directory).
 * The math is the standard WGS84 transverse-Mercator forward — same as
 * UTM — followed by the MGRS letter substitutions for the latitude band
 * and the 100 km grid square.
 *
 * Differences from upstream (see also betaflight_readme.txt):
 *   - integer 1e-7 deg inputs at the API boundary (matches
 *     gpsLocation_t in io/gps.h);
 *   - no exceptions / allocations / errno;
 *   - fixed OSD-friendly output format with zero-padded zone and
 *     space-separated easting / northing groups.
 *
 * Numerical precision: the inner math uses `double`. The sixth-order
 * meridional-arc series and the easting / northing polynomials accumulate
 * enough rounding error in single precision to drift 1–2 m at the OSD's
 * 1 m precision step — which would round to the wrong MGRS digit. The
 * existing Plus Codes feature in osd_elements.c already promotes lat/lon
 * to `double` for OLC_Encode (see OSD_ELEMENT_TYPE_4 path), so this is
 * consistent precedent. The function is only called at OSD redraw cadence
 * (~30 Hz worst case), well within budget on every supported MCU.
 *
 * Public API contract is documented in mgrs.h.
 */

#include <math.h>
#include <stddef.h>

#include "mgrs.h"

/* ---------------------------------------------------------------------------
 * WGS84 ellipsoid + transverse-Mercator constants.
 *
 * All values are taken from the proj4js port of the NGA Geotrans library.
 * The inner math uses `double` to match upstream precision; see file
 * header for rationale.
 * ------------------------------------------------------------------------ */

#define MGRS_PI                3.14159265358979323846  /* full double-precision M_PI */
#define MGRS_ECC_SQUARED       0.00669438              /* WGS84 first eccentricity squared */
#define MGRS_SCALE_FACTOR      0.9996                  /* UTM scale factor on central meridian */
#define MGRS_SEMI_MAJOR_AXIS   6378137.0               /* WGS84 equatorial radius, metres */
#define MGRS_EASTING_OFFSET    500000.0                /* False easting, metres */
#define MGRS_NORTHING_OFFSET   10000000.0              /* False northing for southern hemisphere */
#define MGRS_UTM_ZONE_WIDTH    6.0                     /* Each UTM zone is 6 deg of longitude */
#define MGRS_HALF_ZONE_WIDTH   3.0                     /* Half of MGRS_UTM_ZONE_WIDTH */

/* ---------------------------------------------------------------------------
 * 100 km grid-square letter set tables.
 *
 * MGRS divides each UTM zone into 100 km squares labelled by two letters.
 * The letter assignment cycles through 6 sets per zone family. The columns
 * (easting axis) start at A/J/S; the rows (northing axis) start at A/F/A/F.
 * Letters 'I' and 'O' are excluded everywhere, and rows additionally exclude
 * letters past 'V'. See the NGA spec, table 1.
 * ------------------------------------------------------------------------ */

#define MGRS_NUM_100K_SETS     6
static const char SET_ORIGIN_COLUMN_LETTERS[MGRS_NUM_100K_SETS + 1] = "AJSAJS";
static const char SET_ORIGIN_ROW_LETTERS[MGRS_NUM_100K_SETS + 1]    = "AFAFAF";

/* Letters used in the column / row rollover logic. */
#define LET_A 'A'
#define LET_I 'I'
#define LET_O 'O'
#define LET_V 'V'
#define LET_Z 'Z'

/* Out-of-range placeholder string written when the polar guard fires. */
#define MGRS_PLACEHOLDER "--MGRS--"

/* ---------------------------------------------------------------------------
 * UTM result struct — internal only. The external API hides the UTM step.
 * ------------------------------------------------------------------------ */

typedef struct mgrsUtm_s {
    int    zone;       /* UTM zone number 1..60 */
    char   band;       /* MGRS latitude band letter, eg 'U' for Vienna */
    double easting;    /* metres east of zone false-origin */
    double northing;   /* metres north of equator (or +1e7 for southern hemi) */
} mgrsUtm_t;

/* ---------------------------------------------------------------------------
 * Helpers
 * ------------------------------------------------------------------------ */

static double mgrsDegToRad(double deg)
{
    return deg * (MGRS_PI / 180.0);
}

/*
 * Returns the MGRS latitude-band letter ('C'..'X', skipping 'I' and 'O')
 * for the given decimal-degree latitude. Returns 'Z' if the latitude is
 * outside the MGRS coverage band [-80, 84].
 *
 * Band 'X' is special-cased because it is 12 deg tall (72..84) instead of
 * the standard 8 deg. All other bands are 8 deg starting at -80.
 */
static char mgrsGetLatBandLetter(double lat_deg)
{
    if (lat_deg <= 84.0 && lat_deg >= 72.0) {
        return 'X';
    }
    if (lat_deg < 72.0 && lat_deg >= -80.0) {
        static const char band_letters[] = "CDEFGHJKLMNPQRSTUVWX";
        const int idx = (int)floor((lat_deg - (-80.0)) / 8.0);
        return band_letters[idx];
    }
    return 'Z';
}

/*
 * MGRS letter sets cycle through 6 patterns per zone, indexed by
 * (zone mod 6). Zones whose mod is 0 use set 6, not set 0.
 */
static int mgrsGet100kSetForZone(int zone)
{
    int set = zone % MGRS_NUM_100K_SETS;
    if (set == 0) {
        set = MGRS_NUM_100K_SETS;
    }
    return set;
}

/*
 * Produce the two-letter 100 km grid-square ID from a column index, row
 * index, and the zone's letter set.
 *
 * The MGRS spec excludes letters 'I' and 'O' from both axes, and rows
 * additionally wrap after 'V'. The control flow below is a verbatim port
 * of proj4js's getLetter100kID (in turn from NGA Geotrans) — small and
 * subtle, but well-tested for decades. Touch with extreme care.
 */
static void mgrsGetLetter100kID(int column, int row, int set, char *col_out, char *row_out)
{
    const int idx = set - 1;
    const int colOrigin = SET_ORIGIN_COLUMN_LETTERS[idx];
    const int rowOrigin = SET_ORIGIN_ROW_LETTERS[idx];

    int colInt = colOrigin + column - 1;
    int rowInt = rowOrigin + row;
    int rollover = 0;

    if (colInt > LET_Z) {
        colInt = colInt - LET_Z + LET_A - 1;
        rollover = 1;
    }
    if (colInt == LET_I
        || (colOrigin < LET_I && colInt > LET_I)
        || ((colInt > LET_I || colOrigin < LET_I) && rollover)) {
        colInt++;
    }
    if (colInt == LET_O
        || (colOrigin < LET_O && colInt > LET_O)
        || ((colInt > LET_O || colOrigin < LET_O) && rollover)) {
        colInt++;
        if (colInt == LET_I) {
            colInt++;
        }
    }
    if (colInt > LET_Z) {
        colInt = colInt - LET_Z + LET_A - 1;
    }

    rollover = 0;
    if (rowInt > LET_V) {
        rowInt = rowInt - LET_V + LET_A - 1;
        rollover = 1;
    }
    if (rowInt == LET_I
        || (rowOrigin < LET_I && rowInt > LET_I)
        || ((rowInt > LET_I || rowOrigin < LET_I) && rollover)) {
        rowInt++;
    }
    if (rowInt == LET_O
        || (rowOrigin < LET_O && rowInt > LET_O)
        || ((rowInt > LET_O || rowOrigin < LET_O) && rollover)) {
        rowInt++;
        if (rowInt == LET_I) {
            rowInt++;
        }
    }
    if (rowInt > LET_V) {
        rowInt = rowInt - LET_V + LET_A - 1;
    }

    *col_out = (char)colInt;
    *row_out = (char)rowInt;
}

/*
 * WGS84 latitude/longitude -> UTM forward conversion (sixth-order series).
 *
 * Includes the Norwegian (zone 32V) and Svalbard (zones 31X/33X/35X/37X)
 * special-case zone re-assignments. Polar regions (above 84N or below 80S)
 * are NOT handled here; mgrsEncode applies that guard before calling.
 */
static void mgrsLLtoUTM(double lat_deg, double lon_deg, mgrsUtm_t *out)
{
    /* Default zone from longitude. Zone 1 is -180..-174, zone 60 is 174..180. */
    int zone = (int)floor((lon_deg + 180.0) / MGRS_UTM_ZONE_WIDTH) + 1;
    if (lon_deg == 180.0) {
        zone = 60;
    }

    /* Norwegian zone exception: south-west Norway extends zone 32V west. */
    if (lat_deg >= 56.0 && lat_deg < 64.0 && lon_deg >= 3.0 && lon_deg < 12.0) {
        zone = 32;
    }

    /* Svalbard zone exceptions: zones 31X / 33X / 35X / 37X cover wider strips. */
    if (lat_deg >= 72.0 && lat_deg < 84.0) {
        if (lon_deg >= 0.0 && lon_deg < 9.0) {
            zone = 31;
        } else if (lon_deg >= 9.0 && lon_deg < 21.0) {
            zone = 33;
        } else if (lon_deg >= 21.0 && lon_deg < 33.0) {
            zone = 35;
        } else if (lon_deg >= 33.0 && lon_deg < 42.0) {
            zone = 37;
        }
    }

    const double lat_rad = mgrsDegToRad(lat_deg);
    const double lon_rad = mgrsDegToRad(lon_deg);
    const double lon_origin_rad =
        mgrsDegToRad((zone - 1) * MGRS_UTM_ZONE_WIDTH - 180.0 + MGRS_HALF_ZONE_WIDTH);

    const double ecc2 = MGRS_ECC_SQUARED;
    const double ecc4 = ecc2 * ecc2;
    const double ecc6 = ecc4 * ecc2;
    const double eccPrimeSquared = ecc2 / (1.0 - ecc2);

    const double sinLat = sin(lat_rad);
    const double cosLat = cos(lat_rad);
    const double tanLat = tan(lat_rad);

    const double N = MGRS_SEMI_MAJOR_AXIS / sqrt(1.0 - ecc2 * sinLat * sinLat);
    const double T = tanLat * tanLat;
    const double C = eccPrimeSquared * cosLat * cosLat;
    const double A = cosLat * (lon_rad - lon_origin_rad);

    /* Meridional arc M (sixth-order series). */
    const double M = MGRS_SEMI_MAJOR_AXIS * (
          (1.0 - ecc2 / 4.0 - 3.0 * ecc4 / 64.0 - 5.0 * ecc6 / 256.0) * lat_rad
        - (3.0 * ecc2 / 8.0 + 3.0 * ecc4 / 32.0 + 45.0 * ecc6 / 1024.0) * sin(2.0 * lat_rad)
        + (15.0 * ecc4 / 256.0 + 45.0 * ecc6 / 1024.0) * sin(4.0 * lat_rad)
        - (35.0 * ecc6 / 3072.0) * sin(6.0 * lat_rad)
    );

    const double A2 = A * A;
    const double A3 = A2 * A;
    const double A4 = A3 * A;
    const double A5 = A4 * A;
    const double A6 = A5 * A;

    const double utmEasting = MGRS_SCALE_FACTOR * N * (
          A
        + (1.0 - T + C) * A3 / 6.0
        + (5.0 - 18.0 * T + T * T + 72.0 * C - 58.0 * eccPrimeSquared) * A5 / 120.0
    ) + MGRS_EASTING_OFFSET;

    double utmNorthing = MGRS_SCALE_FACTOR * (
        M + N * tanLat * (
              A2 / 2.0
            + (5.0 - T + 9.0 * C + 4.0 * C * C) * A4 / 24.0
            + (61.0 - 58.0 * T + T * T + 600.0 * C - 330.0 * eccPrimeSquared) * A6 / 720.0
        )
    );
    if (lat_deg < 0.0) {
        utmNorthing += MGRS_NORTHING_OFFSET;
    }

    out->zone     = zone;
    out->band     = mgrsGetLatBandLetter(lat_deg);
    out->easting  = utmEasting;
    out->northing = utmNorthing;
}

/* ---------------------------------------------------------------------------
 * Output formatting
 *
 * Writes:    "NN Z SS [DDDDD DDDDD]"   for precision >= 1
 * or:        "NN Z SS"                  for precision == 0
 *
 * where N = decimal digit, Z = band letter, S = 100k-square letter, and
 * D = easting/northing digit. The number of D's per group equals
 * `precision`. Zone is always two digits (zero-padded) so the OSD column
 * width is fixed.
 * ------------------------------------------------------------------------ */

/* Write n decimal digits of `value` (with leading zeros) starting at `out`. */
static void mgrsWriteDigits(char *out, unsigned int value, unsigned int n)
{
    out += n;
    for (unsigned int i = 0; i < n; i++) {
        *--out = (char)('0' + (value % 10));
        value /= 10;
    }
}

/* Truncate `value` (a positive easting or northing in metres) to its
 * lowest 5 decimal digits (mod 100000), then drop the lowest (5-precision)
 * digits to get the MGRS digit group at that precision.  */
static unsigned int mgrsTruncateAxis(double value, unsigned int precision)
{
    unsigned int truncated = (unsigned int)floor(value) % 100000U;
    for (unsigned int i = 0; i < (5U - precision); i++) {
        truncated /= 10U;
    }
    return truncated;
}

/* ---------------------------------------------------------------------------
 * Public API
 * ------------------------------------------------------------------------ */

bool mgrsEncode(int32_t lat_e7, int32_t lon_e7, uint8_t precision, char *out)
{
    if (out == NULL) {
        return false;
    }

    if (precision > MGRS_PRECISION_MAX) {
        precision = MGRS_PRECISION_MAX;
    }

    const double lat_deg = (double)lat_e7 * 1.0e-7;
    const double lon_deg = (double)lon_e7 * 1.0e-7;

    /* Polar guard: latitudes outside [-80, 84] are MGRS UPS regions. We do
     * not implement UPS — a flight controller in those regions has bigger
     * problems. Write a placeholder and signal failure. */
    if (lat_deg < -80.0 || lat_deg > 84.0) {
        const char *p = MGRS_PLACEHOLDER;
        unsigned int i = 0;
        while (p[i] != '\0' && i < (MGRS_BUFFER_SIZE - 1U)) {
            out[i] = p[i];
            i++;
        }
        out[i] = '\0';
        return false;
    }

    mgrsUtm_t utm;
    mgrsLLtoUTM(lat_deg, lon_deg, &utm);

    const int set = mgrsGet100kSetForZone(utm.zone);
    const int eastCol = (int)floor(utm.easting / 100000.0);
    const int northRow = ((int)floor(utm.northing / 100000.0)) % 20;

    char col100k = '?';
    char row100k = '?';
    mgrsGetLetter100kID(eastCol, northRow, set, &col100k, &row100k);

    /* "NN Z SS" — zone (zero-padded two digits), band letter, 100k pair. */
    int p = 0;
    out[p++] = (char)('0' + (utm.zone / 10));
    out[p++] = (char)('0' + (utm.zone % 10));
    out[p++] = utm.band;
    out[p++] = col100k;
    out[p++] = row100k;

    if (precision > 0) {
        const unsigned int eastDigits  = mgrsTruncateAxis(utm.easting,  precision);
        const unsigned int northDigits = mgrsTruncateAxis(utm.northing, precision);

        out[p++] = ' ';
        mgrsWriteDigits(&out[p], eastDigits, precision);
        p += precision;

        out[p++] = ' ';
        mgrsWriteDigits(&out[p], northDigits, precision);
        p += precision;
    }
    out[p] = '\0';
    return true;
}
