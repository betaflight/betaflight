Military Grid Reference System (MGRS)

Source from: https://github.com/proj4js/mgrs (commit reference: master, 2025-01-05 release v2.1.0)
License:     MIT (see LICENSE)

This is a port — not a 1:1 copy — of the proj4js/mgrs forward conversion path
(latitude/longitude -> MGRS string). It is restructured for an embedded C
flight-controller environment:

  - Forward-only: only LL -> MGRS is implemented. The inverse path
    (MGRS -> lat/lon, bounding boxes, decode) is NOT included. A flight
    controller emits MGRS for OSD/telemetry; it never parses one.
  - Single precision floats throughout. Cortex-M FPUs are single-precision;
    using doubles trips the soft-float path and is too slow for our budget.
  - No exceptions / no allocations / no errno. The single failure mode
    (latitude outside the [-80, 84] MGRS band) returns false and writes a
    fixed placeholder string to the caller's buffer.
  - Inputs use int32 latitude/longitude scaled by 1e7 to match
    gpsLocation_t in src/main/io/gps.h (avoids a degree-string round-trip).
  - Output is a single readable string of the form "NN Z SS NNNNN NNNNN"
    (space-separated grid-zone, 100k square, easting, northing) sized for
    direct OSD use. Zones 1-9 are zero-padded so the string width is fixed
    at 17 chars + NUL — useful for OSD column layout.

The math is unchanged from upstream: WGS84 ellipsoid, transverse Mercator
forward, 6th-order series. Norwegian (zone 32V) and Svalbard (31X/33X/35X/37X)
exceptions are preserved. The 100k-square letter set logic (with I/O
exclusion and the row/column rollover dance) is preserved verbatim.

Test vectors in src/test/unit/mgrs_unittest.cc are taken directly from the
upstream proj4js/mgrs test suite (test/test.js) so this port can be
verified against the same gold-standard outputs as the source library.
