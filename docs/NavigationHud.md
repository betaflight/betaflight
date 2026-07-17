# Navigation HUD: OSD Minimap and GPS Rescue Guidance

Feature flag: `USE_OSD_NAV_HUD` (auto-enabled on any target with `USE_GPS` and `USE_OSD`).

This document is the design record for the navigation HUD: feasibility analysis,
architecture, rendering strategy, UI mockups, configuration, testing, performance,
the Configurator work required, and known limitations.

---

## 1. What it is

A continuously available top-down minimap OSD element plus an optional flight-director
element. During normal GPS flight the map shows home, the aircraft, the flown
breadcrumb trail and the direct line home. When GPS Rescue engages, the map can
automatically expand one size and switches to a rescue presentation: the planned
return route, the descent-start marker, the live autopilot target, the actual rescue
track, the rescue phase, ETA, target altitude/speed, cross-track error and GPS health.

Everything renders with glyphs already present in the standard Betaflight font, so
MSP DisplayPort systems (DJI O3/O4, HDZero, Walksnail) and MAX7456 analog OSDs work
without font changes.

## 2. Feasibility and rendering constraints (verified against this tree)

* **There is no pixel/vector canvas for the relevant displays.** The canvas API in
  `drivers/display_canvas.h` is implemented only by the FrSky OSD displayport.
  MSP DisplayPort (`io/displayport_msp.c`) exposes `writeString`/`writeChar`
  (max 30 characters per write, wire format `row, col, attr, chars`) and MAX7456 is a
  character generator. The HUD is therefore a **character-grid renderer**.
* **Multi-cell elements are supported by the OSD scheduler.** An element draw
  function that sets `element->rendered = false` is called again on the next
  scheduler pass (see `osdElementArtificialHorizon`, `osdBackgroundStickOverlay`).
  The minimap emits **one row-string per pass**, so a 19x9 map plus footers costs
  ~12 passes per OSD frame — the same order as the artificial horizon's 9 passes.
* **Fonts:** the FC sends codepoints only; MSP goggles use their locally installed
  Betaflight font. The renderer restricts itself to: the 16 direction arrows
  (`0x60-0x6F`), `SYM_HOMEFLAG`, `SYM_OVER_HOME`, the stick-overlay box/sprite glyphs
  (`0x08-0x0B`, `0x16-0x17`), `SYM_ARROW_SMALL_UP/DOWN`, `SYM_SAT_L/R`, `SYM_FLY_M`
  and plain ASCII digits/letters/punctuation. All of these exist in every standard
  Betaflight-compatible font.
* **What GPS Rescue actually plans** (`flight/gps_rescue_multirotor.c`): climb to the
  return altitude, rotate the nose to home, then slide the autopilot position target
  along the **straight line from the craft to home** at the configured ground speed,
  begin descending when inside `descentDistanceM`, then land. There is no
  multi-segment route. The HUD therefore draws exactly that plan: the engage-point →
  home line, a descent-start marker on it, and the live autopilot target ("carrot"),
  rather than any invented path.

## 3. Architecture

```
io/gps.c ── onGpsNewData() ──► flight/nav_hud.c        (core: no display knowledge)
                                 ├─ breadcrumb trail (decimation, glitch filter)
                                 ├─ rescue route capture (engage point → home)
                                 └─ navHudState_t snapshot (20 Hz cap)
                                          │
osd/osd_elements.c ─ dispatch ──► osd/osd_nav_hud.c    (renderer: no navigation logic)
                                 ├─ projection (orientation/center/zoom)
                                 ├─ rasteriser (grid buffer, row emission)
                                 └─ footers / flight director
pg/nav_hud.c                     configuration (PG 567, nav_hud_* CLI settings)
```

* `flight/nav_hud.c` — all navigation calculation. Consumes `gpsSol`,
  `GPS_home_llh`, `GPS_distance2d()` (the meridian-safe lat/lon → local ENU cm
  converter), `attitude`, `imuIsHeadingValid()`, `getAltitudeCm()`, and the GPS
  Rescue/autopilot getters added for this feature. Produces `navHudState_t`.
* `osd/osd_nav_hud.c` — pure rasterisation of the prepared state. Heavy work
  (projection of ≤120 trail points, two clipped Bresenham lines) happens **once per
  OSD frame** into a static grid buffer; each scheduler pass then just copies one row.
* Getters added (no behaviour change): `gpsRescueGetPhase/Failure/TargetAltitudeCm/
  ReturnAltitudeCm/TargetVelocityCmS/DescentDistanceCm`, and
  `autopilotGetTargetPositionEfCm()`.

### navHudState_t (consumed by the renderer)

Validity flags (`gpsValid, homeValid, positionValid, headingValid, cogValid,
gpsGlitch, stale`), rescue status (`rescueActive, rescueHealthy, rescueFailure,
phase, routeValid, rescueStartCm, targetPosCm, descentDistanceCm`), geometry
(`craftPosCm, distanceToHomeCm, bearingToHomeDeg, headingDeg, cogDeg,
desiredHeadingDeg, headingErrorDeg, crossTrackCm`), progress
(`etaSeconds, etaValid, trackTrend`), altitude/speed with rescue targets, and GPS
health (`satellites, hdop`). Phases: `IDLE, INIT, CLIMB, ORIENT, TURN, RETURN,
DESCEND, LAND, HOLD, ABORTED, UNAVAILABLE` — a 1:1 honest mapping of
`rescuePhase_e` (`ORIENT` = `RESCUE_PITCH_FORWARD` IMU-heading recovery,
`ABORTED` = `RESCUE_EMERG_DESCENT`).

## 4. Coordinate system

* World frame: local ENU centimetres relative to home (home = origin), from
  `GPS_distance2d(&GPS_home_llh, &gpsSol.llh, …)` — handles the 180° meridian and
  cos(latitude) scaling.
* View transform: optional rotation by the aircraft heading (heading-up), then
  scale to cells. Heading-up is adaptive: the renderer stays north-up while
  acquiring GPS, before home, or when stationary normal flight has no useful
  GPS course authority; it rotates once the craft is moving, and during active
  GPS Rescue when heading matters. Character cells are ~1.5× taller than wide on
  both SD (12x18 px) and HD (24x36 px) grids, so `cmPerRow = 1.5 * cmPerCol`
  keeps the map isotropic.
* Centering: `AUTO` (midpoint of craft and home), `HOME`, or `CRAFT`. Default is
  `CRAFT`, so the map behaves like a minimap: the aircraft stays central while home
  and breadcrumbs move around it. Landing phase forces a home-centred 50 m view for
  touchdown accuracy.
* Auto-zoom fits craft, home and (during rescue) the engage point with 25% margin,
  quantised to 20/50/100/200/400/800/…/51200 m map widths, with hysteresis (zoom out
  immediately; zoom in only when the content needs < 35% of the current width).
* Lines are clipped Liang-Barsky in view space before integer Bresenham on cells, so
  a home point kilometres off-screen costs nothing. Off-map home is edge-pinned on
  the border. Rows below/right of the screen are clipped at emission.

## 5. Breadcrumb storage strategy

* `navHudTrailPoint_t` = `int16 eastM, int16 northM, uint8 flags` → 6 bytes/point.
  Build cap `NAV_HUD_TRAIL_CAPACITY = 240` (1440 bytes); active count configurable
  8–240 (`nav_hud_breadcrumb_count`, default 240).
* Distance-threshold sampling: a point is recorded when it is
  ≥ `nav_hud_breadcrumb_spacing` metres (default 5 m) from the last recorded point.
* **Adaptive compaction**: when the buffer is full, every second point is dropped and
  the spacing threshold doubles (capped at 5 km). A 100 km flight remains fully
  represented at progressively coarser resolution — no rolling window that forgets
  the launch area. (Douglas-Peucker was considered and rejected: O(n log n) passes
  and unpredictable stack/CPU for marginal visual benefit at ≤27 map columns.)
* Points recorded while rescue is flying carry `NAV_HUD_TRAIL_FLAG_RESCUE`, giving
  the "actual rescue track" for free.
* **Glitch filter**: a fix implying > 100 m/s (+30 m margin) movement is rejected and
  flagged (`gpsGlitch`); a second consistent sample is accepted as a genuine
  reacquisition (e.g. after GPS loss). Resets occur on arming and whenever the home
  point moves.

## 6. Route rendering vocabulary

| Layer                | Glyphs                                     | Notes |
|----------------------|--------------------------------------------|-------|
| breadcrumb history   | stick-overlay dot sprites (`0x08-0x0A`)     | 3× vertical sub-cell resolution |
| actual rescue track  | `:`                                        | denser than history dots |
| planned rescue route | 16-direction arrows pointing along route   | unmistakably "go this way" |
| descent-start marker | `SYM_ARROW_SMALL_DOWN` on the route        | where rescue begins descending |
| autopilot target     | `SYM_STICK_OVERLAY_CENTER` (small cross)   | the live "carrot" |
| direct home line     | sparse `.` every second cell               | normal flight |
| projected track      | sparse `.` along COG, 10 s ahead           | optional |
| aircraft             | 16-direction arrow (up in heading-up mode) | `SYM_OVER_HOME` when over home; COG arrow when IMU heading is untrusted; neutral dot when neither source is valid |
| home                 | `SYM_HOMEFLAG`, edge-pinned when off-map   | |
| hold anchor          | `SYM_STICK_OVERLAY_CENTER` + shimmer dots  | the autopilot's hold point during POS HOLD; dots alternate H/V every ~0.7 s so drift off the anchor is obvious |
| border               | stick-overlay box glyphs                   | home-direction cue left; ground speed `[dial][value][unit]` centred (MPH/KPH per `osd_units`); satellites right, yielding to speed on COMPACT; scale in bottom border |

Draw order (lowest → highest): projected track, breadcrumbs, planned route,
descent marker, carrot, hold anchor, home, aircraft, warnings.

**Hold-mode banner:** while POS and/or ALT hold is engaged (and rescue is not),
the first footer line shows `❯ POS+ALT HOLD ❮` (or `POS HOLD` / `ALT HOLD`;
`HOLDING` on COMPACT widths) with the brackets converging onto the label over
the first second after engaging — a deliberate "lock" animation that then sits
still.

**Instrument ladder (sanity):** map rotation and the craft arrow use the fused
IMU yaw (`attitude.values.yaw`, which blends mag when fitted/calibrated and GPS
course otherwise) only while `imuIsHeadingValid()`; without that the arrow falls
back to raw GPS course over ground above 1.5 m/s, and below that to a neutral
dot rather than a made-up direction. Heading-up rotation is gated by
`compassEnabledAndCalibrated()` — the same test the IMU uses before fusing the
mag: with a calibrated compass (e.g. on the GPS puck, wired to the FC's I2C)
the heading is true at any speed and the map rotates even at hover; without one
the rotation uses 2.5/1.5 m/s enter/exit hysteresis so hovering on
GPS-course-derived heading doesn't flap the map.

## 7. Display modes and mockups

`nav_hud_mode`: `OFF | COMPACT (13x7) | STANDARD (19x9) | FULL (27x13)`.
Default presentation is COMPACT, adaptive heading-up, with GPS health hidden: the
map is small enough for normal flying, stays calm while parked or seeking GPS,
rotates with the craft in real flight, and keeps the footer focused on home
distance and altitude.
Before fix/home, the map interior shows a restrained animated `WAIT GPS` / `WAIT GPS FIX`
message while the top border remains the single source for satellite count.
`nav_hud_rescue_auto_expand` grows the map one size while rescue is active.
"Minimal rescue" for small screens = `COMPACT` + the flight-director element.

Legend for the mockups: `·` history dots, `:` actual rescue track, `➤` aircraft
arrow, `⌂` home flag, `→/↘…` route arrows, `▼` descent marker, `+` carrot,
`↑/↓` climb/descent, `▲` small up arrow.

**Normal flight, COMPACT (heading-up, auto-zoom):**

```
┌⌂↙ ⏲47ᴹ ──┐
│   ··      │
│  ·  .  ▲  │
│ ⌂· .      │        map:    craft-centred trail, direct home line, home
│   ·       │        border: home cue left, speed centred, scale bottom
└───────400M┘
 ⌂428M   ↕91M
```

**Position hold engaged (banner brackets converge, anchor shimmers):**

```
┌⌂↙ ⏲2ᴹ S12┐
│   ··      │
│  ·  ·┼·   │        ┼ = autopilot hold point, dots shimmer around it
│ ⌂·   ▲    │        craft arrow shows drift relative to the anchor
│           │
└────────50M┘
 ❯POS+ALT HOLD❮
 ⌂428M   ↕91M
```

**Rescue TURN phase (auto-expanded to FULL):**

```
┌N▲───────────────────────┐
│      ·······            │
│   ···       ····        │
│  ·              ··      │
│ ⌂ ←←←←←←←←←←▼←←←← ➤     │      planned route: arrow chain engage→home
│                         │      ▼ = descent ring crossing
│                         │
│                         │
│                         │
│                         │
│                         │
└────────────────────1600M┘
 RTH TURN
 ⌂612M  ✈-:--
 ↑ 91/100M 0/54K
 ⛭18 H0.9
```

**Rescue RETURN phase:**

```
┌N▲───────────────────────┐
│      ·······            │
│   ···       ····        │
│  ·              ··      │
│ ⌂ ←←←←←▼←←+::::::➤      │      : = actual rescue track since engage
│                         │      + = live autopilot target (carrot)
│                         │
└────────────────────1600M┘
 RTH RETURN
 ⌂512M  ✈0:38
 ↑ 98/100M 52/54K
 X9MR ⛭18 H0.9
```

**Rescue DESCEND / LAND (auto home-centred, 50 m zoom):**

```
┌N▲───────────┐
│      :      │
│     ::      │
│    :        │
│   ➤         │
│  ⌂          │
└──────────50M┘
 RTH LAND
 ⌂6M  ✈0:04
 ↓ 8/0M 4/0K
```

**Flight director (independent element, rescue only):**

```
    ⌂090 ››
E100R X9MR
▲12M V1.2
```
Row 1: desired heading with turn-direction arrows (1–3 per 25° of error);
row 2: heading error side + cross-track; row 3: altitude error to the rescue
target + vertical speed.

**Degraded states:** `NO HOME` / `NO GPS` centred in the map, `STALE` overlay when
no accepted fix for 2.5 s (craft marker suppressed, `positionValid` false), `!`
appended to the home distance on glitch, banner shows `RTH FLYAWAY / GPS LOST /
LOW SATS / STALLED / NO HOME` from the real `rescueFailureState_e`, and the
existing `osd_warnings.c` `RESCUE FAIL / RESCUE N/A` warnings are untouched and
still take priority on screen.

## 8. Configuration

| Setting | Values / range | Default |
|---|---|---|
| `nav_hud_mode` | OFF, COMPACT, STANDARD, FULL | COMPACT |
| `nav_hud_orientation` | NORTH_UP, HEADING_UP | HEADING_UP |
| `nav_hud_center` | AUTO, HOME, CRAFT | CRAFT |
| `nav_hud_auto_zoom` | OFF, ON | ON |
| `nav_hud_fixed_scale` | 20–10000 m map width | 400 |
| `nav_hud_breadcrumbs` | OFF, ON | ON |
| `nav_hud_breadcrumb_count` | 8–240 | 240 |
| `nav_hud_breadcrumb_spacing` | 5–500 m | 5 |
| `nav_hud_projected_track` | OFF, ON | OFF |
| `nav_hud_direct_home_line` | OFF, ON | ON |
| `nav_hud_rescue_route` | OFF, ON | ON |
| `nav_hud_rescue_auto_expand` | OFF, ON | ON |
| `nav_hud_show_eta` | OFF, ON | ON |
| `nav_hud_show_cross_track` | OFF, ON | ON |
| `nav_hud_show_targets` | OFF, ON | ON |
| `nav_hud_show_gps_health` | OFF, ON | OFF |
| `nav_hud_show_speed` | OFF, ON | ON |

**Local build note:** building from a vendored board config (`make <CONFIG_NAME>`)
defines `USE_CONFIG`, which — exactly like Configurator cloud builds — makes
optional hardware opt-in rather than kitchen-sink. A GPS puck's compass therefore
needs `make <CONFIG_NAME> OPTIONS="USE_MAG"` (the option expands to the full mag
driver suite in `common_post.h`). VTX control, GPS, position/altitude hold are
already included by the default config build.

Element visibility/position uses the normal OSD element mechanism
(`osd_nav_hud_pos`, `osd_nav_flight_director_pos` via `item_pos[]`; both are off by
default like every element). A `nav_hud_refresh_rate` setting was deliberately
omitted: the OSD clears and redraws the whole screen every frame, so the redraw rate
is governed by `osd_framerate_hz`; the state snapshot is internally capped at 20 Hz.

## 9. Performance

Measured with `arm-gnu-toolchain 13.3.rel1` on `GEPRC_TAKER_H743` (STM32H743),
this branch vs. its master merge-base, clean builds:

* **Flash:** text+data 575,141+7,740 vs. 565,881+7,740 → **+9,260 bytes (~9.0 KB)**
  including the renderer, core, hold/speedometer additions, settings tables and
  getters (both new files are in `SIZE_OPTIMISED_SRC`). GPS-capable targets are
  ≥1 MB flash by `TARGET_FLASH_SIZE` gating, so this is < 0.9% of the budget.
* **RAM (static):** bss 109,320 vs. 107,136 → **+2,184 bytes**: trail 1,440 B
  (240 points), core state/bookkeeping ≈ 170 B, renderer frame buffer (27x13 grid +
  4 footer lines + projection state) ≈ 510 B, PG copies ≈ 60 B. No heap, no
  per-frame stack beyond ~120 B of locals.
* **CPU:** per GPS fix (≤ 20 Hz): ~40 FLOPs (glitch window + EMA + occasional trail
  push) — negligible. Per OSD frame (default 12 Hz): one frame preparation =
  ≤120 point projections + 2 clipped lines + footer sprintf ≈ 3–5 k cycles ≈ **tens
  of µs on an F405, single-digit µs on H7**; then 10–16 row-copy passes that the OSD
  scheduler already budgets per element (same mechanism as the artificial horizon).
  The GPS task gains one function call per fix.
* **DisplayPort bandwidth** (MSP): STANDARD ≈ 12 writes ≈ 350 B/frame ≈ 4.2 KB/s at
  12 Hz; FULL + 4 footers ≈ 600 B/frame ≈ 7.2 KB/s. A 115200-baud MSP link carries
  ≈ 11.5 KB/s total, so on analog-SD MSP links use COMPACT or reduce
  `osd_framerate_hz` to 8–10 when running FULL. HD systems (DJI O3/O4, HDZero,
  Walksnail) negotiate faster links and full-screen HD redraws already dwarf this.
* **Target classes:** F405/F7: full feature, default settings recommended
  (COMPACT/STANDARD). H7/F765: FULL mode + 120 breadcrumbs comfortably. Sub-1MB
  targets: excluded automatically because `USE_GPS` is not defined there.

## 10. Testing

* `src/test/unit/nav_hud_unittest.cc` (26 tests, all passing): angle wrapping,
  bearing, cross-track sign/magnitude, auto-zoom quantisation, breadcrumb spacing,
  compaction (halving + spacing doubling), rescue-track flagging, ±32 km clamping,
  single-sample glitch rejection, consistent-jump reacquisition, state assembly,
  ETA from closing speed, heading wraparound, rescue phase mapping and route
  capture, cross-track appearance when off-route, trail reset on arm/home-move,
  staleness, mode-off gating — plus a renderer harness that rasterises the real
  element functions into a virtual 53x20 screen and asserts the border, home
  direction cue, home flag, craft arrow, route arrows, expansion, phase/failure
  banners and flight-director output.
* Regression: `osd_unittest`, `link_quality_unittest`, `cli_unittest`,
  `althold_unittest`, `flight_failsafe_unittest`, `check-pg-ids` all pass, and
  `make TARGET=SITL` builds cleanly.

### SITL plan (follow-up)

SITL (`src/platform/SIMULATOR`) has `USE_GPS`, `USE_VIRTUAL_GPS` and GPS Rescue but
`#undef USE_OSD`. Proposed harness:

1. Feed recorded tracks (GPX/CSV → `gpsSetFixState` + `gpsSol` via the virtual GPS)
   through `src/test/sitl/sitl_harness.py`, including: long-range cruises, a
   180°-meridian crossing, GPS dropouts, and a scripted failsafe that engages rescue.
2. Assert core behaviour via MSP debug values or blackbox: trail point counts and
   compaction, rescue-start capture, cross-track sign vs. injected wind drift, ETA
   convergence.
3. Optional: enable `USE_OSD` + `USE_MSP_DISPLAYPORT` in the SITL target and attach a
   displayport listener to snapshot rendered frames per rescue phase (gives real
   "screenshots" for documentation).

## 11. Configurator work (betaflight-configurator; repo not in this tree)

Append-only changes, keyed to the two new element indices:

1. **Element registration** — `src/js/tabs/osd.js`, `OSD.constants.DISPLAY_FIELDS`:
   add `NAV_HUD` and `NAV_FLIGHT_DIRECTOR` entries (after the WP elements in this
   fork) with `positionable: true` and preview functions. The firmware transmits
   `OSD_ITEM_COUNT` dynamically in `MSP_OSD_CONFIG`, so no MSP change is needed —
   but the Configurator's field list must be appended in exactly this order.
2. **Preview** — the minimap preview renders the STANDARD 19x9 frame from this
   document (border + `N` + scale + sample trail/home/craft glyphs); a phase
   dropdown in the preview pane switches between the normal / TURN / RETURN /
   DESCEND sample frames so users can preview rescue layouts. Glyph indices match
   the standard font (`0x60-0x6F` arrows, `0x11` home, `0x16/0x17` box, `0x08-0x0A`
   dots).
3. **Settings UI** — the `nav_hud_*` settings appear automatically in the CLI and
   in Presets; optionally add a "Navigation HUD" section to the OSD tab bound to
   `nav_hud_mode/orientation/center/auto_zoom/...` via MSP `settings` API, with the
   preview switching size when `nav_hud_rescue_auto_expand` is toggled.
4. **Font page note** — none required (standard glyphs only). If the optional
   enhanced glyph set (below) is adopted, the font files gain page `0xA0-0xB7` and
   the font-upload tab needs no code change (fonts are data).

## 12. Optional enhanced glyph set (future, MAX7456/custom goggle fonts)

The standard-font renderer is complete, but 24 free codepoints at `0xA0-0xB7`
(verified unused in `osd_symbols.h`; `0xA0-0xFE` is free) would upgrade line
quality on systems where the user can upload fonts:

* `0xA0-0xA7`: thin line segments in 8 orientations (22.5° steps cover with mirroring)
* `0xA8-0xAF`: bold route segments in 8 orientations
* `0xB0-0xB2`: 3 dot densities (history / rescue track / faded)
* `0xB3-0xB6`: box corners for a seamless border
* `0xB7`: descent-ring marker

The renderer would select these behind a `displaySupportsOsdSymbols()`-style
capability check with automatic fallback to the standard set. Not implemented in
this change to keep DJI/HDZero/Walksnail first-class.

## 13. Upstreaming concerns

* This fork's `osd_items_e` already contains a conditionally-compiled `OSD_WP_*`
  block, so `OSD_ITEM_COUNT` varies per build; the new elements were added
  unconditionally (draw slots gated) to avoid making that worse. Upstream would want
  the same unconditional style.
* `PG_OSD_ELEMENT_CONFIG` version was bumped 3→4 (item_pos array grew) — stored
  layouts reset on upgrade, as usual for element additions.
* `PG_NAV_HUD_CONFIG = 567` — coordinate with upstream PG allocation before PR.
* The rescue getters are read-only and additive; wing builds compile them out
  (`!USE_WING` guards) and the HUD degrades to a generic rescue display on wings.
* Settings names follow the `nav_hud_` prefix; upstream may prefer `osd_nav_`.

## 14. Limitations

* Character-cell resolution: at a 400 m map width each cell is ~24 m horizontally;
  the trail is impressionistic, not survey-grade. Sub-cell dot sprites mitigate this
  vertically only.
* The rescue "planned route" is the engage-point→home line. Betaflight rescue
  re-aims at home continuously, so after large wind drift the *live* intent is the
  carrot marker + arrows, while cross-track is measured against the original line —
  this is labelled deviation, not a controller error.
* Heading-up mode rotates with raw yaw at the frame rate; at 12 Hz fast yaw can look
  steppy. North-up is the default.
* No per-cell blink (row writes share one attribute); critical alerts rely on the
  existing `osd_warnings` element, which already blinks.
* FULL mode (13 map rows + up to 4 footers) exceeds a 16-row SD screen if placed low;
  rows are clipped rather than reflowed.
* CPU figures are analytic (derived from operation counts); flash/RAM are measured
  (§9). No on-bench task-time capture yet — check `tasks` CLI output on hardware.

## 15. Future improvements

* Enhanced glyph set (§12) and per-device capability negotiation.
* Flight-plan (waypoint mission) overlay: `flightPlanConfig()` waypoints and
  `positionNavGetActiveCommand()` are already accessible — render the mission
  polyline and the active leg with the same vocabulary.
* Persist the trail across a mid-air reboot (config-flash scratch area).
* SITL displayport snapshot harness (§10) and golden-frame regression images.
* CMS menu for on-the-fly mode/orientation changes.
* Elevation profile strip for DESCEND phase on FULL layouts.


## Waypoint missions

Waypoints and missions ride on two GUI-visible modes (Modes tab):

| Mode | Switch style | Action |
|---|---|---|
| **USER2** | momentary | **Tap** = drop a numbered waypoint at the craft's position (captured at the press, committed on release). **Hold ~1.5 s** = delete the last waypoint (undo a mis-drop). Up to 8 per flight; cleared at each arm. |
| **USER3** | latching | **On** = fly the dropped waypoints in order. **Off** = stop and return control. |

### How a mission flies

One switch does everything: engaging USER3 self-enables the position and
altitude hold engines (the OSD flymode will read `POSH` while it flies), captures
the current altitude, and then flies each leg **nose-first**:

1. rotate in place until the nose points at the active waypoint
   (GPS Rescue's yaw law, `gps_rescue_yaw_p`),
2. cruise the leg at `gps_rescue_ground_speed` (a carrot target marched ahead
   of the craft, chased by the position-hold autopilot - tuning rescue tunes
   missions and vice versa),
3. within 5 m of the waypoint, begin the next leg,
4. after the last waypoint, park on it (position + altitude hold) until the
   switch is released.

The map shows the dashed guide line to the active waypoint (its marker
blinks) and a footer with `WP n/m` plus the distance to go.

### 3D missions (stadium profile)

With `nav_hud_mission_3d = ON` (default), every waypoint records the altitude
flown when it was dropped, and the mission flies a **stadium profile** on GPS
Rescue's climb/descend rates:

- legs between waypoints at/above the hard deck fly as SLANTED climbs and
  descents at the rescue ascend/descend rates - direct angled flight to the
  next altitude, never below `gps_rescue_return_alt`,
- a waypoint recorded below the deck is a deliberate low pass: cruise at the
  deck, arrive directly overhead, dip vertically to the recorded altitude
  (which the craft demonstrably flew at that exact spot), then climb back out
  onto the next leg,
- the final waypoint is descended to and held at its recorded altitude until
  the switch is released,
- waypoint markers on the map carry a compact altitude tag (`1:500`), and the
  auto-zoom keeps the whole mission in view. Set `nav_hud_mission_3d = OFF`
  for classic fixed-altitude missions.

### Safety

- **Sticks always win**: roll/pitch beyond 15% deflection aborts instantly
  (`WP ABORT` in the footer). Every abort latches until USER3 is cycled.
- Failsafe, GPS Rescue, or a lost fix abort the mission; RC loss falls through
  to the configured failsafe procedure as usual.
- Missions refuse to engage on the ground (< 3 m altitude), with no waypoints,
  or with any waypoint beyond 2 km from home.
- Dropping position hold (if the pilot had it switched on) aborts rather than
  letting the mission resume unannounced.

### Field tuning

Set `debug_mode = NAV_MISSION` before a mission flight and blackbox records:
phase, active leg, distance to waypoint (dm), heading error (deg), commanded
yaw rate (dps), carrot progress and craft progress along the leg (dm). If a
mission ever behaves oddly, this log answers why.

### Verification

`src/test/unit/nav_mission_sim_unittest.cc` is a software-in-the-loop harness:
the real sequencer, position-hold engine and autopilot PIDs flown by a
point-mass physics model. A full three-waypoint mission, a worst-case
(engaged facing away) turn, and a mid-cruise stick abort are simulated on
every test run.
