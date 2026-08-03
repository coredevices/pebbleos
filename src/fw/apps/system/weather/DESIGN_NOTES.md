# Emery Weather Forecast — Design Notes

Research synthesis (a survey of Pebble design language, best-in-class weather UIs,
small rectangular-screen UX, typography, colour, temp-graph data-viz, layout grids) for the
**emery / obelix 200×228** animated 5-day forecast screen. Goal: make it read as a polished,
**official first-party Pebble app**, matching the round (gabbro) build's quality.

## Pebble first-party design rules (the "feel")
- **Content-first, one purpose.** Flat white background, e-paper-style solid fills, no gradients.
- **Type IS the hierarchy.** One family (Raster Gothic Condensed) + a numeric display font (LECO).
  Rank by size+weight, not by switching fonts. 3 tiers only.
- **Colour is functional + sparse (~10% budget).** Black on white default; saturated hues only to
  ENCODE data (warm=high, cool=low, condition discs). Mute connective elements.
- **Flat fills, 1px hairlines, hollow rings.** A 1px `GColorLightGray` hairline is the canonical
  region divider (timeline-card idiom; see the gauge port in `expanded_view.c`).
- **Respect the grid.** One consistent edge margin (left == right), top breathing room, left-aligned
  body text; centre only symmetric column/hero elements.
- **One animated element** (the today icon). Everything else still.

## Critique of the current emery screen
- Header is two corner-pinned objects, not a balanced pair — the descender-less LECO temp parks low
  vs the icon's visual mass (hence the repeated hand-nudging). Fix by optical CAP-HEIGHT centering.
- Flat header hierarchy: today date + condition are Gothic 14 — same as the tiny fan labels.
- Inconsistent insets (icon ~18px vs hairline 8px) — edges don't line up.
- Hi/lo graph is two CO-EQUAL lines + 10 black numerals crammed in a ~36px band — reads "plotted"/busy.
- Low series in VividCerulean/PictonBlue fails the 3:1 non-text contrast floor on white (washed out).
- Five fully-saturated discs = a rainbow that steals the colour budget; pale discs dissolve into white.

## Redesign spec (target)
**Grid:** 8px margin every side (usable x=8..192). 5 shared column centres `x = 8 + (i+0.5)*36.8` →
**26, 63, 100, 136, 173**, reused by the disc row AND the graph dots (vertical line connects a day's
icon to its temps). Bands: HEADER (icon+temp) → 1px hairline → DAY ROW (weekday+disc) → HI/LO GRAPH,
with even gaps and a symmetric ~8px top/bottom margin.

**Header:** 40×40 animated icon left edge at x=8; LECO_36_BOLD_NUMBERS temp right edge at x=192; both
share a vertical centreline; centre the temp on the digits' CAP-HEIGHT midpoint (fixed optical rise,
not hand-tuning). Date `Sat, Jun 27` in **GOTHIC_18_BOLD** left@8; condition `Heavy Snow` in
**GOTHIC_18 regular** right@192 (Title-case, 1 line + ellipsis) — bumped from 14 so the header outranks
the fan. Hairline 1px `GColorLightGray` x=8..192, ~8px clear above/below.

**Day row:** weekday SUN…THU `GOTHIC_14_BOLD` ALL-CAPS centred on the 5 column centres; condition disc
r=13 (26px) with a **1px outline ring** (LightGray for saturated discs, Black for pale ones:
ElectricBlue/TiffanyBlue/ChromeYellow/LightGray) so edges are always defined; 25px PDC icon centred.
**Today's weekday label is the only accent — GColorOrange**; the other four stay black.

**Hi/lo graph (the biggest upgrade):** ONE shared temperature scale `[minLow-1, maxHigh+1]`, clamp the
mapped pixel spread to ≥16px so a calm week still undulates. Highs = **HERO**: 2px `GColorWindsorTan`
connector + hollow r4 dots (white fill, 2px `GColorOrange` ring), numerals `GOTHIC_14_BOLD` black ABOVE.
Lows = **SUPPORTING**: 1px `GColorCadetBlue` connector + solid r2 `GColorCadetBlue` dots, numerals
`GOTHIC_14` regular `GColorCadetBlue` BELOW. No degree glyphs on graph numerals. Draw lines first, then
dots. No area fill, no axis box, no gridlines. (Optional: 1px LightGray vertical stem hi→lo per day.)

**Type ramp (strict 3 tiers):** HERO = LECO_36_BOLD_NUMBERS (+ Gothic 24 bold degree). PRIMARY =
GOTHIC_18 (bold date / regular condition). DETAIL = GOTHIC_14 (bold weekday + high numerals / regular low).

**Colour:** white bg; black header type (do NOT tint the hero temp); hairline + disc outlines + stems
LightGray; condition discs keep `weather_types.c` map; graph high = Orange dots / WindsorTan line;
graph low = CadetBlue (NOT VividCerulean/PictonBlue); today accent = Orange on leftmost weekday only.

## Prioritised changes (impact-ranked)
1. **[HIGH]** Optically pair the header (icon left@8 + temp right@192, shared centreline, CAP-height rise).
2. **[HIGH]** (Optional) split degree off the LECO string (LECO digits + Gothic 24 °) — only if the
   combined `"%d°"` ever mis-renders; currently it renders fine, so keep unless it breaks.
3. **[HIGH]** Rebuild the graph: hero highs (Orange hollow r4 + 2px WindsorTan) / supporting lows
   (CadetBlue 1px + solid r2); drop degree glyphs; high numerals bold-black above, low regular-blue below.
4. **[HIGH]** One shared temp scale with a ≥16px min-spread clamp (kills flat-line ambiguity).
5. **[HIGH]** Unify to an 8px MARGIN; 5 shared column centres reused by row + graph.
6. **[MED]** Type ramp: date→Gothic 18 bold, condition→Gothic 18 regular.
7. **[MED]** Disc outlines + single Orange accent on today's weekday label.
8. **[MED]** Low series → CadetBlue everywhere (contrast); recolour raindrop metric glyph to black.
9. **[LOW]** No area fill between lines; optional 1px LightGray hi→lo stem.

## Keep authentic (don't over-design)
- One family + LECO hero number; flat fills / 1px hairlines / hollow rings only.
- Keep the `weather_types.c` condition→colour map (only add outlines).
- Colour strictly functional & sparse; one accent.
- Exactly ONE animated element (today icon); row + graph stay static.
- ALL-CAPS weekday tokens, Title-case date/condition; left-aligned text + single edge margin.

_Note: the synthesis assumed a VividCerulean top "location strip" status bar — the forecast screen
does **not** have one (neither does gabbro). We reserve a modest top margin instead of adding a strip._

## v4.2 schema — warning readings (FOR STEVE, )

The weather report screen's alert line is now a proper warnings ladder. It needs four new
raw readings per location, appended to the WeatherDBEntry as **v4 minor 2** (see
`include/pbl/services/blob_db/weather_db.h` — fields sit after `daily_metrics[]`, before the
trailing pstrings; stamp `minor_version = 2`). All are TODAY-only. Open-Meteo mapping:

| Field | Type | Open-Meteo source | Notes |
|---|---|---|---|
| `today_wmo_code` | u8 | daily `weather_code` | raw WMO 4677 code; 0xFF unknown |
| `today_humidity_pct` | u8 | hourly `relative_humidity_2m` → daily MEAN | 0..100; 0xFF unknown |
| `today_visibility_m` | u16 | hourly `visibility` → daily MINIMUM, meters | clamp 65534; 0xFFFF unknown |
| `today_precip_sum_mm` | u16 | daily `precipitation_sum`, whole mm | clamp 65534; 0xFFFF unknown |
| `daily_feels_like[7]` | i16 x7 | daily `apparent_temperature_max`, per day (index 0 = today) | same unit as the other temps; 32767 (UNKNOWN_TEMP) per unknown slot |

Watch-side thresholds (weather_report.c `prv_build_alert`, most severe first): Hail (wmo 96/99),
Chance of storms (95-99), Heavy snow (type or wmo 75/86), Heavy rain, Wintry mix, Flood risk
(precip_sum ≥ 30mm), Strong winds (≥ 25mph), Below freezing (high ≤ 0), Hard frost (low ≤ −4),
Feels below freezing (feels ≤ 0 < actual), Heatwave (high ≥ 30), Very high UV (≥ 8), High UV (≥ 6),
Poor visibility (wmo 45/48 or vis ≤ 1000m), High humidity (≥ 85% and high ≥ 20), Snow/Rain likely
(prob ≥ 60), Chance of snow/rain/sleet (prob > 0), Windy (≥ 20mph); calm sign-offs otherwise
(Clear skies / Light winds / Calm breeze / Fair conditions / Mild conditions / All clear).
Old-minor records simply never fire the new rungs — nothing breaks if v4.2 ships later.

## FOR STEVE — schema v4.3 addition 

One more appended block after the v4.2 fields (same append-only pattern; minor
version bumps 2 → 3; older records parse unchanged):

| field | type | source (Open-Meteo) | unknown |
|---|---|---|---|
| `today_wind_dir_deg` | i16 | `winddirection_10m_dominant` for today (or dominant of today's hourly `winddirection_10m`) | -1 |
| `daily_wind_dir_deg[7]` | i16[7] | daily `winddirection_10m_dominant` | -1 |

Degrees 0..359, meteorological (0 = N, clockwise). Used by the day screen's
forecast description ("Winds SW at 12mph") — 8-point compass rendering on-watch.
