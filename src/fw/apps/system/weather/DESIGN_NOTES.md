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

## Layout spec (as built)
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

## Keep authentic (don't over-design)
- One family + LECO hero number; flat fills / 1px hairlines / hollow rings only.
- Keep the `weather_types.c` condition→colour map (only add outlines).
- Colour strictly functional & sparse; one accent.
- Exactly ONE animated element (today icon); row + graph stay static.
- ALL-CAPS weekday tokens, Title-case date/condition; left-aligned text + single edge margin.

_Note: the synthesis assumed a VividCerulean top "location strip" status bar — the forecast screen
does **not** have one (neither does gabbro). We reserve a modest top margin instead of adding a strip._
