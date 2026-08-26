#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

# coding=utf-8

import argparse
import copy
import json
import re
import sys
import urllib
from math import sqrt
from os.path import basename, splitext

COLORS_JSON_PREFIX = splitext(basename(__file__))[0]
COLORLOVERS_COLORS_JSON = COLORS_JSON_PREFIX + "_colorlovers.json"
WIKIPEDIA_COLORS_JSON = COLORS_JSON_PREFIX + "_wikipedia.json"


def download_values_from_color_lovers(r, g, b):
    """
    Returns values for a single color from colourlovers.com

    NOTE: does a single HTTP request per call, please call wisely
    """

    url = "http://www.colourlovers.com/api/color/%02x%02x%02x?format=json" % (r, g, b)
    opener = urllib.request.build_opener()
    opener.addheaders = [("User-agent", "Mozilla/5.0")]
    response = opener.open(url)
    s = response.read()
    values = json.loads(s)
    print("r: %03d, g:%03d, b:%03d: %s" % (r, g, b, values))
    return values


def download_all_colors_from_color_lovers():
    """
    Requests and caches into colorlovers_colors.json all 64 colors
    """
    colors = []
    for r2 in range(4):
        for g2 in range(4):
            for b2 in range(4):
                colors += download_values_from_color_lovers(r2 * 85, g2 * 85, b2 * 85)

    with open(COLORLOVERS_COLORS_JSON, "w") as f:
        json.dump({"colors": colors}, f, indent=2)


def load_cached_colorlovers_colors():
    """
    Loads cached color values from colourlovers.com and converts them to the expected format
    """
    with open(COLORLOVERS_COLORS_JSON) as f:
        colors = json.load(f)["colors"]
        return [
            {
                "r": c["rgb"]["red"],
                "g": c["rgb"]["green"],
                "b": c["rgb"]["blue"],
                "name": c["title"],
                "source": c["url"],
            }
            for c in colors
        ]


def parse_colors_from_wikipedia_html(html):
    """
    Requests and return all color values from a single, paginated wikipedia "list of colors" page
    """
    from bs4 import BeautifulSoup

    url_base = "http://en.wikipedia.org"
    colors = []
    soup = BeautifulSoup(html, features="html.parser")
    for tr in soup.find("table").find_all("tr"):
        tds = tr.find_all("td")
        if len(tds) == 9:
            hex = tds[0].text.strip()
            r = int(hex[1 : 1 + 2], 16)
            g = int(hex[3 : 3 + 2], 16)
            b = int(hex[5 : 5 + 2], 16)
            color = {"r": r, "g": g, "b": b}

            th_a = tr.find("th").find("a")
            if th_a is None:
                color["name"] = tr.find("th").text.strip()
                color["url"] = "http://en.wikipedia.org/wiki/List_of_colors"
            else:
                color["name"] = th_a.text.strip()
                color["url"] = url_base + th_a["href"]

            print(color)
            colors.append(color)

    return colors


def download_and_parse_colors_from_wikipedia():
    """
    Requests and caches into wikipedia_colors.json all available colors from wikipedias "list of colors" pages
    """
    from mediawikiapi import Config, MediaWikiAPI

    wikipedia = MediaWikiAPI(config=Config(timeout=15))

    colors = []
    for title in ["List of colors: A-F", "List of colors: G-M", "List of colors: N-Z"]:
        colors += parse_colors_from_wikipedia_html(wikipedia.page(title).html())

    with open(WIKIPEDIA_COLORS_JSON, "w") as f:
        json.dump({"colors": colors}, f, indent=2)
    return colors


def load_cached_wikipedia_colors():
    """
    loads all previously cached colors from wikipedia
    """
    with open(WIKIPEDIA_COLORS_JSON) as f:
        return [copy.copy(c) for c in json.load(f)["colors"]]


def hardwired_colors():
    """
    creates a set of hard-wired colors. Used to overrule any other source.
    """
    result = []

    result.append(
        {
            "r": 0,
            "g": 85,
            "b": 255,
            "name": "Blue Moon",
            "url": "http://en.wikipedia.org/wiki/Blue_Moon_(beer)",
        }
    )
    result.append(
        {
            "r": 0,
            "g": 170,
            "b": 85,
            "name": "Jaeger Green",
            "url": "http://en.wikipedia.org/wiki/Jägermeister",
        }
    )

    result.append(
        {
            "r": 0,
            "g": 255,
            "b": 0,
            "name": "Green",
            "url": "http://en.wikipedia.org/wiki/Green",
        }
    )
    result.append(
        {
            "r": 0,
            "g": 255,
            "b": 255,
            "name": "Cyan",
            "url": "http://en.wikipedia.org/wiki/Cyan",
        }
    )
    result.append(
        {
            "r": 255,
            "g": 0,
            "b": 255,
            "name": "Magenta",
            "url": "http://en.wikipedia.org/wiki/Magenta",
        }
    )
    result.append(
        {
            "r": 255,
            "g": 0,
            "b": 170,
            "name": "Fashion Magenta",
            "url": "http://en.wikipedia.org/wiki/Fuchsia_(color)#Fashion_fuchsia",
        }
    )
    result.append(
        {
            "r": 255,
            "g": 255,
            "b": 0,
            "name": "Yellow",
            "url": "http://en.wikipedia.org/wiki/Yellow",
        }
    )
    result.append(
        {
            "r": 255,
            "g": 85,
            "b": 0,
            "name": "Orange",
            "url": "http://en.wikipedia.org/wiki/Orange_(colour)",
        }
    )  # verify with display
    result.append(
        {
            "r": 170,
            "g": 0,
            "b": 170,
            "name": "Purple",
            "url": "http://en.wikipedia.org/wiki/Purple",
        }
    )  # verify with display
    # TODO: find brown value, core graphics says: r:0.6,g:0.4,b:0.2

    # colors to match CoreGraphics names
    result.append(
        {
            "r": 85,
            "g": 85,
            "b": 85,
            "name": "Dark Gray",
            "url": "http://en.wikipedia.org/wiki/Shades_of_gray#Dark_medium_gray_.28dark_gray_.28X11.29.29",
        }
    )
    result.append(
        {
            "r": 170,
            "g": 170,
            "b": 170,
            "name": "Light Gray",
            "url": "http://en.wikipedia.org/wiki/Shades_of_gray#Light_gray",
        }
    )

    return result


def color_dist(a, b):
    # TODO: use YUV dist
    sum = 0
    for c in ["r", "g", "b"]:
        sum += abs(a[c] - b[c]) ** 2
    return sqrt(sum)


def closest_color(c, colors):
    min_dist = None
    min_value = None
    for candidate in colors:
        dist = color_dist(c, candidate)
        if min_dist is None or dist < min_dist:
            min_dist = dist
            min_value = candidate
            if dist == 0:
                break
    return min_value


def enhanced_color(color):
    """
    Add additional, derived data to a color used for json output, c header file generation, etc.
    """
    result = copy.copy(color)
    result["identifier"] = (
        re.sub(r"\([^\)]+\)|[\s_'-]", " ", color["name"]).title().replace(" ", "")
    )
    result["name"] = result["name"].title()
    r = result["r"]
    g = result["g"]
    b = result["b"]
    r2 = int(r / 85)
    g2 = int(g / 85)
    b2 = int(b / 85)

    c_identifier = "GColor%s" % result["identifier"]
    result["c_identifier"] = c_identifier
    result["c_value_identifier"] = "GColor%sARGB8" % result["identifier"]
    hex_value = "0x%0.2X%0.2X%0.2X" % (r, g, b)
    html_value = "#%0.2X%0.2X%0.2X" % (r, g, b)
    result["html"] = html_value
    binary = f"0b11{r2:02b}{g2:02b}{b2:02b}"
    result["binary"] = binary

    result["literals"] = [
        {"id": "define", "description": "SDK Constant", "value": c_identifier},
        {
            "id": "rgb",
            "description": "Code (RGB)",
            "value": "GColorFromRGB(%d, %d, %d)" % (r, g, b),
        },
        {
            "id": "hex",
            "description": "Code (Hex)",
            "value": "GColorFromHEX(%s)" % hex_value,
        },
        {"id": "html", "description": "HTML code", "value": html_value},
        {
            "id": "gcolor_argb",
            "description": "GColor (argb)",
            "value": "(GColor){.argb=%s}" % binary,
        },
        {
            "id": "gcolor_fields",
            "description": "GColor (components)",
            "value": f"(GColor){{.a=0b11, .r=0b{r2:02b}, .g=0b{g2:02b}, .b=0b{b2:02b}}}",
        },
    ]

    return result


def validate_colors(colors):
    """
    Some sanity checks on the set of colors.
    """
    if len(colors) != 64:
        raise Exception(
            "Number of derived colors (%d) is different from expectation (64)",
            len(colors),
        )

    for c in colors:
        if len([cc for cc in colors if cc["identifier"] == c["identifier"]]) != 1:
            raise Exception("duplicate identifier name: %s and %s", c["name"])


def all_colors_with_names():
    """
    Will construct a set of our 64 colors with the closest colors from a hard-wired set of sources.
    """
    candidates = []
    candidates += hardwired_colors()
    try:
        candidates += load_cached_wikipedia_colors()
        # for now, we only look at colors from wikipedia
        # color lovers code can be deleted as soon as we agreed on final color names
        # candidates += load_colorlovers_colors()
    except OSError as e:
        raise OSError(
            "%s\n\n%s" % (e, "make sure you called --download_wikipedia once")
        )

    result = []
    for r2 in range(4):
        for g2 in range(4):
            for b2 in range(4):
                c = {"r": r2 * 85, "g": g2 * 85, "b": b2 * 85}
                closest = closest_color(c, candidates)
                dist = color_dist(c, closest)
                c["dist"] = dist
                c["closest"] = closest
                for k in ["name", "url"]:
                    if k in closest:
                        c[k] = closest[k]
                result.append(enhanced_color(c))

    validate_colors(result)

    return result


def render_header(colors):
    """
    produces the contents of color_definitions.h
    """
    color_value_maxlen = max([len(c["c_value_identifier"]) for c in colors])
    color_value_defines = []
    color_value_defines.append(
        "//%s AARRGGBB" % "".ljust(color_value_maxlen + len("#define (uint_8_t)"))
    )
    for c in colors:
        identifier = c["c_value_identifier"]
        color_value_defines.append(
            "#define %s ((uint8_t)%s)"
            % (identifier.ljust(color_value_maxlen), c["binary"])
        )

    color_define_maxlen = max([len(c["c_identifier"]) for c in colors])
    color_defines = []
    for c in colors:
        identifier = c["c_identifier"]
        value_identifier = c["c_value_identifier"]
        hex_value = "#%0.2X%0.2X%0.2X" % (c["r"], c["g"], c["b"])
        color_defines.append("")
        color_defines.append(
            '//! <span class="gcolor_sample" style="background-color: %s;"></span> <a href="https://developer.getpebble.com/tools/color-picker/%s">%s</a>'
            % (hex_value, hex_value, identifier)
        )
        color_defines.append(
            "#define %s (GColor8){.argb=%s}"
            % (identifier.ljust(color_define_maxlen), value_identifier)
        )

    file_content = """/* SPDX-FileCopyrightText: 2024 Google LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

// @%s
// THIS FILE HAS BEEN GENERATED, PLEASE DON'T MODIFY ITS CONTENT MANUALLY
// USE <TINTIN_ROOT>/tools/%s TO MAKE CHANGES

//! @addtogroup Graphics
//! @{

//! @addtogroup GraphicsTypes
//! @{

//! Convert RGBA to GColor.
//! @param red Red value from 0 - 255
//! @param green Green value from 0 - 255
//! @param blue Blue value from 0 - 255
//! @param alpha Alpha value from 0 - 255
//! @return GColor created from the RGBA values
#define GColorFromRGBA(red, green, blue, alpha) ((GColor8){ \\
  .b = (uint8_t)(blue) >> 6, \\
  .g = (uint8_t)(green) >> 6, \\
  .r = (uint8_t)(red) >> 6, \\
  .a = (uint8_t)(alpha) >> 6, \\
  })

//! Convert RGB to GColor.
//! @param red Red value from 0 - 255
//! @param green Green value from 0 - 255
//! @param blue Blue value from 0 - 255
//! @return GColor created from the RGB values
#define GColorFromRGB(red, green, blue) \\
  GColorFromRGBA(red, green, blue, 255)

//! Convert hex integer to GColor.
//! @param v Integer hex value (e.g. 0x64ff46)
//! @return GColor created from the hex value
#define GColorFromHEX(v) GColorFromRGB(((v) >> 16) & 0xff, ((v) >> 8) & 0xff, ((v) & 0xff))

//! @addtogroup ColorDefinitions Color Definitions
//! A list of all of the named colors available with links to the color map on the Pebble Developer website.
//! @{

// 8bit color values of all natively supported colors
%s

// GColor values of all natively supported colors
%s

// Additional 8bit color values
#define GColorClearARGB8 ((uint8_t)0b00000000)

// Additional GColor values
#define GColorClear ((GColor8){.argb=GColorClearARGB8})

//! @} // group ColorDefinitions

//! @} // group GraphicsTypes

//! @} // group Graphics

""" % (
        "generated",
        basename(__file__),
        "\n".join(color_value_defines),
        "\n".join(color_defines),
    )

    return file_content


def render_html(colors):
    """
    renders a HTML file for debugging purposes. Not even close to the awesome Pebble color picker(tm)
    """
    html = '<table style="border-spacing:0"><thead>'
    html += '<tr><th colspan="4">Closest</th><th colspan="7">Actual Color</th></tr>'
    html += "<tr>%s</tr>" % "".join(
        [
            "<th>%s</th>" % s
            for s in [
                "r",
                "g",
                "b",
                "color",
                "color",
                "&Delta;",
                "r",
                "g",
                "b",
                "c code",
                "name",
                "identifier",
            ]
        ]
    )
    html += "</thead><tbody>"
    for c in colors:

        def rgb(c):
            return "<td>%d</td><td>%d</td><td>%d</td>" % (c["r"], c["g"], c["b"])

        def color(c):
            return '<td style="background-color:rgb(%d,%d,%d); width:4em;"></td>' % (
                c["r"],
                c["g"],
                c["b"],
            )

        def c_code(c):
            return "(GColor){{.rgba=0b{:02b}{:02b}{:02b}11}}".format(
                c["r"] / 64, c["g"] / 64, c["b"] / 64
            )

        html += "<tr>"
        html += rgb(c["closest"])
        html += color(c["closest"])
        html += color(c)
        html += "<td><strong>%d</strong></td>" % c["dist"]
        html += rgb(c)
        html += "<td><pre>" + c_code(c) + "</pre></td>"

        html += "<td>"
        if "url" in c:
            html += '<a href="%s">%s</a>' % (c["url"], c["name"])
        else:
            html += c["name"]
        html += "<td>%s</td>" % c["identifier"]

        html += "</tr>"

    html += "</tbody></table>"

    return html


def render_json(colors):
    """
    JSON as being used by the awesome Pebble color picker(tm)
    """
    obj = {}
    for c in colors:
        color_attr = "#%0.2X%0.2X%0.2X" % (c["r"], c["g"], c["b"])
        obj[color_attr] = c

    return json.dumps(obj, indent=2)


def render_svg(colors=None):
    """
    renders all 64 colors, ignores provided colors (only there to share same signature with other functions
    """

    polygons = []

    dd = 300
    for r in range(4):
        yy = r * dd
        xx = -742 - dd
        for g in range(4):
            for b in range(4):
                xx += dd
                points = [
                    (850, 75),
                    (958, 137.5),
                    (958, 262.5),
                    (850, 325),
                    (742, 262.6),
                    (742, 137.5),
                ]
                points = [(p[0] + xx, p[1] + yy) for p in points]

                points_attr = " ".join(["%f,%f" % (p[0], p[1]) for p in points])
                color_attr = "#%0.2X%0.2X%0.2X" % (r * 85, g * 85, b * 85)
                polygon = (
                    """<polygon fill="%s" stroke="black" stroke-width=".1" points="%s" />"""
                    % (color_attr, points_attr)
                )
                polygons.append(polygon)

    xml = """<?xml version="1.0" standalone="no"?>
    <!DOCTYPE svg PUBLIC "-//W3C//DTD SVG 1.1//EN"
      "http://www.w3.org/Graphics/SVG/1.1/DTD/svg11.dtd">
    <svg viewBox="0 0 4000 4000"
         xmlns="http://www.w3.org/2000/svg" version="1.1">
    %s
    </svg>""" % "\n".join(polygons)

    return xml


if __name__ == "__main__":
    # e.g. --download_wikipedia --json snowy_colors.json --header ../src/fw/applib/graphics/gcolor_definitions.h
    parser = argparse.ArgumentParser(
        description="Generate various files that contain Snowy's 64 colors"
    )
    parser.add_argument(
        "--download_wikipedia",
        action="store_true",
        help="loads and caches colors from wikipedia",
    )
    parser.add_argument(
        "--download_colorlovers",
        action="store_true",
        help="loads and caches colors from colourlovers.com",
    )
    parser.add_argument("--html", help="generates HTML file for test purposes")
    parser.add_argument(
        "--json", help="generates JSON used by awesome Pebble color picker(tm)"
    )
    parser.add_argument(
        "--header", help="generates C header file that can replace color_definitions.h"
    )
    parser.add_argument(
        "--svg", help="generates SVG file with hexagons of all supported colors"
    )

    if len(sys.argv) <= 1:
        parser.print_usage()
        sys.exit(1)

    args = parser.parse_args()

    if args.download_wikipedia:
        download_and_parse_colors_from_wikipedia()
    if args.download_colorlovers:
        download_all_colors_from_color_lovers()

    colors = all_colors_with_names()
    for k, v in {
        "html": render_html,
        "json": render_json,
        "header": render_header,
        "svg": render_svg,
    }.items():
        file_name = getattr(args, k)
        if file_name is not None:
            with open(file_name, "w") as f:
                f.write(v(colors))
