# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

import argparse
import json
import os
import shutil

import watch_templates as temps


class WatchGenerator:
    def __init__(self):
        self._static_defs = []
        self._init_func = []
        self._update_func = []
        self.watch_name = None
        self.version = None
        self.company_name = None
        self.visible_name = None

    def h_file_name(self):
        return "pebble_gen_header.h"

    def c_file_name(self):
        return self.watch_name + ".c"

    def defs_file_name(self):
        return "pebble_gen_defs.h"

    def generate_c_code(self):
        fs = temps.FileScope
        res_strings = []
        res_strings.append(fs.includes.format(header_name=self.h_file_name()))
        res_strings.append("\n".join(self._static_defs))
        res_strings.append(
            fs.update_function.format(
                watch_name=self.watch_name, update_content="\n".join(self._update_func)
            )
        )
        res_strings.append(
            fs.init_function.format(
                watch_name=self.watch_name, init_content="\n".join(self._init_func)
            )
        )

        return "\n".join(res_strings)

    def generate_h_code(self):
        return temps.FileScope.header.format(
            watch_name=self.watch_name, app_version=self.version
        )

    def generate_defs_file(self):
        return temps.FileScope.defs_file.format(
            watch_name=self.watch_name,
            visible_name=self.visible_name,
            company_name=self.company_name,
        )

    def load_config(self, conf):
        self.watch_name = conf["name"]
        self.version = conf["friendlyVersion"]
        self.company_name = conf["company-name"]
        self.visible_name = conf["visible-name"]

        for elem in conf["elements"]:
            t = elem["type"]
            if t == "static-image":
                self.parse_static_image(elem)
            elif t == "time-text":
                self.parse_time_text(elem)
            elif t == "hand":
                self.parse_hand(elem)
            else:
                raise "Unknown element type: " + t

    def parse_static_image(self, elem):
        si = temps.StaticImage
        self._static_defs.append(si.static_defs.format(name=elem["name"]))
        self._init_func.append(
            si.init_lines.format(
                res_def_name=elem["image"]["def-name"], name=elem["name"]
            )
        )

    def parse_date_format(self, fmt_list):
        ### parses a json object list into a c strftime format string and required buffer size
        buffer_size = 1
        fmt_string = ""
        for fmt in fmt_list:
            # TODO: implement the other operations for strftime
            t = fmt["type"]
            if t == "string":
                fmt_string += fmt.val
                buffer_size += len(fmt.val)
            elif t == "day-name-abbreviated":
                fmt_string += "%a"
                buffer_size += 4
            elif t == "day-of-month":
                fmt_string += "%d"
                buffer_size += 2
            else:
                raise "Unknown date format type: " + t
        return buffer_size, fmt_string

    def parse_color(self, color_string):
        if color_string == "black":
            return "GColorBlack"
        elif color_string == "white":
            return "GColorWhite"
        elif color_string == "transparent":
            return "GColorTrans"
        else:
            raise "Unknown color: " + color_string

    def parse_font_info(self, font_obj):
        f = font_obj["font"]
        if f == "gothic":
            bold = "modifiers" in font_obj and "bold" in font_obj["modifiers"]
            size = font_obj["size"]
            return "graphics_text_get_system_font({size}, GFontStyle{style})".format(
                size=size, style=("Bold" if bold else "Regular")
            )
        else:
            raise "Unknown font: " + f

    def parse_time_text(self, elem):
        tt = temps.TimeText

        buffer_len, date_fmt = self.parse_date_format(elem["format"])
        color = self.parse_color(elem["color"])
        bg_color = self.parse_color(elem["background-color"])
        font_func = self.parse_font_info(elem["font"])

        self._static_defs.append(
            tt.static_defs.format(name=elem["name"], buffer_length=buffer_len)
        )
        self._init_func.append(
            tt.init_lines.format(
                name=elem["name"],
                x=elem["position"][0],
                y=elem["position"][1],
                w=elem["size"][0],
                h=elem["size"][1],
                color=color,
                background_color=bg_color,
                font_func=font_func,
            )
        )
        self._update_func.append(
            tt.update_lines.format(name=elem["name"], date_format_string=date_fmt)
        )

    def parse_time_element(self, te):
        if te == "hour":
            return "t.tm_hour / 12"
        elif te == "hour24":
            return "t.tm_hour / 24"
        elif te == "minute":
            return "t.tm_min / 60"
        elif te == "second":
            return "t.tm_sec / 60"
        else:
            raise "Unknown time element: " + te

    def parse_hand(self, elem):
        h = temps.Hand

        ratio = self.parse_time_element(elem["time-element"])

        self._static_defs.append(h.static_defs.format(name=elem["name"]))
        self._init_func.append(
            h.init_lines.format(
                res_def_name=elem["trans-image"]["def-name"],
                name=elem["name"],
                pivot_x=elem["pivot"][0],
                pivot_y=elem["pivot"][1],
            )
        )
        self._update_func.append(
            h.update_lines.format(name=elem["name"], angle_ratio=ratio)
        )


def generate(watch_config, src_output_dir):
    with open(watch_config, "r") as conf:
        config_data = json.loads(conf.read())

        gen = WatchGenerator()

        gen.load_config(config_data)

        if not os.path.isdir(src_output_dir):
            os.makedirs(src_output_dir)

        with open(os.path.join(src_output_dir, gen.c_file_name()), "w") as output:
            output.write(gen.generate_c_code())

        with open(os.path.join(src_output_dir, gen.h_file_name()), "w") as output:
            output.write(gen.generate_h_code())

        with open(os.path.join(src_output_dir, gen.defs_file_name()), "w") as output:
            output.write(gen.generate_defs_file())


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("watch_config")
    parser.add_argument("src_output_dir")

    args = parser.parse_args()
    generate(args.watch_config, args.src_output_dir)

    shutil.copy(
        "watch_boilerplate.template.c",
        os.path.join(args.src_output_dir, "watch_boilerplate.template.c"),
    )


if __name__ == "__main__":
    main()
