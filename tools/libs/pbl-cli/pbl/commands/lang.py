# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0

"""Translation catalogs and the language packs built from them."""

import json
import os
import string
import subprocess
import sys
import tempfile
from collections import OrderedDict

from pbl.command import PblCommand

LANG_MAP = "lang_map.json"
CATALOG = "tintin.po"

# A new language's lang_map.json. Only the strings entry is filled in; the
# fonts are given files by hand once the language needs extended glyphs.
LANG_MAP_TEMPLATE = string.Template(
    json.dumps(
        {
            "strings": {"lang": "${lang}", "name": "STRINGS", "file": CATALOG},
            "fonts": [
                {"name": name, "file": ""}
                for name in (
                    "GOTHIC_14_EXTENDED",
                    "GOTHIC_14_BOLD_EXTENDED",
                    "GOTHIC_18_EXTENDED",
                    "GOTHIC_18_BOLD_EXTENDED",
                    "GOTHIC_24_EXTENDED",
                    "GOTHIC_24_BOLD_EXTENDED",
                    "GOTHIC_28_EXTENDED",
                    "GOTHIC_28_BOLD_EXTENDED",
                    "BITHAM_18_LIGHT_SUBSET_EXTENDED",
                    "BITHAM_30_BLACK_EXTENDED",
                    "BITHAM_34_LIGHT_SUBSET_EXTENDED",
                    "BITHAM_34_MEDIUM_NUMBERS_EXTENDED",
                    "BITHAM_42_BOLD_EXTENDED",
                    "BITHAM_42_LIGHT_EXTENDED",
                    "BITHAM_42_MEDIUM_NUMBERS_EXTENDED",
                    "ROBOTO_CONDENSED_21_EXTENDED",
                    "ROBOTO_BOLD_SUBSET_49_EXTENDED",
                    "DROID_SERIF_28_BOLD_EXTENDED",
                )
            ],
            "images": [],
        },
        indent=4,
    )
)


class _LangCommand(PblCommand):
    group = "i18n"

    def lang_dir(self, lang=None):
        root = self.workspace.lang_dir
        return os.path.join(root, lang) if lang else root

    def lang_build_dir(self, build, lang):
        """Where a language's intermediates go, mirroring the source tree."""
        relative = os.path.relpath(self.workspace.lang_dir, self.topdir)
        return build.join(relative, lang)

    def _import_generators(self):
        """Put tools/ on the path so the resource generators the normal
        resource build uses resolve. It goes ahead of the workspace root so
        'resources' binds to tools/resources, not the firmware's resources/."""
        tools = os.path.join(self.topdir, "tools")
        if tools not in sys.path:
            sys.path.insert(0, tools)

    def pack(self, build, lang):
        """Build one language's pbpack from its catalog and fonts."""
        self._import_generators()

        import pbpack
        from resources.resource_map.resource_generator_font import FontResourceGenerator

        source = self.lang_dir(lang)
        output = self.lang_build_dir(build, lang)
        os.makedirs(output, exist_ok=True)

        with open(os.path.join(source, LANG_MAP)) as f:
            resource_map = json.load(f)

        resources = OrderedDict()

        # The strings entry, if any, holds the compiled .mo catalog.
        strings = resource_map["strings"]
        codepoints = None
        if strings["file"] == "":
            resources[strings["name"]] = b""
        else:
            po = os.path.join(source, strings["file"])
            if subprocess.check_output(["msgattrib", "--untranslated", po]):
                self.wrn(f"{po} still contains untranslated strings")

            mo = os.path.join(output, f"{strings['file']}.{strings['name']}.mo")
            self.check_cmd(["msgfmt", "-c", "-v", "-o", mo, po], "msgfmt failed")
            self.inf("created", mo)
            with open(mo, "rb") as f:
                resources[strings["name"]] = f.read()

            # Fonts default to covering exactly what the translated UI needs.
            codepoints = os.path.join(output, "codepoints.json")
            self.check_cmd(
                [
                    sys.executable,
                    self.script("tools", "generate_codepoint_requirements.py"),
                    po,
                    f"--output={codepoints}",
                ],
                "codepoint extraction failed",
            )

        for entry in resource_map["fonts"]:
            name = entry["name"]
            if "alias" in entry:
                # Aliases reuse another resource's bytes; the pbpack dedups.
                self.inf(f"aliasing {entry['alias']} to {name}")
                resources[name] = resources[entry["alias"]]
            elif entry["file"] == "":
                self.inf(f"building empty resource {name}")
                resources[name] = b""
            else:
                self.inf(f"building font resource {name}")
                # Make the entry look like a normal resource_map.json font.
                entry["type"] = "font"
                definition = FontResourceGenerator.font_definition_from_dict(
                    build.config, entry
                )
                if definition.character_list is None:
                    definition.character_list = codepoints
                else:
                    definition.character_list = os.path.join(
                        source, definition.character_list
                    )
                resources[name] = FontResourceGenerator.build_font_data(
                    os.path.join(source, entry["file"]), definition
                )

        pack = pbpack.ResourcePack(False)
        for resource in resources.values():
            pack.add_resource(resource)

        path = os.path.join(output, f"{lang}.pbl")
        with open(path, "wb+") as f:
            pack.serialize(f)
        self.inf("created language pack", path)


class MakeLang(_LangCommand):
    def __init__(self):
        super().__init__(
            "make_lang",
            "Create or update a language's translation catalog",
            "Seed a new language from the build's .pot, or merge newly "
            "extracted strings into an existing catalog.",
        )

    def do_add_parser(self, parser_adder):
        parser = self.add_subparser(parser_adder)
        parser.add_argument(
            "--lang", default="en_US", help="Language isocode (default: %(default)s)"
        )
        return parser

    def do_run(self, args, unknown):
        import polib

        build = self.build_dir()
        pot = build.join(f"{build.project}.pot")
        if not os.path.exists(pot):
            self.die(f"{pot} does not exist -- run 'pbl build' first")

        lang = args.lang
        source = self.lang_dir(lang)
        catalog = os.path.join(source, CATALOG)
        msginit = ["msginit", "-l", lang, "--no-translator", "-i", pot]

        if not os.path.exists(source):
            os.mkdir(source)
            with open(os.path.join(source, LANG_MAP), "w") as f:
                f.write(LANG_MAP_TEMPLATE.substitute(lang=lang) + "\n")

        if not os.path.exists(catalog):
            return self.run_cmd(msginit + ["-o", catalog])

        # Keep the existing header, regenerate, then merge the new strings in.
        header = str(polib.pofile(catalog).metadata_as_entry())
        with open(catalog, "w") as f:
            f.write(header)

        with tempfile.NamedTemporaryFile(suffix=".po") as regenerated:
            rc = self.run_cmd(msginit + ["-o", regenerated.name])
            if rc:
                return rc
            return self.run_cmd(
                ["msgmerge", f"--lang={lang}", "--update", catalog, regenerated.name]
            )


class PackLang(_LangCommand):
    def __init__(self):
        super().__init__("pack_lang", "Build one language's language pack")

    def do_add_parser(self, parser_adder):
        parser = self.add_subparser(parser_adder)
        parser.add_argument(
            "--lang", default="en_US", help="Language isocode (default: %(default)s)"
        )
        return parser

    def do_run(self, args, unknown):
        self.pack(self.build_dir(), args.lang)


class PackAllLangs(_LangCommand):
    def __init__(self):
        super().__init__("pack_all_langs", "Build every language's language pack")

    def do_add_parser(self, parser_adder):
        return self.add_subparser(parser_adder)

    def do_run(self, args, unknown):
        build = self.build_dir()
        root = self.lang_dir()
        for lang in sorted(next(os.walk(root))[1]):
            self.pack(build, lang)
