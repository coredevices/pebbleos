# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

from waflib import Configure, Task, TaskGen

GETTEXT_KEYWORDS = [
    "i18n_noop",
    "i18n_get",
    "i18n_get_with_buffer",
    "sys_i18n_get_with_buffer",
    "i18n_ctx_noop:1c,2",
    "i18n_ctx_get:1c,2",
    "i18n_ctx_get_with_buffer:1c,2",
]


def configure(conf):
    conf.find_program(
        "xgettext",
        exts="",
        errmsg="""
=======================================================================
`gettext` might not be installed properly.
 - If using a Mac, try running `brew install gettext; brew link gettext --force`
 - If using Linux, and you fix this error, please insert solution here
=======================================================================""",
    )
    conf.find_program("msgcat")

    # gettext >=0.22 shells out to git for a reproducible POT-Creation-Date,
    # which warns per-source-file on out-of-tree build paths. Skip it when the
    # installed xgettext supports --no-git.
    help_text = conf.cmd_and_log(conf.env.XGETTEXT + ["--help"])
    conf.env.XGETTEXT_NO_GIT = ["--no-git"] if "--no-git" in help_text else []


class xgettext(Task.Task):
    run_str = (
        "${XGETTEXT} ${XGETTEXT_NO_GIT} -c/ -k --from-code=UTF-8 --language=C "
        + " ".join("--keyword=" + word for word in GETTEXT_KEYWORDS)
        + " -o ${TGT[0].abspath()} ${SRC}"
    )


class msgcat(Task.Task):
    run_str = "${MSGCAT} ${SRC} -o ${TGT}"


@TaskGen.before("process_source")
@TaskGen.feature("gettext")
def do_gettext(self):
    sources = [
        src for src in self.to_nodes(self.source) if src.suffix() not in (".s", ".S")
    ]

    # There is a convenient to_nodes method for sources (that already exist),
    # but no equivalent for targets (files which don't exist yet).
    if isinstance(self.target, str):
        target = self.path.find_or_declare(self.target)
    else:
        target = self.target
    self.create_task("xgettext", src=sources, tgt=target)
    # Bypass the execution of process_source
    self.source = []


@TaskGen.before("process_source")
@TaskGen.feature("msgcat")
def do_msgcat(self):
    if isinstance(self.target, str):
        target = self.path.find_or_declare(self.target)
    else:
        target = self.target
    self.create_task("msgcat", src=self.to_nodes(self.source), tgt=target)
    # Bypass the execution of process_source
    self.source = []


@Configure.conf
def gettext(self, *args, **kwargs):
    kwargs["features"] = "gettext"
    return self(*args, **kwargs)


@Configure.conf
def msgcat(self, *args, **kwargs):  # noqa: F811
    kwargs["features"] = "msgcat"
    return self(*args, **kwargs)
