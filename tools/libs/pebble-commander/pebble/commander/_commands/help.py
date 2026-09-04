# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

import inspect

from .. import PebbleCommander, exceptions


def trim_docstring(var):
    return inspect.getdoc(var) or ""


def get_help_short(cmdr, cmd_name, help_output=None):
    """
    cmd_name is the command's name.
    help_output is the raw output of the `!help` command.
    """
    output = None
    func = cmdr.get_command(cmd_name)
    if func:  # Host command
        # cmdstr is the actual function name
        cmdstr = func.name
        spec = inspect.getargspec(func)

        if len(spec.args) > 1:
            maxargs = len(spec.args) - 1
            if spec.defaults is None:
                cmdstr += f" {{{maxargs:d} args}}"
            else:
                minargs = maxargs - len(spec.defaults)
                cmdstr += f" {{{minargs:d}~{maxargs:d} args}}"

        if func.__doc__ is not None:
            output = f"{cmdstr:<30} - {trim_docstring(func).splitlines()[0]}"
        else:
            output = cmdstr
    else:  # Prompt command
        if cmd_name[0] == "!":  # Strip the bang if it's there
            cmd_name = cmd_name[1:]

        # Get the output if it wasn't provided
        if help_output is None:
            help_output = cmdr.send_prompt_command("help")

        for prompt_cmd in help_output[1:]:
            # Match, even with argument count provided
            if prompt_cmd == cmd_name or prompt_cmd.startswith(cmd_name + " "):
                # Output should be the full argument string with the bang
                output = "!" + prompt_cmd
                break

    return output


def help_arginfo_nodefault(arg):
    return f"{arg.upper()}"


def help_arginfo_default(arg, dflt):
    return f"[{arg.upper()} (default: {dflt!s})]"


def get_help_long(cmdr, cmd_name):
    output = ""

    func = cmdr.get_command(cmd_name)

    if func:
        spec = inspect.getargspec(func)
        specstr = []
        for i, arg in enumerate(spec.args[1:]):
            if spec.defaults is not None:
                minargs = len(spec.args[1:]) - len(spec.defaults)
                if i >= minargs:
                    specstr.append(
                        help_arginfo_default(arg, spec.defaults[i - minargs])
                    )
                else:
                    specstr.append(help_arginfo_nodefault(arg))
            else:
                specstr.append(help_arginfo_nodefault(arg))

        specstr = " ".join(specstr)
        cmdstr = func.name + " " + specstr
        if func.__doc__ is None:
            output = f"{cmdstr}\n\nNo help available."
        else:
            output = f"{cmdstr} - {trim_docstring(func)}"
    else:  # Prompt command
        cmdstr = get_help_short(cmdr, cmd_name)
        if cmdstr is None:
            output = None
        else:
            output = f"{cmdstr}\n\nNo help available, due to being a prompt command."
    return output


@PebbleCommander.command()
def help(cmdr, cmd=None):
    """Show help.

    You're lookin' at it, dummy!
    """
    out = []
    if cmd is not None:
        helpstr = get_help_long(cmdr, cmd)
        if helpstr is None:
            raise exceptions.ParameterError(f"No command '{cmd}' found.")
        out.append(helpstr)
    else:  # List commands
        out.append("===Host commands===")
        # Bonus, this list is sorted for us already
        for cmd_name in dir(cmdr):
            if cmdr.get_command(cmd_name):
                out.append(get_help_short(cmdr, cmd_name))

        out.append("\n===Prompt commands===")
        ret = cmdr.send_prompt_command("help")
        if ret[0] != "Available Commands:":
            raise exceptions.PromptResponseError("'help' prompt command output invalid")
        for cmd_name in ret[1:]:
            out.append(get_help_short(cmdr, "!" + cmd_name, ret))
    return out
