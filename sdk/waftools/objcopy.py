# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

# FIXME: For some reason this doesn't work with multiple rules with the same input extension.
# from waflib import TaskGen
# TaskGen.declare_chain(name='hex', rule='${OBJCOPY} -O ihex ${SRC} ${TGT}', ext_in='.elf', ext_out='.hex')
# TaskGen.declare_chain(name='bin', rule='${OBJCOPY} -O binary ${SRC} ${TGT}', ext_in='.elf', ext_out='.bin')


def objcopy(task, mode, extra_args=None):
    cmd = "arm-none-eabi-objcopy -S -R .stack -R .priv_bss -R .bss -R .retained "

    if hasattr(task.generator, "extra_args"):
        cmd += f"{task.generator.extra_args} "

    if extra_args is not None:
        cmd += f"{extra_args} "

    cmd += f'-O {mode} "{task.inputs[0].abspath()}" "{task.outputs[0].abspath()}"'
    return task.exec_command(cmd)


def objcopy_fill_bss(task, mode):
    return task.exec_command(
        f"arm-none-eabi-objcopy -O {mode} -j .text -j .data "
        f'-j .bss --set-section-flags .bss=alloc,load,contents "{task.inputs[0].abspath()}" "{task.outputs[0].abspath()}"'
    )


def objcopy_hex(task):
    return objcopy(task, "ihex")


def objcopy_bin(task):
    return objcopy(task, "binary")
