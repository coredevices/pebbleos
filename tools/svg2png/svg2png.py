# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

import argparse
import glob
import logging
import os
import shutil
import subprocess
import sys

from generate_pdcs import pdc_gen

logger = logging.getLogger(__name__)


def find_pdc2png():
    if os.path.exists(
        os.path.abspath(
            os.path.join(os.path.dirname(__file__), "../../build/pdc2png/pdc2png")
        )
    ):
        pdc2png = os.path.abspath(
            os.path.join(os.path.dirname(__file__), "../../build/pdc2png/pdc2png")
        )
    elif os.path.exists(
        os.path.abspath(
            os.path.join(os.path.dirname(__file__), "../../build/tools/pdc2png")
        )
    ):
        pdc2png = os.path.abspath(
            os.path.join(os.path.dirname(__file__), "../../build/tools/pdc2png")
        )
    elif os.path.exists(os.path.abspath("./pdc2png")):
        pdc2png = os.path.abspath("./pdc2png")
    else:
        pdc2png = None
        logger.warning("Can't find pdc2png")

    return pdc2png


def set_path_to_svg2pdc():
    root_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir))
    if os.path.exists(os.path.join(root_dir, "generate_pdcs/pdc_gen.py")):
        sys.path.insert(0, root_dir)
    elif os.path.exists(os.path.abspath("./generate_pdcs/pdc_gen.py")):
        sys.path.insert(0, os.path.abspath("."))
    else:
        logger.warning("Can't find generate_pdcs/pdc_gen.py")


def log_exception(filename, exc_type, exc_value, exc_traceback):
    import traceback

    lines = traceback.format_exception(exc_type, exc_value, exc_traceback)
    s = f"Exception while processing {filename}\n"
    s += "".join(lines)
    logger.error(s)


# process_svg_files finds all the SVGs in the root and converts them to individual PDCs and then converts them to PNGs.
# It treats all the subdirectories as sequences, creating one PDC from all the SVGs in that directory and then generates
# PDCs from all the frames in the sequence
def process_svg_files(path):
    set_path_to_svg2pdc()

    files = glob.glob(path + "/*.svg")
    dirs = glob.glob(path + "/*/")

    error_files = []
    for f in files:
        try:
            error_files += pdc_gen.create_pdc_from_path(
                f,
                None,
                viewbox_size=(0, 0),
                verbose=True,
                duration=0,
                play_count=0,
                precise=True,
                raise_error=True,
            )
        except Exception:  # noqa: BLE001
            exc_type, exc_value, exc_traceback = sys.exc_info()
            log_exception(f, exc_type, exc_value, exc_traceback)

    for d in dirs:
        try:
            error_files += pdc_gen.create_pdc_from_path(
                d,
                None,
                viewbox_size=(0, 0),
                verbose=True,
                duration=33,
                play_count=1,
                precise=True,
                raise_error=True,
            )
        except Exception:  # noqa: BLE001
            exc_type, exc_value, exc_traceback = sys.exc_info()
            log_exception(f, exc_type, exc_value, exc_traceback)

    return error_files


# convert all PDC files to PNGs
def process_pdc_files(path):
    pdc2png = find_pdc2png()
    if pdc2png is None:
        return

    pdc_files = glob.glob(path + "/*.pdc") + glob.glob(path + "/*/*.pdc")
    if pdc_files:
        try:
            p = subprocess.Popen(
                [pdc2png] + pdc_files,
                cwd=os.path.abspath(path),
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )
            stdout, stderr = p.communicate()
            if stdout:
                logger.info(stdout)
            if stderr:
                logger.error(stderr)
        except OSError:
            logger.warning("Can't find pdc2png executable")

        for f in pdc_files:
            os.remove(f)
    else:
        print(logger.info("No .pdc files found in " + path))


# If any files contain invalid points, the images are moved to the 'failed' subdirectory to highlight this for the
# designers
def copy_error_files_to_failed_subdir(path, error_files):
    if len(error_files) == 0:
        return
    fail_dir = os.path.join(path, "failed")
    if not os.path.exists(fail_dir):
        os.makedirs(fail_dir)
    for f in error_files:
        base = os.path.basename(f)
        dir_name = os.path.dirname(f)
        png_base = ".".join(base.split(".")[:-1]) + ".png"
        png_error = os.path.join(dir_name, png_base)
        png_copy = os.path.join(fail_dir, png_base)
        logger.debug(png_error + " => " + png_copy)
        try:
            shutil.copy(png_error, png_copy)
        except OSError:
            logger.error(f"Failed to copy {f} to failed directory")


def main(path=None):
    # detect whether the script is running within svg2png.app
    is_app = os.path.dirname(os.path.abspath("../")).split("/")[-1] == "svg2png.app"

    if is_app:
        # if this is running as the app, the SVG files will be placed in the same directory as 'svg2png.app'
        path = "../../../"
        # output errors to log file when running as an app
        logging.basicConfig(
            format="%(asctime)s - %(levelname)s - %(message)s",
            filename=os.path.join(path, "log.log"),
            level=logging.DEBUG,
        )
    else:
        logging.basicConfig(
            format="%(asctime)s - %(levelname)s - %(message)s", level=logging.DEBUG
        )

    if path:
        path = os.path.abspath(path)
        error_files = process_svg_files(path)
        process_pdc_files(path)
        copy_error_files_to_failed_subdir(path, error_files)
    else:
        logger.warning("No path specified")


if __name__ == "__main__":
    # path argument is provided so that this can be used as the standalone script (if no path is provided nothing will
    # happen when invoked as script
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "path",
        type=str,
        nargs="?",
        help="Path to svg file or directory (with multiple svg files)",
    )
    args = parser.parse_args()
    main(args.path)
