import imghdr
import os
import subprocess
import sys
import tempfile

# gif2apng
from exceptions import *

GIF2APNG_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "../bin/gif2apng"
)
COLORMAP_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "colormap.txt")


def read_gif(obj):
    data = obj.read()
    if imghdr.what(None, data) != "gif":
        raise Gif2ApngFormatError(f"{path} is not a valid GIF data")

    return data


def convert_to_apng(gif):
    # Write data to temporary file
    gif_file = tempfile.NamedTemporaryFile(delete=False)
    gif_file.write(gif)
    gif_file.close()

    # Map onto Pebble colors
    mod_file = tempfile.NamedTemporaryFile(delete=False)
    mod_file.close()
    p = subprocess.Popen(
        [
            "gifsicle",
            "--colors",
            "64",
            "--use-colormap",
            COLORMAP_PATH,
            "-O1",
            "-o",
            mod_file.name,
            gif_file.name,
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    out, err = p.communicate()
    # Deal with https://github.com/kohler/gifsicle/issues/28
    # Which still exists in some of the packages out there
    if p.returncode not in [0, 1]:
        print(err, file=sys.stderr)
        raise Gif2ApngError(p.returncode)

    # Convert to APNG
    apng_file = tempfile.NamedTemporaryFile(delete=False)
    apng_file.close()
    p = subprocess.Popen(
        [GIF2APNG_PATH, "-z0", mod_file.name, apng_file.name],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    out, err = p.communicate()
    if p.returncode != 0:
        print(err, file=sys.stderr)
        raise Gif2ApngError(p.returncode)

    with open(apng_file.name) as f:
        apng_data = f.read()

    os.unlink(gif_file.name)
    os.unlink(mod_file.name)
    os.unlink(apng_file.name)

    return apng_data
