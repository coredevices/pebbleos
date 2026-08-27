# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0


import argparse
import errno
import os
import zipfile
from shutil import rmtree


class MissingFileException(Exception):
    pass


class DuplicatePackageFileException(Exception):
    pass


def _calculate_file_size(path):
    return os.stat(path).st_size


def _calculate_crc(path):
    pass


class PebblePackage:
    def __init__(self, package_filename):
        self.package_filename = package_filename
        self.package_files = {}

    def add_file(self, name, file_path):
        if not os.path.exists(file_path):
            raise MissingFileException(f"The file '{file_path}' does not exist")
        if name in self.package_files and self.package_files.get(name) != file_path:
            raise DuplicatePackageFileException(
                f"The file '{file_path}' cannot be added to the package "
                f"because `{self.package_files.get(name)}` has already been assigned to `{name}`"
            )
        else:
            self.package_files[name] = file_path

    def pack(self, package_path="."):
        with zipfile.ZipFile(
            os.path.join(package_path, self.package_filename), "w"
        ) as zip_file:
            for filename, file_path in self.package_files.items():
                zip_file.write(file_path, filename)
            zip_file.comment = type(self).__name__.encode("utf-8")

    def unpack(self, package_path=""):
        try:
            rmtree(package_path)
        except OSError as e:
            if e.errno != errno.ENOENT:
                raise
        with zipfile.ZipFile(self.package_filename, "r") as zip_file:
            zip_file.extractall(package_path)


class LibraryPackage(PebblePackage):
    def __init__(self, package_filename="dist.zip"):
        super().__init__(package_filename)

    def add_files(self, includes, binaries, resources, js):
        for include, include_path in includes.items():
            self.add_file(os.path.join("include", include), include_path)
        for binary, binary_path in binaries.items():
            self.add_file(os.path.join("binaries", binary), binary_path)
        for resource, resource_path in resources.items():
            self.add_file(os.path.join("resources", resource), resource_path)
        for js_file, js_file_path in js.items():
            self.add_file(os.path.join("js", js_file), js_file_path)

    def unpack(self, package_path="dist"):
        super().unpack(package_path)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Manage Pebble packages")
    parser.add_argument("command", type=str, help="Command to use")
    parser.add_argument("filename", type=str, help="Path to your Pebble package")
    args = parser.parse_args()

    with zipfile.ZipFile(args.filename, "r") as package:
        cls = globals()[package.comment](args.filename)
    getattr(cls, args.command)()
