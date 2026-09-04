# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0


rm -r src

pushd "tools/generate_watchface"
python generate_watchface.py ../../watch_generators/gen_analog.json ../../src/
popd
./pbl configure --board $BOARD
./pbl build
