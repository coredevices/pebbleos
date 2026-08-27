# `release-hw-android-obelix`

This directory is the source of truth for the release tests exercised against PebbleOS firmware on
real hardware. PebbleOS owns the suite order, watch behavior, and watch-side assertions. CoreApp
owns reusable phone capabilities and its UI selectors under `qa/release/` in the CoreApp repo.
Unicorn supplies the hardware, artifact staging, Pebble Protocol sidecar, and result collection.

`suites/android-obelix.json` has the stable suite ID `release-hw-android-obelix` and deliberately
references CoreApp by logical capability ID. It must not contain CoreApp selectors. Watch flows are
small Maestro wrappers because Maestro provides the
JavaScript HTTP bridge used to drive Unicorn's watch sidecar.

To validate references using a checkout of Unicorn and CoreApp:

```shell
python3 ../unicorn/qa-release/bin/compose-release-suite.py \
  --suite qa/release/suites/android-obelix.json \
  --capabilities ../CoreApp/qa/release/capabilities.json --check
```

The production runner checks out this directory at the firmware artifact's exact commit and checks
out CoreApp's capabilities at the APK artifact's exact commit. If either older commit predates the
catalog, Unicorn uses its legacy monolithic flow during the migration window.
