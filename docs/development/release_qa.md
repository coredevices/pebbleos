# `release-hw-android-obelix`

`release-hw-android-obelix` installs a shipping-style dual-slot firmware artifact on a real watch,
pairs it with a real phone, upgrades through CoreApp's production sideload path, and then exercises
the firmware over Pebble Protocol. The executable catalog lives in
`qa/release/suites/android-obelix.json` (whose stable suite ID is
`release-hw-android-obelix`); this page describes exactly what it covers and where each
piece is maintained.

## Ownership

| Concern | Owner | Location |
|---|---|---|
| Ordered test catalog and required watch behavior | PebbleOS | `qa/release/suites/` |
| Watch input and assertions | PebbleOS | `qa/release/watch/` |
| Phone navigation and UI selectors | CoreApp | `qa/release/capabilities.json` and `qa/release/capabilities/` |
| Artifact staging, physical devices, sidecar, logs and JUnit | Unicorn | `qa-release/` |

The catalog refers to phone behavior only by logical capability ID. This prevents a PebbleOS test
from embedding CoreApp screen structure, while keeping the full end-to-end order visible beside the
firmware it qualifies.

## `release-hw-android-obelix` coverage

Before the catalog runs, Unicorn erases the watch, installs the fixed debug PRF, clears phone and
Bluetooth state, installs the selected CoreApp APK, onboards and pairs, discovers the default
"from" firmware, installs the selected PebbleOS dual-slot PBZ through CoreApp, reconnects over
Pebble Protocol, and verifies the target version.

The catalog then performs these tests in order:

1. Open Bluetooth, Notifications, and System menus and prove each screen renders.
2. Open Health and record its rendered step-count screen. Physical step detection is out of scope.
3. Install Stride through CoreApp, start it from Watchfaces, and prove a face replaces the menu.
4. Install Weather Land and prove its date and temperature slot render after phone data arrives.
5. Install Transit and prove it renders a Caltrain/Palo Alto schedule.
6. Verify the installed watchface and app are synchronized back into the CoreApp locker.
7. Select the test vibe, post CoreApp's own test notification, and prove it appears on the watch.
8. Select and verify the Cloud Only speech-recognition option.
9. Submit a real QA bug report containing the run marker so eng-dash can associate it with the run.
10. Read and capture the final firmware version.

After the catalog, Unicorn pulls the firmware log over BLE and publishes per-step JUnit, phone
video, screenshots, CoreApp logcat, sidecar logs, and the dehashed watch log.

## Pull-request testing

An unmerged change is tested without copying definitions into Unicorn:

1. Build the PebbleOS PR to get its merged dual-slot `firmware-obelix_pvt` artifact.
2. Build the CoreApp PR to get its `app-release` APK.
3. Manually dispatch PebbleOS `release-hw-android-obelix` from its integration branch with the
   Unicorn integration ref, both run IDs, and the CoreApp source SHA. It resolves the PebbleOS SHA
   from the firmware run and dispatches Unicorn through the same REST handoff used after merge.
   Unicorn checks out each catalog at the exact SHA that produced its artifact.

The recommended merge order is Unicorn executor first, CoreApp capabilities second, and PebbleOS
catalog last. Older artifact commits without both catalogs continue through Unicorn's legacy flow
during the migration window.
