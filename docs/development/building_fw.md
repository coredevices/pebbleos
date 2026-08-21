# Building firmware

Before building, make sure you've configured {doc}`./pbl <../development/options>`. Then, run the following:

```shell
./pbl build
```

## Loading firmware with a firmware development kit

Before attempting to flash, check the documentation for each {doc}`board <../boards/index>` on how to prepare and connect your watch for programming.

You can flash the built firmware by running:

```shell
./pbl flash
```

In some cases, you may have to specify the `--tty $SERIAL_ADAPTER` option where `$SERIAL_ADAPTER` is the path for your serial adapter, e.g. `/dev/ttyACM0`, `/dev/tty.usbmodem1102`, etc.

If flashing for the first time, you will also need to flash resources.
Some boards support direct resource programming by passing the `--resources` option.
The alternative is to flash while the firmware is running via the serial port using:

```shell
./pbl image_resources --tty $SERIAL_ADAPTER
```

When both firmware and resources are flashed, you should observe the watch booting into the main application.
You can also see the logs by opening the console:

```shell
./pbl console --tty $SERIAL_ADAPTER
```

Try sending `help` to get a list of available console commands.

## Loading firmware via Bluetooth

If you don't have a firmware development kit, you may bundle a `.pbz` file for sideloading 
onto your sealed watch:

```shell
./pbl bundle
```

The resulting `.pbz` file will be located in the `build/` directory. Transfer this file
to the device paired to your watch, then, in the Pebble app, enable `Settings -> Show debug options`.
Go back to the Devices tab, tap your watch, then `Firmware Update Debug -> Sideload FW`, and select the `.pbz` file.

On Android, flashing repeatedly is better scripted over adb, which installs without asking. This route
is open to an adb shell, which holds the `android.permission.DUMP` the broadcast requires,
and it needs `Show debug options` on in the app. `$PBZ` is the file from `build/`:

```shell
adb push $PBZ /data/local/tmp/firmware.pbz
adb shell am broadcast -a coredevices.pebble.SIDELOAD_FIRMWARE \
    -n coredevices.coreapp/coredevices.pebble.firmware.FirmwareSideloadReceiver \
    --es path /data/local/tmp/firmware.pbz
```

The app waits up to a minute for the watch to be connected, then starts the update and
shows its progress on the watch card in the Devices tab.

Not every Android version lets an app read `/data/local/tmp`. If the app reports the file
as missing, create `/sdcard/Android/data/coredevices.coreapp/cache` and push it there
instead.
