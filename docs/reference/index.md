# Reference

Lookup material: the firmware C API reference, external talks and documents
about PebbleOS, and specifications for the protocols used by the firmware
and its tooling.

## C API reference

The Doxygen-generated reference for the firmware sources (applib, services,
kernel, drivers) is published alongside this site:

<a href="../apidoc/index.html">PebbleOS API reference</a>

## App SDK guides

Hand-written guides for app-facing APIs that need more than the per-function
Doxygen reference.

```{toctree}
:maxdepth: 1

recognizers.md
```

## External resources

Podcasts, presentations and developer documents from the Pebble community.

```{toctree}
:maxdepth: 1

external.md
```

## PULSEv2 protocol suite

PULSE is the serial protocol spoken between the firmware and the host
tooling (`pbl console`, flash imaging, `tools/pulse/`). These pages
specify the wire format and its transports.

```{toctree}
:maxdepth: 1

pulse2/pulse2.md
pulse2/reliable-transport.md
pulse2/flash-imaging.md
pulse2/history.md
```

## Pebble Protocol endpoints

Wire formats of the Bluetooth protocol spoken between the firmware and
the phone apps.

```{toctree}
:maxdepth: 1

blob-db.md
```

## Binary formats

The file and resource formats produced by the firmware tooling. These
pages summarize the layouts; the packed structs in the referenced
sources stay authoritative.

```{toctree}
:maxdepth: 1

formats/font.md
formats/pbi.md
formats/pdc.md
formats/timezone.md
formats/sle.md
```
