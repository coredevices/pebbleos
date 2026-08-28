```{raw} html
<div style="display: none;">
```

# PebbleOS documentation

```{raw} html
</div>
```

::::{grid} 2
:reverse:

:::{grid-item}
:columns: 4
:class: sd-m-auto

<img src="_static/images/logo.svg" />

:::

:::{grid-item}
:columns: 8

```{div} sd-fs-3 home-title
Welcome to the PebbleOS documentation site!
```

PebbleOS is a lightweight, power-efficient operating system originally developed for Pebble smartwatches and now [released open source](https://opensource.googleblog.com/2025/01/see-code-that-powered-pebble-smartwatches.html).
It supports custom watchfaces and apps using C and JavaScript.
Optimized for memory-in-pixel (MIP) displays and long battery life, it features Bluetooth sync, a timeline interface, and a strong developer ecosystem through an open SDK.

```{button-ref} development/getting_started
:ref-type: doc
:color: primary
:class: sd-rounded-pill float-left

Get started
```

```{only} html
[![](https://img.shields.io/github/stars/coredevices/pebbleos?style=social)](https://github.com/coredevices/pebbleos)
```

:::

::::

::::{grid} 1 1 2 3
:class-container: text-center
:gutter: 3

:::{grid-item-card}
:link: development/getting_started
:link-type: doc

🚀 Prerequisites
^^^

Set up your environment to build PebbleOS from source!
:::

:::{grid-item-card}
:link: https://github.com/coredevices/pebbleos

⌚ Browse the sources
^^^

Browse the PebbleOS sources!
:::

:::{grid-item-card}
:link: reference/index
:link-type: doc

📖 Reference
^^^

Look things up: protocol specifications, podcasts, developer documents and more!
:::

::::

```{toctree}
:hidden:
:caption: 🛠️ Development
development/getting_started.md
development/options.md
development/building_fw.md
development/build_system.md
development/testing.md
development/qemu.md
development/debugging.md
development/moddable.md
development/sdk_export.md
development/contributing.md
```

```{toctree}
:hidden:
:caption: 🏛️ Architecture
architecture/index.md
```

```{toctree}
:hidden:
:caption: ⌚ Boards
boards/index.md
```

```{toctree}
:hidden:
:caption: 📖 Reference
reference/index.md
```
