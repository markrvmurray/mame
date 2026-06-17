# MAME — `mrvmurray/mame` fork

A personal fork of [MAME](https://github.com/mamedev/mame) carrying two
custom homebrew drivers used as **MC6809 / HD6309 simulators for testing the
[`llvm-mc6809`](https://github.com/mrvmurray/llvm-mc6809) backend and the
companion picolibc port**. Upstream MAME is the host project; the fork
adds drivers and headless wrappers without touching anything else of
significance.

## Why this fork exists

The `llvm-mc6809` backend produces ELF objects for the Motorola 6809 and
Hitachi 6309. Validating those objects needs a fast, scriptable simulator
that:

- runs the ELF (or a flattened cart image) headless,
- streams the program's ACIA output to stdout for `FileCheck`,
- propagates an exit code from a halt port,
- counts cycles for the picolibc benchmark ledger,
- and — for 6309 coverage — runs Hitachi-only instructions natively.

The primary simulator is [`usim09batch`](https://github.com/mrvmurray/usim)
(MC6809-only). This fork's `llvm6309` driver is its HD6309 counterpart, and
the `picothing` driver is a near-faithful emulation of the Pico-Thing SBC
(the target hardware) used to validate higher-level workloads against the
same toolchain.

Both drivers share an IO layout in the `$FFCx` zone with `usim09batch` and
`usim09pt`, so a debug-build firmware runs unchanged across all four
simulators.

## The drivers

| Subtarget    | CPU       | Hardware                                                                                                          | Purpose                                                                 |
|--------------|-----------|-------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------|
| `llvm6309`   | HD6309    | ~64 KB flat RAM, MC6850 ACIA, SystemWatchpoint, TraceCtl, halt port                                               | Headless lit-test runner for the `llvm-mc6809` backend's 6309 coverage  |
| `picothing`  | MC6809E   | DAT MMU + 2 MB paged RAM, dual MC6850 ACIA, 50 Hz tick, PATA IDE (file-backed), FRAM (file-backed), SystemWatchpoint | Heavyweight Pico-Thing SBC emulator for picolibc + NitrOS-9 work        |

Full hardware models, memory maps, build flags, gotchas, and a hand-written
test-program skeleton are in [`README_MRVM.md`](README_MRVM.md). Interactive
lldb / gdb attach via MAME's gdbstub is documented in
[`README_GDBSTUB.md`](README_GDBSTUB.md).

## Quick start

Prerequisites on macOS: `brew install sdl3 pkg-config`.

### `llvm6309` (HD6309 lit-test runner)

```sh
make SUBTARGET=llvm6309 REGENIE=1 -j$(sysctl -n hw.ncpu)
./mamellvm6309-headless path/to/test.bin [timeout-seconds]
# stdout = ACIA serial output
# $?     = byte written to halt port ($FFCC), or 124 on -seconds_to_run cap
```

The wrapper sets `SDL_VIDEO_DRIVER=dummy` and forwards every headless flag
MAME needs. Cycle counts land in stderr as `cycles=<N>` for the picolibc
bench ledger.

For lldb / gdb attach:

```sh
./mamellvm6309-headless --debug[=PORT] path/to/test.bin
# MAME pauses until a client connects on the chosen port (default 2159)
```

### `picothing` (Pico-Thing SBC)

```sh
make SUBTARGET=picothing REGENIE=1 -j$(sysctl -n hw.ncpu)
PICOTHING_DISK_FILE=/path/to/disk.img \
PICOTHING_FRAM_FILE=/path/to/picothing.fram \
    ./mamepicothing-headless path/to/firmware.s19 [timeout-seconds]
```

The firmware loader auto-detects raw 64 KB binaries, Intel HEX, Motorola
S-record, and ELF32 (LE/BE). Pico-Thing firmware has no halt port — the
wrapper always exits with `124` on a clean run; termination is via
`-seconds_to_run` or a SystemWatchpoint trap.

### Both subtargets are *subset* builds

Each `SUBTARGET=` build only compiles its one driver plus the minimal set
of devices it depends on. Link time is seconds, not minutes — there's no
need to ever do the full `make` in this fork unless you're chasing an
upstream regression.

See [`BUILDING.md`](BUILDING.md) for the build mechanics in detail:
how the subtarget `.lua` and `.lst` files scope the compile, what's
unavoidably always built, and how to handle build breaks coming from
upstream churn.

## What's not modified

Everything outside `src/mame/homebrew/{llvm6309,picothing}*`, the two
subtarget LST + Lua files, the headless wrappers, and the
`run-mc6809-mame` helper is upstream MAME. The fork tracks
`mamedev/mame:master` and merges upstream periodically. See `git log` for
the local commits and `README_MRVM.md` for the list of fork-only files.

## Upstream MAME

The rest of this document is upstream MAME's `README` content — kept
intact for context and to honour the project's licensing.

---

## What is MAME?

MAME is a multi-purpose emulation framework.

MAME's purpose is to preserve decades of software history. As electronic
technology continues to rush forward, MAME prevents this important
"vintage" software from being lost and forgotten. This is achieved by
documenting the hardware and how it functions. The source code to MAME
serves as this documentation. The fact that the software is usable
serves primarily to validate the accuracy of the documentation (how else
can you prove that you have recreated the hardware faithfully?). Over
time, MAME (originally stood for Multiple Arcade Machine Emulator)
absorbed the sister-project MESS (Multi Emulator Super System), so MAME
now documents a wide variety of (mostly vintage) computers, video game
consoles and calculators, in addition to the arcade video games that
were its initial focus.

## Where can I find out more?

* [Official MAME Development Team Site](https://www.mamedev.org/) (includes binary downloads, wiki, forums, and more)
* [MAME Testers](https://mametesters.org/) (official bug tracker for MAME)

### Community

* [MAME Forums on bannister.org](https://forums.bannister.org/ubbthreads.php?ubb=cfrm&c=5)
* [r/MAME](https://www.reddit.com/r/MAME/) on Reddit
* [MAMEWorld Forums](https://www.mameworld.info/ubbthreads/)

## Development

### CI status and code scanning

[![CI (Linux)](https://github.com/mamedev/mame/workflows/CI%20(Linux)/badge.svg)](https://github.com/mamedev/mame/actions/workflows/ci-linux.yml) [![CI (Windows](https://github.com/mamedev/mame/workflows/CI%20(Windows)/badge.svg)](https://github.com/mamedev/mame/actions/workflows/ci-windows.yml) [![CI (macOS)](https://github.com/mamedev/mame/workflows/CI%20(macOS)/badge.svg)](https://github.com/mamedev/mame/actions/workflows/ci-macos.yml) [![Compile UI translations](https://github.com/mamedev/mame/workflows/Compile%20UI%20translations/badge.svg)](https://github.com/mamedev/mame/actions/workflows/language.yml) [![Build documentation](https://github.com/mamedev/mame/workflows/Build%20documentation/badge.svg)](https://github.com/mamedev/mame/actions/workflows/docs.yml)  [![Coverity Scan Status](https://scan.coverity.com/projects/5727/badge.svg?flat=1)](https://scan.coverity.com/projects/mame-emulator)

(These badges reflect upstream `mamedev/mame`, not this fork.)

### How to compile

If you're on a UNIX-like system (including Linux and macOS), it could be
as easy as typing

```
make
```

for a full build, or

```
make SUBTARGET=tiny
```

for a build including a small subset of supported systems. See the
[Compiling MAME](http://docs.mamedev.org/initialsetup/compilingmame.html)
page for full prerequisites.

For recent versions of macOS you need [Xcode](https://developer.apple.com/xcode/)
including command-line tools and SDL 3.

For Windows users, the upstream project provides a
[build environment](http://www.mamedev.org/tools/) based on MinGW-w64.
Visual Studio builds are also possible:

```
make vs2022
make vs2022 MSBUILD=1
```

### Coding standard

MAME source code should be viewed and edited with your editor set to use
four spaces per tab. Tabs are used for initial indentation of lines, with
one tab used per indentation level. Spaces are used for other alignment
within a line.

Some parts of the code follow [Allman style](https://en.wikipedia.org/wiki/Indent_style#Allman_style);
some parts of the code follow [K&R style](https://en.wikipedia.org/wiki/Indent_style#K.26R_style)
-- mostly depending on who wrote the original version. **Above all else,
be consistent with what you modify, and keep whitespace changes to a
minimum when modifying existing source.** For new code, the majority
tends to prefer Allman style.

All contributors need to either add a standard header for license info
(on new files) or inform us of their wishes regarding which of the
following licenses they would like their code to be made available under:
the [BSD-3-Clause](http://opensource.org/licenses/BSD-3-Clause) license,
the [LGPL-2.1](http://opensource.org/licenses/LGPL-2.1), or the
[GPL-2.0](http://opensource.org/licenses/GPL-2.0).

See more specific [C++ Coding Guidelines](https://docs.mamedev.org/contributing/cxx.html)
on the upstream documentation site.

## License

The MAME project as a whole is made available under the terms of the
[GNU General Public License, version 2](http://opensource.org/licenses/GPL-2.0)
or later (GPL-2.0+), since it contains code made available under multiple
GPL-compatible licenses. A great majority of the source files (over 90%
including core files) are made available under the terms of the
[3-clause BSD License](http://opensource.org/licenses/BSD-3-Clause). The
fork-only files in `src/mame/homebrew/` are BSD-3-Clause.

Please note that MAME is a registered trademark of Gregory Ember, and
permission is required to use the "MAME" name, logo, or wordmark.

<a href="http://opensource.org/licenses/GPL-2.0" target="_blank">
<img align="right" width="100" src="https://opensource.org/wp-content/uploads/2009/06/OSIApproved.svg">
</a>

    Copyright (c) 1997-2026  MAMEdev and contributors

    This program is free software; you can redistribute it and/or modify it
    under the terms of the GNU General Public License version 2, as provided in
    docs/legal/GPL-2.0.

    This program is distributed in the hope that it will be useful, but WITHOUT
    ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
    FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
    more details.

Please see [COPYING](COPYING) for more details.
