# Building this fork's MC6809 / HD6309 simulators

This fork carries two homebrew drivers, `llvm6309` (HD6309) and `picothing`
(MC6809E, Pico-Thing SBC), used as simulators for validating
[`llvm-mc6809`](https://github.com/mrvmurray/llvm-mc6809) and its picolibc
port. Both are built as **GENie subtargets** so they compile only the code
they actually need — anything broken in the rest of upstream MAME
(`src/mame/<other-manufacturer>/`) literally cannot fail this fork's build,
because it never enters the compile.

## Build commands

```sh
make SUBTARGET=llvm6309  REGENIE=1 -j$(sysctl -n hw.ncpu)
make SUBTARGET=picothing REGENIE=1 -j$(sysctl -n hw.ncpu)
```

- **`REGENIE=1`** is mandatory on the first build, after editing the
  subtarget's `.lua`, and any time you switch between subtargets. Without
  it GENie reuses cached project files that point at the *previous*
  subtarget's source list, and your changes vanish silently.
- Output binaries: `./mamellvm6309` and `./mamepicothing` (each ~70 MB).
- Build time on Apple Silicon: ~2–5 minutes once `3rdparty/` is cached
  (vs ~30+ minutes for a full `make` of MAME).
- Link RAM: a few GB (vs ~30 GB for full MAME).

Prerequisites on macOS: `brew install sdl3 pkg-config`. If pkg-config is
unavailable for some reason, you can pass
`SDL_INSTALL_ROOT=/opt/homebrew USE_LIBSDL=1` on the make line to skip
it.

## How the scoping works

`SUBTARGET=<name>` redirects GENie to `scripts/target/mame/<name>.lua`
instead of the default `mame.lua`, and to `src/mame/<name>.lst` instead of
`mame.lst`. The two files have separate jobs:

| File                                       | Role                                                                   | Effect                                                              |
|--------------------------------------------|------------------------------------------------------------------------|---------------------------------------------------------------------|
| `scripts/target/mame/<name>.lua`           | Whitelists CPU / sound / video / machine / bus tokens + source files   | Anything not whitelisted never enters the compile                   |
| `src/mame/<name>.lst`                      | Lists the `GAME(...)` macros to register at runtime                    | Drivers absent here aren't registered, even if compiled             |

### What each subtarget pulls in

- **`CPUS["M6809"]`** — the m6809 directory (covers MC6809, HD6309, and the
  Konami variants — they share the same code).
- **`MACHINES["ACIA6850"]`** — the 6850 ACIA, used as the serial console
  on both drivers.
- **`MACHINES["INPUT_MERGER"]`** — picothing only; wires the two ACIAs and
  the tick timer onto a single CPU IRQ line via
  `input_merger_any_high_device`.
- **`BUSES["GENERIC"]`** — provides the runtime cartridge slot used to
  feed firmware via `-rom`.
- The driver source (`llvm6309.cpp` or `picothing.cpp`), the local
  stripped `llvm6309_rs232_core.cpp` (replaces the full `rs232.cpp` bus
  core — see `README_MRVM.md` for why), and `null_modem.cpp`.

### Always built regardless of subtarget

Even with a minimal subtarget, the following always compile:

- `src/emu/*` — the core emulation framework (device-tree, scheduler,
  memory system, save state).
- `src/osd/*` — the OS abstraction layer (SDL3 backend on macOS).
- `src/frontend/mame/*` — CLI handling and the built-in UI.
- `src/lib/util/*` — internal utility libraries.
- `3rdparty/*` — vendored externals (asio, lua, zlib, BGFX, …).

**Nothing from `src/mame/<other-manufacturer>/` is even seen by the
compiler.**

## When something in the always-built core breaks

The subtarget can't insulate you from upstream regressions in the
always-built tree — when one of those layers breaks, your subtarget
breaks too. Options in roughly increasing severity:

- **`NOWERROR=1`** — turn off `-Werror` for new warnings introduced by
  upstream churn. Won't help genuine compile errors.
- **`make clean`** — drop stale link state after switching subtargets or
  pulling a large upstream batch.
- **Roll back the offending upstream merge** — usually easier than
  patching around it. The merge is straightforward to spot in `git log`
  because upstream merges land as named merge commits; revert the merge
  with `git revert -m 1 <merge-sha>` and bisect later if you want.
- **Patch the offender locally** — last resort. Keep the patch in a
  fork-only commit so it stays visible and can be rebased over future
  upstream changes.

When investigating a build failure, read the actual error message:

- Error path under `src/mame/<other>/...` → your subtarget is misconfigured
  and accidentally including it. Check the `.lua` for an unintended
  `files {}` entry or a `MACHINES["..."]`/`BUSES["..."]` token that drags
  the file in transitively.
- Error path under `src/emu/`, `src/osd/`, `src/frontend/mame/`, a CPU
  core you depend on, or `3rdparty/` → genuine upstream regression. Apply
  one of the options above.

## Quick sanity check that the scope is right

After a successful build, each subtarget's driver list should contain
exactly one entry:

```sh
./mamellvm6309   -listxml 2>/dev/null | grep -c '<machine '   # → 1
./mamepicothing  -listxml 2>/dev/null | grep -c '<machine '   # → 1
```

If you get more than 1, the subtarget is leaking — usually a stray
entry has crept into the `.lst` file.

## Other useful make flags

| Flag                  | Effect                                                                  |
|-----------------------|-------------------------------------------------------------------------|
| `NOWERROR=1`          | Disable `-Werror`. Useful when touching upstream-adjacent code.         |
| `DEBUG=1`             | Debug build with assertions. Slower but easier to inspect under lldb.   |
| `TOOLS=1`             | Also build `chdman`, `romcmp`, `unidasm`, etc.                          |
| `TESTS=1`             | Build the unit-test binary (`mametests`).                               |
| `OSD=sdl`             | Force SDL2 OSD on macOS instead of the SDL3 default.                    |
| `make clean`          | Wipe build artifacts. Doesn't touch GENie cache; combine with `REGENIE=1`. |

## Running the resulting binary

See [`README_MRVM.md`](README_MRVM.md) for the full per-driver invocation
and [`README_GDBSTUB.md`](README_GDBSTUB.md) for the lldb / gdb attach
workflow over MAME's built-in gdbstub.
