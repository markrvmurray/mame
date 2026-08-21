# README_MRVM

Personal notes for this MAME fork. The `MRVM` suffix marks documents that are
mine and are not part of upstream MAME.

Three subtargets live here — two custom homebrew drivers and one scoped
build of upstream code:

| Subtarget    | Purpose                                                            |
|--------------|--------------------------------------------------------------------|
| `llvm6309`   | Headless HD6309 SBC for `llvm-mc6809` backend lit execution tests  |
| `picothing`  | Pico-Thing MC6809 SBC emulator (DAT MMU, IDE, FRAM, dual ACIA)     |
| `coco3`      | Tandy Color Computer 3 (+ CoCo 1/2, Dragon) for NitrOS-9 Level 2   |

`coco3` contains **no forked driver code** — it is upstream's
`src/mame/trs/` compiled behind a narrow token list. Only the subtarget
`.lua`, the `.lst` and the headless wrapper are ours.

The two homebrew drivers share the same IO layout in the `$FFCx` zone so a
debug-build firmware can target either without an `#ifdef`. The split is:

- `llvm6309` is intentionally minimal — flat 64 KB RAM, one ACIA, watchpoint,
  halt port. No MMU, no IDE, no FRAM. Optimised for fast lit-test turnaround.
- `picothing` is the heavyweight sibling — DAT MMU + 2 MB paged RAM, dual
  ACIA, 50 Hz tick, PATA IDE (file-backed), FRAM (file-backed), and the
  same watchpoint. No halt port — Pico-Thing firmware has no host-exit
  mechanism, so termination is via `-seconds_to_run`.

Both layouts mirror the `usim` reference emulators (`usim09batch` and
`usim09pt` next door in `../usim`) so the same test programs run unchanged
across emulators. `coco3` shares none of this — it is stock hardware, not
a harness.

## `llvm6309` — HD6309 SBC for llvm-mc6809 backend validation

A minimal single-board computer driver added to exercise the
[`llvm-mc6809`](../llvm-mc6809) backend against real HD6309 semantics. The
HD6309 counterpart to the existing `usim09batch` headless 6809 simulator —
`usim09batch` is 6809-only, so the 6309-specific instructions need a separate
harness.

### Hardware model

| Range         | Size      | Device                                                          |
|---------------|-----------|-----------------------------------------------------------------|
| `$0000-$FFC3` | ~64K      | RAM (contiguous — no IO wedge in the middle)                    |
| `$FFC4-$FFC5` | 2 bytes   | MC6850 ACIA (serial console, polled — no IRQ hookup)            |
| `$FFC6-$FFC9` | 4 bytes   | RAM (no devices wired)                                          |
| `$FFCA`       | 1 byte    | SystemWatchpoint control register                               |
| `$FFCB`       | 1 byte    | TraceCtl (program-controlled host-side logging toggle)          |
| `$FFCC`       | 1 byte    | Halt / exit port (write-only; any write → deferred clean exit; value captured as process exit code) |
| `$FFCD-$FFDF` | 19 bytes  | RAM                                                             |
| `$FFE0-$FFFF` | 32 bytes  | SystemWatchpoint vector + snippet shadow                        |

- CPU: Hitachi HD6309 at an 8 MHz crystal (2 MHz E clock). Boots in 6809
  emulation mode — execute `LDMD #1` early to engage native mode.
- ACIA clock: 1.8432 MHz → ÷16 → 115200 baud. The RS232 slot's `null_modem`
  card has its input-port defaults overridden to match.
- IO overlays sit on top of the RAM map via later-wins `address_map` entries;
  the RAM backing under the IO range is wasted but harmless.
- The exit port **defers the actual exit by 2 ms** via a one-shot timer so any
  byte still being shifted out of the ACIA's TX shift register reaches the
  bitbanger output file before MAME tears down.
- The TraceCtl encoding matches `usim`'s:
  - `0x00` → disable
  - `0x01` → enable (emit a logerror marker with PC)
  - `0x02` → enable + verbose (logerror with full register dump)
- SystemWatchpoint at `$FFE0-$FFFF`: 32-byte shadow buffer owned by the
  device. Writes go to the shadow when disarmed. Writing non-zero to `$FFCA`
  arms the watchpoint: the device saves the current NMI vector, copies a
  16-byte BREAK snippet to `$FFE0-$FFEF`, and repoints the NMI vector at
  `$FFE0`. Any subsequent write to `$FFE0-$FFFF` while armed is blocked and
  asserts NMI; the snippet copies the 12-byte NMI register frame
  (`CC,A,B,DP,X,Y,U,PC`) to `$FFD0-$FFDB` (SHARED area) and spins. Disarm
  by writing 0 to `$FFCA` or via CPU reset. Matches the real Pico-Thing
  hardware behaviour byte-for-byte.

### Files

| Path                                          | Purpose                                                                              |
|-----------------------------------------------|--------------------------------------------------------------------------------------|
| `src/mame/homebrew/llvm6309.cpp`              | Driver source                                                                        |
| `src/mame/homebrew/llvm6309_rs232_core.cpp`   | Local stripped copy of the RS-232 bus core (shared with `picothing`)                 |
| `src/mame/llvm6309.lst`                       | Driver list for the subtarget                                                        |
| `scripts/target/mame/llvm6309.lua`            | GENie subtarget — minimal scope                                                      |

### Build

```sh
make SUBTARGET=llvm6309 REGENIE=1 -j$(sysctl -n hw.ncpu)
```

- `REGENIE=1` is required on the first build and any time you edit
  `scripts/target/mame/llvm6309.lua`.
- Output binary: `./mamellvm6309` (~70 MB).
- On macOS, MAME's SDL3 detection needs `pkg-config` + `brew install sdl3`,
  or pass `SDL_INSTALL_ROOT=/opt/homebrew USE_LIBSDL=1` on the make line.

### Running tests

Use the `mamellvm6309-headless` wrapper:

```sh
./mamellvm6309-headless /path/to/test.bin [timeout-seconds]
# stdout = serial output; $? = value written to halt port ($FFCC)
```

The wrapper sets `SDL_VIDEO_DRIVER=dummy` and forwards all the headless
flags. Without it, MAME's SDL3 OSD will open a real Cocoa window and steal
focus on macOS.

### Test program skeleton (lwasm)

```asm
ACIA_STATUS     equ     $ffc4
ACIA_DATA       equ     $ffc5
TRACE_CTL       equ     $ffcb
HALT_PORT       equ     $ffcc
TDRE            equ     $02

                org     $0100
reset           ldmd    #1              ; 6309 native mode
                lds     #$ffc3          ; stack just above the IO slab
                lda     #$03            ; 6850 master reset
                sta     ACIA_STATUS
                lda     #$15            ; /16, 8N1, no IRQ
                sta     ACIA_STATUS
                ldx     #message
print_loop      lda     ,x+
                beq     done
tx_wait         ldb     ACIA_STATUS
                bitb    #TDRE
                beq     tx_wait
                sta     ACIA_DATA
                bra     print_loop
done            clra
                sta     HALT_PORT
halt            bra     halt

message         fcc     "Hello from HD6309"
                fcb     $0d,$0a,0

                org     $fffe
                fdb     reset
```

```sh
lwasm --format=rawrel --6309 --output=test.bin test.asm
./mamellvm6309-headless test.bin 2
```

`--format=rawrel` emits a full 64K image; the cart loader accepts it
directly (no `tail -c` slicing).

### Caveats / gotchas

- The MC6850 ACIA must be initialised (master reset `$03` then config `$15`).
- HD6309 boots in 6809 emulation mode — `LDMD #1` to engage native mode.
- The exit port defers exit by 2 ms so the ACIA TX shift register can drain.
- Writes to `$FFE0-$FFFF` while the watchpoint is armed are blocked and
  raise NMI. While disarmed (the default at reset) they update the shadow.
- The cart slot is mandatory; MAME refuses to start without `-rom`.

## `picothing` — Pico-Thing MC6809 SBC emulator

A close mirror of the real Pico-Thing hardware: DAT MMU with 2 MB physical
RAM paged through 8 KB windows, dual MC6850 ACIAs (console + aux), 50 Hz
tick timer, PATA IDE (file-backed disk image), FRAM (file-backed
persistence), and the same SystemWatchpoint as `llvm6309`. Tracks the
`usim09pt` reference emulator (see `../usim/main_picothing.cpp`).

### Hardware model

| Range         | Size      | Device                                                          |
|---------------|-----------|-----------------------------------------------------------------|
| `$0000-$FDFF` | ~63K virt | DAT-translated RAM (2 MB physical via 8 KB pages, 32 tasks)     |
| `$FE00-$FEFF` | 256 bytes | DAT page table (32 tasks × 8 entries)                           |
| `$FF00-$FF09` | 10 bytes  | PATA IDE controller (file-backed disk image)                    |
| `$FFC0`       | 1 byte    | DAT task register (writes set the 5-bit active task)            |
| `$FFC4-$FFC5` | 2 bytes   | Console ACIA (MC6850)                                           |
| `$FFC6-$FFC7` | 2 bytes   | Auxiliary ACIA (MC6850, null peer — discards TX)                |
| `$FFC8-$FFC9` | 2 bytes   | 50 Hz tick timer ($FFC8 = ctrl, $FFC9 = status)                 |
| `$FFCA`       | 1 byte    | SystemWatchpoint control register                               |
| `$FFCB`       | 1 byte    | TraceCtl                                                        |
| `$FFD0-$FFDF` | 16 bytes  | FRAM (file-backed; SHARED area)                                 |
| `$FFE0-$FFFF` | 32 bytes  | SystemWatchpoint vector + snippet shadow                        |

- CPU: MC6809E at 1 MHz E clock (4 MHz crystal / 4).
- IRQ wiring: console ACIA, aux ACIA, and the 50 Hz tick all share the
  CPU's IRQ line via an `input_merger_any_high_device`. The watchpoint
  drives NMI. FIRQ is unused.
- DAT MMU: each task selects 8 entries from the 256-byte page table; each
  entry is a 1-byte physical-page index (0-255 → 8 KB chunks of the 2 MB
  physical store). Boot identity-maps every entry to its own index, so
  task 0 sees a 64 KB virtual space that lines up with the first 64 KB of
  physical RAM.
- FRAM persistence: 16 bytes at `$FFD0-$FFDF` saved to disk on every write.
  Path is from `PICOTHING_FRAM_FILE` env var (default `picothing.fram`).
- PATA IDE backing: 512-byte sectors via `fopen`/`fread`/`fwrite`. Path is
  from `PICOTHING_DISK_FILE` env var; if unset, IDE commands report
  `DRDY|ERR`.
- TraceCtl + SystemWatchpoint behave exactly as in `llvm6309`.

### Files

| Path                                          | Purpose                                                                              |
|-----------------------------------------------|--------------------------------------------------------------------------------------|
| `src/mame/homebrew/picothing.cpp`             | Driver source — all devices inline                                                   |
| `src/mame/picothing.lst`                      | Driver list                                                                          |
| `scripts/target/mame/picothing.lua`           | GENie subtarget                                                                      |

Shares `llvm6309_rs232_core.cpp` for the stripped RS-232 bus core.

### Build

```sh
make SUBTARGET=picothing REGENIE=1 -j$(sysctl -n hw.ncpu)
```

Output binary: `./mamepicothing` (~70 MB). Same SDL3 / pkg-config note as
`llvm6309`.

### Running

Use the `mamepicothing-headless` wrapper:

```sh
PICOTHING_DISK_FILE=/path/to/disk.img \
PICOTHING_FRAM_FILE=/path/to/picothing.fram \
./mamepicothing-headless /path/to/firmware.s19 [timeout-seconds]
# stdout = console serial output
# exit always 124 (timeout) — Pico-Thing has no host-exit mechanism
```

Firmware is loaded via the cart slot. The loader sniffs the file and
accepts:
- Raw 64K binary
- Intel HEX (`.hex`)
- Motorola S-record (`.s19`/`.srec`)
- ELF32 (LE or BE)

The bytes are distributed:
- `$0000-$FEFF` → DAT physical store (task 0 identity-mapped at boot)
- `$FFD0-$FFDF` → FRAM (16 bytes)
- `$FFE0-$FFFF` → SystemWatchpoint shadow (32 bytes; includes the reset
  vector at `$FFFE-$FFFF`)

### Caveats / gotchas

- The cart slot is mandatory.
- FRAM file is written synchronously on every byte write. For tests that
  shouldn't persist state, point `PICOTHING_FRAM_FILE` at a fresh temp
  file or delete `picothing.fram` between runs.
- The aux ACIA is wired but has no peer — its TX is dropped and it never
  delivers RX. Useful for firmware that probes for it without crashing.
- Pico-Thing firmware has no halt port. Use `-seconds_to_run` (or a
  SystemWatchpoint trap) to terminate.

## `coco3` — Tandy Color Computer 3

A scoped build of upstream's CoCo/Dragon drivers, added to boot and
exercise NitrOS-9 Level 2 on real GIME hardware semantics. Unlike the
other two this is not a simulator built for the LLVM backend — it is the
reference machine NitrOS-9 actually targets, so it answers "does this
behave on a real CoCo 3" rather than "does this instruction sequence
execute correctly".

### What is registered

24 machines, all from three upstream sources:

- `trs/coco3.cpp` — `coco3` (NTSC), `coco3h` (HD6309), `coco3p` (PAL), `msm3`
- `trs/coco12.cpp` — `coco`, `cocoh`, `coco2b`, `coco2bh`, `cp400`, `t4426`, …
- `trs/dragon.cpp` — `dragon32`, `dragon64`, `dragon64h`, `tanodr64`, …

`coco12.cpp` and `dragon.cpp` are compiled in whether or not they are
registered — `coco3_state` derives from `coco_state`, and the cartridge
slot-option function lives in `coco12.cpp` — so listing their machines
costs `.lst` lines and nothing else. The `coco3h` / `cocoh` / `dragon64h`
HD6309 variants are a useful cross-check against `llvm6309`.

### ROMs

**Not in this repo and not obtainable from it.** `coco3` needs
`coco3.rom` (Tandy/Microsoft Super Extended Color BASIC) and, because the
default cartridge is the floppy controller, `disk11.rom`. Put them under
`roms/` and check with:

```sh
./mamecoco3 -verifyroms coco3
```

Without them MAME exits with "Required files are missing" before the
machine ever starts.

### Build

```sh
make SUBTARGET=coco3 REGENIE=1 -j$(sysctl -n hw.ncpu)
```

Output binary: `./mamecoco3` (~81 MB). This subtarget is much fatter than
the other two — see `BUILDING.md` ("Why `coco3` is the fat one") for the
cascade that causes it and the recipe for fixing the link errors an
upstream merge will eventually produce.

### Running

```sh
./mamecoco3 coco3 -flop1 nitros9_l2.dsk          # interactive, with a window
./mamecoco3-headless -1 nitros9_l2.dsk 30        # scripted
```

Useful media and slot options (`-listmedia` / `-listslots` for the rest):

| Option              | Meaning                                                     |
|---------------------|-------------------------------------------------------------|
| `-flop1` / `-flop2` | Floppy drives 0 and 1 (`.dsk .dmk .jvc .vdk .os9` …)         |
| `-hard1` / `-hard2` | CoCo Virtual Hard Disk (`.vhd`) — NitrOS-9 hard disk         |
| `-cart`             | ROM cartridge image (`.ccc` / `.rom`)                        |
| `-ext`              | Cartridge slot device; defaults to `fdc`, also `multi`, `ide`, `rs232` |

### Headless capture — read this before scripting it

The CoCo 3 is **not** like `llvm6309` / `picothing`. Those are SBCs whose
console *is* a 6850 ACIA, so headless capture is total: the ACIA goes to
stdout and nothing is lost. A CoCo 3 has no such port. Its console is the
GIME video screen plus the keyboard matrix, and a stock NitrOS-9 Level 2
talks to `/term`, which is that screen. **There is no serial stream to
redirect.**

`mamecoco3-headless` therefore offers three partial views rather than one
complete one:

| View | Flag | What you get | Limitation |
|------|------|--------------|------------|
| Final screenshot | on by default | PNG of the GIME screen at the `-seconds_to_run` cap, written by `src/emu/video.cpp:989`. Path printed to stderr. | It is a picture — needs eyes or an image diff, not `grep` |
| Bit-banger serial | on by default | The 4-pin DIN port (PIA1 PA1/PB0) wired to a `null_modem`, echoed to stdout. Catches `/p` output, or a bit-banger term driver if your boot disk has one | 600 baud on a CoCo 3; not the console unless you make it so |
| DriveWire / Becker | `--becker[=PORT]` | A real headless console when the boot disk uses NitrOS-9's `term_becker` as `/term` | Needs a DriveWire server already listening on 127.0.0.1; the wrapper does not spawn one |

The Becker port is a `PORT_CONFNAME` configuration switch, not a slot or a
command-line option, so it cannot be set with a flag. `--becker` works by
generating a throwaway cfg directory holding just that one setting and
pointing `-cfg_directory` at it, leaving the real `cfg/` alone. The XML
shape mirrors `ioport_manager::save_game_inputs()`
(`src/emu/ioport.cpp:2896`) and hard-codes `CONFIG_VERSION` 10
(`src/emu/config.h:47`). **If an upstream merge bumps that constant, MAME
warns and ignores the file and `--becker` silently stops working** — that
is the one fragile thing in the wrapper.

Exit status is 124 on a clean run to the cap, matching the GNU
`timeout(1)` convention the other wrappers use. A CoCo 3 has no halt port,
so reaching the cap *is* the normal successful path; a non-124 status
means MAME itself failed (missing ROM, bad image) and is not laundered
into a pass.

### Files

| Path                                | Purpose                                  |
|-------------------------------------|------------------------------------------|
| `scripts/target/mame/coco3.lua`     | GENie subtarget — token list + file list |
| `src/mame/coco3.lst`                | Driver list                               |
| `mamecoco3-headless`                | Headless wrapper                          |

No driver source of ours; everything compiled comes from upstream
`src/mame/trs/`.

## The RS-232 workaround (`llvm6309` and `picothing` only)

`BUSES["RS232"] = true` is intentionally NOT set in either of those two
subtargets. **`coco3` does not and cannot use this workaround** — see the
note at the end of this section. If it
were, `bus/rs232/rs232.cpp`'s `default_rs232_devices()` would drag every
terminal card (`heath_h19`, `ie15`, `mboardd`, `scorpion`, `sun_kbd`,
`swtpc8212`, `terminal`, `votraxtnt`) into the link, transitively pulling in
`beep_device`, `mc6845_device`, `ay8910_device`, `m6803`/`hd6301` CPUs, and
more. Even though neither driver calls `default_rs232_devices()`, the linker
pulls `rs232.o` whole from `liboptional.a` to satisfy `rs232_port_device`
references, dragging the cascade.

Instead:

- `src/mame/homebrew/llvm6309_rs232_core.cpp` is a local stripped copy of
  `src/devices/bus/rs232/rs232.cpp` without the terminal-card includes and
  without `default_rs232_devices()`. It provides the bus-core symbols
  (`rs232_port_device`, `device_rs232_port_interface`) only. Used by both
  the `llvm6309` and `picothing` subtargets.
- `src/devices/bus/rs232/null_modem.cpp` is compiled directly into each
  subtarget's driver library via explicit `files {}` entries.
- Each driver defines its own minimal rs232 slot-option function offering
  only `null_modem`.

If upstream's `rs232.cpp` bus-core API changes, `llvm6309_rs232_core.cpp`
must be kept in sync (~130 lines, doesn't change often).

`coco3` is excluded from all of this. Its drivers are upstream's, so it
cannot supply its own slot-option function — and one of the CoCo cartridge
cards (`coco_rs232.cpp`) includes `bus/rs232/rs232.h` directly, so the
stripped core would buy nothing even if it could. `coco3` sets
`BUSES["RS232"] = true` and absorbs the full terminal-card cascade.

## Not upstream

These files are local to this fork. Don't expect them to survive a rebase
onto `mamedev/mame` without carrying the commits forward.
