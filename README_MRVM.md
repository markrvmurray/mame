# README_MRVM

Personal notes for this MAME fork. The `MRVM` suffix marks documents that are
mine and are not part of upstream MAME.

## `llvm6309` — HD6309 SBC for llvm-mc6809 backend validation

A minimal single-board computer driver added to exercise the
[`llvm-mc6809`](../llvm-mc6809) backend against real HD6309 semantics. Serves
as the HD6309 counterpart to the existing `usim09batch` headless 6809
simulator used for `llvm-mc6809` lit execution tests — `usim09batch` is
6809-only, so the 6309-specific instructions need a separate harness. This
driver fills that gap.

### Hardware model

| Range         | Size     | Device                                                 |
|---------------|----------|--------------------------------------------------------|
| `$0000-$BFFF` | 48K      | RAM                                                    |
| `$C000-$C001` | 2 bytes  | MC6850 ACIA (serial console, polled — no IRQ hookup)   |
| `$C002`       | 1 byte   | Exit port (write-only; any write → deferred clean exit) |
| `$C003-$FFFF` | ~16K − 3 | ROM window, backed by a runtime `generic_slot_device`  |

- CPU: Hitachi HD6309 at an 8 MHz crystal (2 MHz E clock). Boots in 6809
  emulation mode — execute `LDMD #1` early to engage native mode.
- ACIA clock: 1.8432 MHz → ÷16 → 115200 baud. The RS232 slot's `null_modem`
  card has its input-port defaults overridden to match (115200 / 8N1); without
  that override it would decode bit-level garbage at its built-in 9600 baud
  default.
- The ACIA and exit-port mappings are defined *after* the ROM mapping in the
  same `address_map`, so they overlay the first 3 bytes of the ROM window
  (later-wins). Your cart binary is a full 16K image; the first 3 bytes are
  wasted — start your code at `$C010` (or anywhere ≥ `$C003`) and leave those
  three bytes as padding.
- The exit port **defers the actual exit by 2 ms** via a one-shot timer so any
  byte still being shifted out of the ACIA's TX shift register reaches the
  bitbanger output file before MAME tears down. Without this delay, the last
  character of every test run gets silently dropped.
- Reset/IRQ vectors live at the usual `$FFF0-$FFFF`, which are ROM.

### Files

| Path                                          | Purpose                                                                              |
|-----------------------------------------------|--------------------------------------------------------------------------------------|
| `src/mame/homebrew/llvm6309.cpp`              | Driver source                                                                        |
| `src/mame/homebrew/llvm6309_rs232_core.cpp`   | Local stripped copy of the RS-232 bus core (see RS-232 workaround below)             |
| `src/mame/llvm6309.lst`                       | Driver list for the subtarget                                                        |
| `scripts/target/mame/llvm6309.lua`            | GENie subtarget — minimal scope                                                      |

### Subtarget scope

The subtarget enables only:

- `CPUS["M6809"]` — covers MC6809, HD6309 and Konami variants
- `MACHINES["ACIA6850"]` — the serial console ACIA
- `BUSES["GENERIC"]` — the runtime cartridge slot used for `-rom file.bin`

Everything else (video, sound, most machine tokens) is off. Link time is a
fraction of a full MAME build.

### The RS-232 workaround

`BUSES["RS232"] = true` is **intentionally NOT set.** If it were, `bus.lua`
would compile every rs232 card (`heath_h19`, `ie15`, `mboardd`, `scorpion`,
`sun_kbd`, `swtpc8212`, `terminal`, `votraxtnt`, ...) into `liboptional.a`,
and `bus/rs232/rs232.cpp`'s `default_rs232_devices()` function would hard-
reference them all. Even though this driver never calls that function, the
linker pulls `rs232.o` whole from the archive to satisfy our own
`rs232_port_device` references, and `rs232.o`'s static references then drag
in every terminal card's `.o`, which recursively pull in `beep_device`,
`mc6845_device`, `ay8910_device`, `m6803`/`hd6301` CPUs, etc. A real rabbit
hole.

Instead:

- `llvm6309_rs232_core.cpp` is a local stripped copy of
  `src/devices/bus/rs232/rs232.cpp` that omits the terminal-card includes and
  omits `default_rs232_devices()`. It provides exactly the bus-core symbols
  (`rs232_port_device`, `device_rs232_port_interface`) that this driver needs.
- `src/devices/bus/rs232/null_modem.cpp` is compiled directly into this
  subtarget's driver library via an explicit entry in the subtarget's
  `files {}` block. It only depends on `imagedev/bitbngr` which is already
  in `machine.lua`'s unconditional section.
- The driver defines its own minimal rs232 slot-option function
  (`llvm6309_rs232_devices`) that offers only `null_modem`, so
  `default_rs232_devices` is never referenced.

Result: no terminal-card transitive dependencies, minimal link time.

If upstream ever changes the rs232 bus-core API, `llvm6309_rs232_core.cpp`
must be kept in sync. The file is small (~130 lines) and the bus core doesn't
change often.

### Build

```sh
make SUBTARGET=llvm6309 REGENIE=1 -j$(sysctl -n hw.ncpu)
```

- `REGENIE=1` is **required on the first build** and **any time you edit
  `scripts/target/mame/llvm6309.lua`** — GENie has to regenerate project files
  to see the new subtarget.
- The output binary is `./mamellvm6309` in the repo root (~70 MB).
- Before the first build, IDE/LSP diagnostics on the .cpp files will complain
  about `emu.h not found` and a cascade of unknown types. This is normal —
  MAME's include paths are generated by GENie, not static. They resolve on
  first build.

### Running tests

**Use the `mamellvm6309-headless` wrapper** for lit-friendly runs on macOS —
it combines `SDL_VIDEO_DRIVER=dummy` with `-video none -sound none` so MAME
produces no window, no focus steal, no audio side effects, and no
CoreAudio warnings. Output capture still works, everything else is
unchanged.

```sh
: > /tmp/out.txt                     # IMPORTANT: pre-create empty file, see below
./mamellvm6309-headless \
    -rom /path/to/test.bin \
    -rs232 null_modem -bitb /tmp/out.txt \
    -seconds_to_run 2
FileCheck %s < /tmp/out.txt
```

The wrapper sets `SDL_VIDEO_DRIVER=dummy` in the environment before exec'ing
`./mamellvm6309`, and forwards `-video none -sound none -window -nothrottle`
plus the driver name `llvm6309` automatically. Your arguments are appended
after those, so MAME's option parser lets you override any of them if
needed (later wins).

If you really want to see the UI (for interactive poking) run
`./mamellvm6309` directly without the wrapper. That uses the full SDL3 OSD
with a visible window.

#### Why all three knobs are needed on macOS

- **`SDL_VIDEO_DRIVER=dummy`** makes SDL3 use its null video driver. SDL
  still constructs `SDL_Window` objects, but they are backed by nothing — no
  `NSWindow`, no Cocoa activation, no focus steal. Without this, MAME's
  SDL3 OSD always creates a real Cocoa window regardless of `-nowindow`.
- **`-video none`** selects MAME's `drawnone` renderer. With
  `SDL_VIDEO_DRIVER=dummy`, SDL's OpenGL context creation fails, so MAME's
  default OpenGL renderer errors out ("OpenGL support is either not
  configured in SDL or not available in current SDL video driver (dummy)").
  `-video none` sidesteps renderer initialisation entirely.
- **`-sound none`** suppresses CoreAudio init. Without it, every run prints
  `CoreAudio: couldn't get size of layout 2003332927` twice on startup.
  Doesn't affect correctness, but the warnings are noisy in lit output.
- **`-window -nothrottle`** — `-window` tells MAME not to go fullscreen (it
  already isn't, with dummy driver, but the option parser still expects a
  decision); `-nothrottle` removes the frame-rate cap so emulation runs as
  fast as the host allows.

#### CLI flag details (what the wrapper forwards)

- **`-rom file.bin`** — the ROM image is loaded through a
  `generic_slot_device`. No hash verification, no ZIP, no `rompath` — just
  hand it a raw 16K binary. Max size 16K; anything larger is rejected. The CLI
  option name is `-rom` (the media name MAME exposes for the cart slot), not
  `-cart`.
- **`-bitb /tmp/out.txt` requires a pre-existing file.** MAME's bitbanger
  device opens the file in read/write mode via `image_device::exists()`; if
  the file does not already exist, load fails with "No such file or directory"
  and no output is captured. `: > /tmp/out.txt` (or `touch`) creates an empty
  file before each run.
- **`-rs232 null_modem -bitb <path>`** — the driver restricts the RS232 slot
  options to `null_modem` only (see the "RS-232 workaround" section above),
  so this is the only sensible choice. `-bitb` captures all serial TX to the
  file. The driver also overrides the `null_modem` card's input-port defaults
  to 115200 baud 8N1 to match the ACIA clock; without that override, the
  null_modem would decode at its built-in 9600 baud default and produce
  bit-level garbage.
- **`-seconds_to_run N`** — safety net for runaway programs. Normal clean
  termination is via the `$C002` exit port — the last instruction of each
  test program stores any value to `$C002` and MAME exits 2 ms later (enough
  time for the ACIA to drain its TX shift register).

#### Performance note

With the headless wrapper, each run takes ~1 second of wall time. The
actual emulation finishes in microseconds; the remaining overhead is
MAME core init/teardown and SDL3 init/quit. The biggest single win was
disabling the MIDI provider (`-midiprovider none`) — PortMidi's
`Pm_Initialize` makes a synchronous XPC call to the CoreMIDI server
that blocked for 3+ seconds on macOS. Disabling all input providers
(`-keyboardprovider none`, etc.), plugins, font enumeration, and the
debugger module shaved another second.

### Test program skeleton (lwasm)

Verified working minimal program — prints `Hello from HD6309\r\n` and exits:

```asm
ACIA_STATUS     equ     $c000
ACIA_DATA       equ     $c001
EXIT_PORT       equ     $c002
TDRE            equ     $02

                org     $c000
                fcb     0,0,0           ; overlaid by ACIA + exit port

                org     $c010
reset           ldmd    #1              ; 6309 native mode (6309-only insn)
                lds     #$bfff
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

done            sta     EXIT_PORT       ; any write → schedule_exit() in 2 ms
halt            bra     halt

message         fcc     "Hello from HD6309"
                fcb     $0d,$0a,0

                org     $fffe
                fdb     reset
```

Assemble and slice the bottom 16K off the full 64K image:

```sh
lwasm --format=rawrel --6309 --output=test.full test.asm
tail -c 16384 test.full > test.bin
```

The `--format=rawrel` note is important: plain `--format=raw` does not handle
the forward `org $fffe` for the reset vector and produces a truncated file.
`rawrel` emits the entire 64K space; `tail -c 16384` keeps only the ROM window
at `$C000-$FFFF`.

### Caveats / gotchas

- **The MC6850 ACIA must be initialised** — master reset (`$03` to control)
  then configure (`$15` for /16 8N1 no-IRQ). Without this, TDRE never
  asserts and the test program loops forever.
- **3 bytes of ROM are unreachable** (`$C000-$C002`) because the ACIA and
  exit port overlay them. Don't put code or data there.
- **ACIA is polled only** — no IRQ line wired from the ACIA to the CPU.
  Test programs must poll TDRE/RDRF on `$C000`.
- **HD6309 native-mode switch** — the CPU boots in 6809-compatible mode.
  Use `LDMD #1` to enable the extra registers (E, F, V, W/Q) and 6309
  instructions.
- **The exit port defers exit by 2 ms**, not instantly, so the ACIA TX
  shift register can drain. If you genuinely want instant exit, remove
  the timer in `exit_w` — but you'll lose the trailing byte of every run.
- **`must_be_loaded`** — the cart slot is mandatory; MAME will refuse to
  start without `-rom`.
- **Pre-create the bitbanger output file** or `-bitb` will fail to open it.
- **No software list** — this driver is deliberately raw. Nothing is wired
  into a `.hsi`/softlist file, and that's intentional (tests change every run).
- **Not upstream** — these files are local to this fork. Don't expect them
  to survive a rebase onto `mamedev/mame` without carrying the commit forward.
