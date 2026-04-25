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

The memory map mirrors `usim09batch` exactly, so the same test programs run
unchanged on both emulators.

### Hardware model

| Range         | Size     | Device                                                          |
|---------------|----------|-----------------------------------------------------------------|
| `$0000-$FFCF` | ~64K     | RAM (contiguous — no IO wedge in the middle)                    |
| `$FFD0-$FFD1` | 2 bytes  | MC6850 ACIA (serial console, polled — no IRQ hookup)            |
| `$FFD2`       | 1 byte   | Exit port (write-only; any write → deferred clean exit; value captured as process exit code) |
| `$FFD3`       | 1 byte   | TraceCtl (program-controlled host-side logging toggle)          |
| `$FFD4-$FFEF` | 28 bytes | RAM (reserved for future IO)                                    |
| `$FFF0-$FFFF` | 16 bytes | Vector ROM — reads come from RAM populated at boot; writes dropped |

- CPU: Hitachi HD6309 at an 8 MHz crystal (2 MHz E clock). Boots in 6809
  emulation mode — execute `LDMD #1` early to engage native mode.
- ACIA clock: 1.8432 MHz → ÷16 → 115200 baud. The RS232 slot's `null_modem`
  card has its input-port defaults overridden to match (115200 / 8N1); without
  that override it would decode bit-level garbage at its built-in 9600 baud
  default.
- The IO slab at `$FFD0-$FFD3` is overlaid on the underlying RAM via
  later-wins `address_map` entries, so CPU accesses to those addresses go to
  the devices, not the RAM beneath. The RAM backing under the IO range is
  wasted but harmless.
- The vector region `$FFF0-$FFFF` uses `.nopw()` so writes are silently
  dropped — a runaway program can't clobber the reset entry. Reads still come
  from the underlying RAM, which is populated from the cart image at
  `machine_start` time.
- The exit port **defers the actual exit by 2 ms** via a one-shot timer so any
  byte still being shifted out of the ACIA's TX shift register reaches the
  bitbanger output file before MAME tears down. Without this delay, the last
  character of every test run gets silently dropped.
- The TraceCtl encoding matches `usim`'s:
  - `0x00` → disable
  - `0x01` → enable (emit a logerror marker with PC)
  - `0x02` → enable + verbose (logerror with full register dump)
  - anything else while enabled → logerror with PC
- For full per-instruction tracing or watchpoints, use MAME's built-in
  debugger: drop `-debugger none` from the wrapper and add
  `-debug -debugscript /path/to/script` (with `trace file.log` and
  `wpset …,1,w` commands inside). That path is not wired into the headless
  wrapper; it's there if you need it.

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
- On macOS, MAME's SDL3 detection needs either `pkg-config` +
  `brew install sdl3`, **or** an explicit `SDL_INSTALL_ROOT=/opt/homebrew
  USE_LIBSDL=1` on the make line. Without one of those the build fails at
  the OSD layer with `'SDL3/SDL.h' file not found`.

### Running tests

**Use the `mamellvm6309-headless` wrapper** for lit-friendly runs on macOS —
it combines `SDL_VIDEO_DRIVER=dummy` with `-video none -sound none` so MAME
produces no window, no focus steal, no audio side effects, and no
CoreAudio warnings. Output capture still works, everything else is
unchanged.

```sh
./mamellvm6309-headless /path/to/test.bin [timeout-seconds]
# stdout = serial output; $? = value written to EXIT_PORT ($FFD2)
```

The wrapper sets `SDL_VIDEO_DRIVER=dummy` in the environment before exec'ing
`./mamellvm6309`, and forwards `-video none -sound none -window -nothrottle`
plus the driver name `llvm6309` and the ROM/RS232 flags automatically.

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

- **`-rom file.bin`** — the image is loaded through a `generic_slot_device`
  as a raw binary of up to 64 KB. At `machine_start` the whole image is
  memcpy'd into the "mem" share, so byte N of the image ends up at
  address $N. No hash verification, no ZIP, no `rompath` — just hand it the
  file.
- **`-bitb <path>` requires a pre-existing file.** MAME's bitbanger device
  opens the file in read/write mode via `image_device::exists()`; if the
  file does not already exist, load fails with "No such file or directory"
  and no output is captured. The wrapper creates a temp file via `mktemp`
  each run, so this is handled automatically.
- **`-rs232 null_modem -bitb <path>`** — the driver restricts the RS232 slot
  options to `null_modem` only (see the "RS-232 workaround" section above),
  so this is the only sensible choice. `-bitb` captures all serial TX to the
  file. The driver also overrides the `null_modem` card's input-port defaults
  to 115200 baud 8N1 to match the ACIA clock; without that override, the
  null_modem would decode at its built-in 9600 baud default and produce
  bit-level garbage.
- **`-seconds_to_run N`** — safety net for runaway programs. Normal clean
  termination is via the `$FFD2` exit port — the last instruction of each
  test program stores a value (becomes the exit code) to `$FFD2` and MAME
  exits 2 ms later (enough time for the ACIA to drain its TX shift register).

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

Verified working minimal program — prints a greeting and exits with code 0:

```asm
ACIA_STATUS     equ     $ffd0
ACIA_DATA       equ     $ffd1
EXIT_PORT       equ     $ffd2
TRACE_CTL       equ     $ffd3
TDRE            equ     $02

                org     $0100           ; anywhere in RAM is fine — it's flat
reset           ldmd    #1              ; 6309 native mode (6309-only insn)
                lds     #$ffcf          ; stack just above the IO slab
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

done            clra                    ; exit code 0
                sta     EXIT_PORT       ; any write → schedule_exit() in 2 ms
halt            bra     halt

message         fcc     "Hello from HD6309"
                fcb     $0d,$0a,0

                org     $fffe
                fdb     reset
```

Assemble to a full 64K image — the whole thing is copied into RAM at boot:

```sh
lwasm --format=rawrel --6309 --output=test.bin test.asm
```

The `--format=rawrel` note is important: plain `--format=raw` does not handle
the forward `org $fffe` for the reset vector and produces a truncated file.
`rawrel` emits the entire 64K space, which the driver's cart loader accepts
directly — no `tail -c` slicing needed anymore now that the whole address
space is RAM-backed.

### Caveats / gotchas

- **The MC6850 ACIA must be initialised** — master reset (`$03` to control)
  then configure (`$15` for /16 8N1 no-IRQ). Without this, TDRE never
  asserts and the test program loops forever.
- **ACIA is polled only** — no IRQ line wired from the ACIA to the CPU.
  Test programs must poll TDRE/RDRF on `$FFD0`.
- **HD6309 native-mode switch** — the CPU boots in 6809-compatible mode.
  Use `LDMD #1` to enable the extra registers (E, F, V, W/Q) and 6309
  instructions.
- **The exit port defers exit by 2 ms**, not instantly, so the ACIA TX
  shift register can drain. If you genuinely want instant exit, remove
  the timer in `exit_w` — but you'll lose the trailing byte of every run.
- **Writes to `$FFF0-$FFFF` are silently dropped.** Reads return what was
  loaded from the cart image. If your program relies on self-modifying
  vectors, put them somewhere else in RAM.
- **`must_be_loaded`** — the cart slot is mandatory; MAME will refuse to
  start without `-rom` (the wrapper always supplies it).
- **Pre-create the bitbanger output file** or `-bitb` will fail to open it.
  The wrapper handles this via `mktemp`.
- **No software list** — this driver is deliberately raw. Nothing is wired
  into a `.hsi`/softlist file, and that's intentional (tests change every run).
- **Not upstream** — these files are local to this fork. Don't expect them
  to survive a rebase onto `mamedev/mame` without carrying the commit forward.
