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
| `$0000-$DFFF` | 56K      | RAM                                                    |
| `$E000-$E001` | 2 bytes  | MC6850 ACIA (serial console, polled — no IRQ hookup)   |
| `$E002`       | 1 byte   | Exit port (write-only; any write → deferred clean exit) |
| `$E003-$FFFF` | ~8K − 3  | ROM window, backed by a runtime `generic_slot_device`  |

- CPU: Hitachi HD6309 at an 8 MHz crystal (2 MHz E clock). Boots in 6809
  emulation mode — execute `LDMD #1` early to engage native mode.
- ACIA clock: 1.8432 MHz → ÷16 → 115200 baud. The RS232 slot's `null_modem`
  card has its input-port defaults overridden to match (115200 / 8N1); without
  that override it would decode bit-level garbage at its built-in 9600 baud
  default.
- The ACIA and exit-port mappings are defined *after* the ROM mapping in the
  same `address_map`, so they overlay the first 3 bytes of the ROM window
  (later-wins). Your cart binary is a full 8K image; the first 3 bytes are
  wasted — start your code at `$E010` (or anywhere ≥ `$E003`) and leave those
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

Canonical lit-friendly invocation — cart file in, serial output to file,
auto-exit, output captured by FileCheck:

```sh
: > /tmp/out.txt                     # IMPORTANT: pre-create empty file, see below
./mamellvm6309 llvm6309 \
    -rom /path/to/test.bin \
    -rs232 null_modem -bitb /tmp/out.txt \
    -window -nothrottle -seconds_to_run 2
FileCheck %s < /tmp/out.txt
```

- **`-rom file.bin`** — the ROM image is loaded through a
  `generic_slot_device`. No hash verification, no ZIP, no `rompath` — just
  hand it a raw 8K binary. Max size 8K; anything larger is rejected. The CLI
  option name is `-rom` (the media name MAME exposes for the cart slot), not
  `-cart`.
- **`-bitb /tmp/out.txt` requires a pre-existing file.** MAME's bitbanger
  device opens the file in read/write mode via `image_device::exists()`; if
  the file does not already exist, load fails with "No such file or directory"
  and no output is captured. `: > /tmp/out.txt` (or `touch`) creates an empty
  file before each run.
- **`-rs232 null_modem -bitb <path>`** — the driver defaults the RS232 slot
  to `null_modem`, so `-bitb` captures all serial TX to the file. The driver
  also overrides the `null_modem` card's input-port defaults to 115200 baud
  8N1 to match the ACIA clock; without this, the null_modem decodes at its
  built-in 9600 baud default and produces bit-level garbage.
- **`-window -nothrottle`** — macOS MAME always wants a window for normal
  drivers (there's no true `-video none` for non-validation runs). A small
  window pops up for ~2 seconds per run. `-nothrottle` removes the frame-rate
  cap so emulation runs as fast as the host allows.
- **`-seconds_to_run N`** — safety net for runaway programs. Normal clean
  termination is via the `$E002` exit port — the last instruction of each
  test program stores any value to `$E002` and MAME exits 2 ms later (enough
  time for the ACIA to drain its TX shift register).

### Test program skeleton (lwasm)

Verified working minimal program — prints `Hello from HD6309\r\n` and exits:

```asm
ACIA_STATUS     equ     $e000
ACIA_DATA       equ     $e001
EXIT_PORT       equ     $e002
TDRE            equ     $02

                org     $e000
                fcb     0,0,0           ; overlaid by ACIA + exit port

                org     $e010
reset           ldmd    #1              ; 6309 native mode (6309-only insn)
                lds     #$dfff
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

Assemble and slice the bottom 8K off the full 64K image:

```sh
lwasm --format=rawrel --6309 --output=test.full test.asm
tail -c 8192 test.full > test.bin
```

The `--format=rawrel` note is important: plain `--format=raw` does not handle
the forward `org $fffe` for the reset vector and produces a truncated file.
`rawrel` emits the entire 64K space; `tail -c 8192` keeps only the ROM window
at `$E000-$FFFF`.

### Caveats / gotchas

- **The MC6850 ACIA must be initialised** — master reset (`$03` to control)
  then configure (`$15` for /16 8N1 no-IRQ). Without this, TDRE never
  asserts and the test program loops forever.
- **3 bytes of ROM are unreachable** (`$E000-$E002`) because the ACIA and
  exit port overlay them. Don't put code or data there.
- **ACIA is polled only** — no IRQ line wired from the ACIA to the CPU.
  Test programs must poll TDRE/RDRF on `$E000`.
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
