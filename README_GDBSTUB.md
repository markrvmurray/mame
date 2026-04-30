# Debugging mamellvm6309 with lldb / gdb (gdbstub)

Bug #164 (llvm-mc6809) — Tier 1: assembly-level interactive
debugging via MAME's built-in gdb-RSP server, attached to over a
local TCP port.

## How to attach

Start MAME in debug mode:

    ./mamellvm6309-headless --debug=2159 path/to/cart.bin

The wrapper passes `-debugger gdbstub -debugger_host localhost
-debugger_port 2159 -debug` to MAME and skips the runaway-safety
`-seconds_to_run` (interactive sessions shouldn't time out). MAME
prints the listen address to stderr:

    mamellvm6309 listening for gdb-RSP on localhost:2159

It then **pauses execution until a client connects** — so the
process appears to hang on startup until you attach.

## Building a cart image from a picolibc test ELF

The standard recipe (matches `mamellvm6309`'s 64 KB raw cart slot).
The raw binary offset must match the ELF's lowest LOAD VMA -- $0100 for
code-only tests, but lower (e.g. $0021) for tests with __directpage data.
The reset vector is NOT in any ELF LOAD segment and must be written explicitly.

    LLVM=/Users/markmurray/GitHub/llvm-mc6809/llvm/cmake-build-debug-system/bin
    ELF=/path/to/test.elf

    # Find lowest LOAD VMA -- byte offset into the 64K cart image
    LOW_VMA=$($LLVM/llvm-readelf -l "$ELF" | awk '/LOAD/{print $3}' | sort | head -1)
    OFFSET=$(printf '%d' "$LOW_VMA")

    $LLVM/llvm-objcopy -O binary --gap-fill=0x00 --pad-to=0x10000 "$ELF" /tmp/raw.bin
    dd if=/dev/zero of=/tmp/cart.bin bs=65536 count=1
    dd if=/tmp/raw.bin of=/tmp/cart.bin bs=1 seek=$OFFSET conv=notrunc

    # Write reset vector ($FFFE-$FFFF) -- _start address big-endian
    START=$($LLVM/llvm-nm "$ELF" | awk '/ _start$/{print $1}')
    printf "\x${START:4:2}\x${START:6:2}" | dd of=/tmp/cart.bin bs=1 seek=65534 conv=notrunc

## Attaching with lldb (preferred)

    lldb path/to/test.elf
    (lldb) gdb-remote localhost:2159
    (lldb) b main
    (lldb) c
    (lldb) si
    (lldb) register read
    (lldb) memory read 0x4000
    (lldb) process kill         # clean MAME exit
    (lldb) quit

## Attaching with gdb (interop spot-check)

    gdb path/to/test.elf
    (gdb) target remote :2159
    (gdb) b main
    (gdb) c
    (gdb) si
    (gdb) info registers
    (gdb) x/16xb 0x4000
    (gdb) kill
    (gdb) quit

## Host-side debugger support — RESOLVED (bug #164 Tier 1, 2026-04-29)

The in-tree lldb (built from the llvm-mc6809 fork) now includes full MC6809
architecture support:

- `ABIMC6809` plugin: register table (D, X, Y, U, S, PC, A, B, CC, DP),
  function-entry and default unwind plans
- `eCore_mc6809` in ArchSpec + core definition
- `ObjectFileELF` overrides for ELF32-container / 2-byte-pointer mismatch

Use the in-tree lldb, not the system lldb:

    LLDB=/Users/markmurray/GitHub/llvm-mc6809/llvm/cmake-build-debug-system/bin/lldb
    $LLDB path/to/test.elf
    (lldb) gdb-remote localhost:2159

The system `/usr/bin/lldb` still cannot load MC6809 ELF files.

## What changed under the hood

- **`src/osd/modules/debugger/debuggdbstub.cpp`**: added
  `hd6309` and `hd6309e` aliases to the
  `gdb_register_maps` table, both reusing
  `gdb_register_map_m6809`. Without these, MAME's gdbstub aborted
  with "cpuname hd6309 not found in gdb stub descriptions" the
  moment the llvm6309 driver tried to register its CPU. With them,
  the m6809 register layout (A, B, D, X, Y, U, PC, S, CC, DP) is
  exposed for HD6309 too. Trade-off: HD6309-only registers (E, F,
  W, V, MD) are not in the map and not visible via the protocol.
  Adding them is a future enhancement (~50 LoC, defines a new
  `gdb_register_map_hd6309`).

- **`mamellvm6309-headless`**: added `--debug[=PORT]` mode. Default
  port 2159 (IANA gdb). Drops `-seconds_to_run` to allow indefinite
  interactive sessions. The non-debug invocation is unchanged — the
  bench harness and test runners keep working without modification.
