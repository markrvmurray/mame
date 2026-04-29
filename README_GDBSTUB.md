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

The standard recipe (matches `mamellvm6309`'s 64 KB raw cart slot):

    LLVM=/Users/markmurray/GitHub/llvm-mc6809/llvm/cmake-build-debug-system/bin
    ELF=/path/to/test.elf
    $LLVM/llvm-objcopy -O binary --gap-fill=0 --pad-to=0x10000 "$ELF" /tmp/raw.bin
    dd if=/dev/zero of=/tmp/cart.bin bs=65536 count=1
    # Lift by lowest-LOAD-addr (typically $0100 for picolibc):
    dd if=/tmp/raw.bin of=/tmp/cart.bin bs=256 seek=1 conv=notrunc
    # Reset vector — _start address big-endian at $FFFE
    # (look up _start with `llvm-nm "$ELF" | grep _start`):
    printf '\xFF\xC0' | dd of=/tmp/cart.bin bs=1 seek=65534 conv=notrunc

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

## Known gap — host-side debugger needs MC6809 architecture support

MAME's gdbstub serves a `target.xml` that declares
`<architecture>m6809</architecture>`. For lldb / gdb to decode the
register-read response (`g` packet) and disassemble the memory
contents (`m` packet output), the **client** must have MC6809
architecture support compiled in.

**As of 2026-04-29**:

- macOS system `/usr/bin/lldb` (`lldb-2100.0.16.12`) connects and
  receives stop events (`SIGTRAP` after `b main`+`c`), but cannot
  load MC6809 ELF executables (`error: ... is not a valid
  executable`) and cannot decode registers/memory because no
  MC6809 plugin is bundled.
- Homebrew `gdb` (via `brew install gdb`) ships gdb-15.x with
  multi-architecture support and CAN consume the served target.xml
  on architectures it knows. **Status of m6809 support in upstream
  gdb is currently uncertain** — needs verification before relying
  on this path.

Workarounds while waiting for first-class lldb/gdb support:

- **Wire-level confirmation**: raw RSP packets via `nc`. MAME's
  gdbstub responds to `qSupported`, `?`, `qC`, etc. before any
  client knows the architecture. `g` (read registers) requires
  `qXfer:features:read:target.xml` to have been requested first —
  a real client does this automatically; raw nc must do it
  explicitly.

- **MAME's built-in debug UI**: drop `-debugger gdbstub` and use
  `-debug` alone. Brings up MAME's native debugger window with full
  HD6309 state inspection, breakpoints, watchpoints, source-file
  disassembly. Heavier but architecture-aware. This is the
  practical fallback today.

- **`tracectl` from inside the program**: poke `0x01` (or `0x02`
  for verbose) to `$FFD3` to enable per-instruction host-side
  trace logging. Lightweight, works through the headless wrapper.

A future-bug pass can address the host-side gap by either (a)
contributing a m6809 ABI plugin to upstream LLDB, or (b) wiring up
a custom gdb (e.g. via the `m6809-elf` target patches that exist
out of tree).

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
