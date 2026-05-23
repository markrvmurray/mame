// license:BSD-3-Clause
// copyright-holders:Mark Murray

// Minimal HD6309 single-board computer for validating the llvm-mc6809 backend.
//
// Memory map (top-to-bottom — $0000 at the top, $FFFF at the bottom). IO
// devices sit in the $FFCx zone, matching the Pico-Thing SBC's hardware
// layout so a debug-build firmware can target this driver and the
// `picothing` driver next door with the same MMIO addresses.
//
//   $0000-$FFC3   RAM (~64K contiguous — no IO wedge in the middle)
//   $FFC4-$FFC5   MC6850 ACIA (status, data) — matches PT console ACIA
//   $FFC6-$FFC9   RAM (no devices wired)
//   $FFCA         SystemWatchpoint control register (hardware-faithful)
//   $FFCB         TraceCtl (program-controlled host-side logging toggle)
//   $FFCC         Halt / exit port — emulator-only; PT has nothing here
//   $FFCD-$FFDF   RAM
//   $FFE0-$FFFF   SystemWatchpoint vector + snippet shadow (replaces the
//                 older write-protected vector ROM). 32-byte window owned
//                 by the watchpoint device; writable when disarmed.
//
// This layout mirrors `usim09batch` (see ../usim) so the same test programs
// run unchanged on both emulators. The previous map (IO at $FFD0-$FFD3,
// vectors at $FFF0-$FFFF write-protected by `.nopw()`) is gone.
//
// The image is loaded at runtime from a generic cartridge slot. Two
// formats are accepted:
//
//   * Raw binary up to 64 KB (typically a full 64 KB image produced by
//     `lwasm --format=rawrel` or `llvm-objcopy -O binary --pad-to=0x10000`).
//     The bytes are memcpy'd into RAM at boot — the bottom 32 bytes
//     ($FFE0-$FFFF) end up in the SystemWatchpoint shadow (vectors plus
//     reserved snippet area), the rest in RAM.
//
//   * ELF32 (the natural output of llvm-mc6809's lld and lwlink — the
//     llvm-mc6809 toolchain emits big-endian, lwlink either; both are
//     accepted). Each PT_LOAD program header with FileSiz > 0 is copied
//     into the 64 KB cart at its VMA. If the reset vector at $FFFE-$FFFF
//     is still zero after all segments are placed, it is patched with the
//     `_start` symbol's value (looked up in `.symtab`).
//
// Typical LLVM execution-test invocation (via the mamellvm6309-headless
// wrapper):
//
//   mamellvm6309-headless test.bin 2
//   FileCheck %s < /tmp/out.txt
//
// The test program ends with `sta $FFCC` which schedules a clean exit
// ~2 ms later (enough time for the ACIA TX shift register to drain),
// and the value stored becomes the process exit code.
//
// IRQ is intentionally not wired from the ACIA — polled TDRE/RDRF only.

#include "emu.h"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "bus/generic/carts.h"
#include "bus/generic/slot.h"
#include "bus/rs232/rs232.h"
#include "bus/rs232/null_modem.h"
#include "cpu/m6809/hd6309.h"
#include "machine/6850acia.h"
#include "machine/clock.h"


namespace {

class llvm6309_state : public driver_device
{
public:
	llvm6309_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
		, m_acia(*this, "acia")
		, m_cart(*this, "cartslot")
		, m_mem(*this, "mem")
	{ }

	void llvm6309(machine_config &config);

protected:
	virtual void machine_start() override ATTR_COLD;
	virtual void machine_reset() override ATTR_COLD;

private:
	// SystemWatchpoint constants — mirror usim's `system_watchpoint.h`.
	static constexpr u16 SWP_WINDOW_BASE   = 0xFFE0;
	static constexpr u16 SWP_WINDOW_SIZE   = 0x0020;
	static constexpr u16 SWP_NMI_OFFSET    = 0xFFFC - SWP_WINDOW_BASE;
	static constexpr u16 SWP_SNIPPET_LEN   = 0x10;
	static const u8      SWP_BREAK_SNIPPET[SWP_SNIPPET_LEN];

	void mem_map(address_map &map) ATTR_COLD;

	void halt_w(u8 data);
	void tracectl_w(u8 data);
	void swp_ctrl_w(u8 data);
	u8   swp_vec_r(offs_t offset);
	void swp_vec_w(offs_t offset, u8 data);

	void swp_arm();
	void swp_disarm();
	void swp_trigger(u16 offending_addr);

	TIMER_CALLBACK_MEMBER(do_exit);
	std::pair<std::error_condition, std::string> load_cart(device_image_interface &image);

	required_device<hd6309_device> m_maincpu;
	required_device<acia6850_device> m_acia;
	required_device<generic_slot_device> m_cart;
	required_shared_ptr<u8> m_mem;

	emu_timer *m_exit_timer = nullptr;
	u8 m_exit_code = 0;
	bool m_trace_enabled = false;

	u8   m_swp_shadow[SWP_WINDOW_SIZE]{};
	bool m_swp_armed = false;
	u8   m_swp_saved_nmi_hi = 0xff;
	u8   m_swp_saved_nmi_lo = 0xff;
};

// BREAK snippet — copied to $FFE0-$FFEF when the watchpoint is armed.
// Same 16 bytes as usim's `system_watchpoint.cpp`. The 6809 NMI auto-
// pushes the full register frame (PC,U,Y,X,DP,B,A,CC), and the snippet
// copies the 12-byte trapped frame to SHARED ($FFD0) and spins.
//
//   FFE0: 30 E4         LEAX 0,S
//   FFE2: CE FF D0      LDU  #$FFD0
//   FFE5: 86 0C         LDA  #12
//   FFE7: E6 80   loop: LDB  ,X+
//   FFE9: E7 C0         STB  ,U+
//   FFEB: 4A            DECA
//   FFEC: 26 F9         BNE  loop
//   FFEE: 20 FE   spin: BRA  spin
const u8 llvm6309_state::SWP_BREAK_SNIPPET[SWP_SNIPPET_LEN] = {
	0x30, 0xE4,
	0xCE, 0xFF, 0xD0,
	0x86, 0x0C,
	0xE6, 0x80,
	0xE7, 0xC0,
	0x4A,
	0x26, 0xF9,
	0x20, 0xFE
};

void llvm6309_state::machine_start()
{
	m_exit_timer = timer_alloc(FUNC(llvm6309_state::do_exit), this);

	// Populate RAM from the cart image. The cart slot always allocates
	// exactly 64 KB (load_cart() pads .bin and lays out ELF PT_LOAD
	// segments into a fixed-size buffer). Copy the whole 64 KB into
	// the "mem" share — IO overlays in mem_map intercept the IO and
	// watchpoint ranges, so the bytes loaded into RAM at those
	// addresses are simply unused.
	u8 const *src = m_cart->get_rom_base();
	if (src != nullptr)
	{
		std::memcpy(&m_mem[0], src, 0x10000);
		// Seed the SystemWatchpoint shadow from the cart's vector
		// region. The reset vector at $FFFE-$FFFF is what tells the
		// CPU where to start; load_cart() either copied it from the
		// raw image or patched it from `_start` for ELFs with no
		// .vectors LOAD segment.
		std::memcpy(m_swp_shadow, src + SWP_WINDOW_BASE, SWP_WINDOW_SIZE);
	}

	save_item(NAME(m_exit_code));
	save_item(NAME(m_trace_enabled));
	save_item(NAME(m_swp_shadow));
	save_item(NAME(m_swp_armed));
	save_item(NAME(m_swp_saved_nmi_hi));
	save_item(NAME(m_swp_saved_nmi_lo));
}

void llvm6309_state::machine_reset()
{
	// CPU reset disarms the watchpoint (matches usim's ActiveDevice::reset).
	swp_disarm();
}

void llvm6309_state::halt_w(u8 data)
{
	// Any write here ends the emulation run cleanly. The actual exit is
	// deferred by ~2 ms so any byte still being shifted out of the ACIA's
	// TX shift register has time to reach the null_modem / bitbanger
	// before MAME tears down. Without this, the last character of the
	// output would be silently dropped.
	m_exit_code = data;
	m_exit_timer->adjust(attotime::from_msec(2));
}

void llvm6309_state::tracectl_w(u8 data)
{
	// Program-controlled host-side logging, matching usim's TraceCtl
	// encoding so the same test programs toggle trace output on both
	// emulators without an #ifdef:
	//   0x00  disable
	//   0x01  enable (brk-style markers via logerror)
	//   0x02  enable + verbose (CPU PC on every tracectl write)
	// Other values are no-ops. MAME's full per-instruction trace lives
	// in the built-in debugger (-debug -debugscript …); this device is
	// just a lightweight checkpoint that a test program can poke.
	switch (data)
	{
	case 0x00:
		m_trace_enabled = false;
		break;
	case 0x01:
		m_trace_enabled = true;
		logerror("TraceCtl: enabled at PC=%04X\n", m_maincpu->pc());
		break;
	case 0x02:
		m_trace_enabled = true;
		logerror("TraceCtl: verbose at PC=%04X A=%02X B=%02X X=%04X Y=%04X U=%04X S=%04X\n",
				m_maincpu->pc(),
				m_maincpu->state_int(M6809_A), m_maincpu->state_int(M6809_B),
				m_maincpu->state_int(M6809_X), m_maincpu->state_int(M6809_Y),
				m_maincpu->state_int(M6809_U), m_maincpu->state_int(M6809_S));
		break;
	default:
		if (m_trace_enabled)
			logerror("TraceCtl: %02X at PC=%04X\n", data, m_maincpu->pc());
		break;
	}
}

void llvm6309_state::swp_ctrl_w(u8 data)
{
	if (data)
		swp_arm();
	else
		swp_disarm();
}

u8 llvm6309_state::swp_vec_r(offs_t offset)
{
	if (offset < SWP_WINDOW_SIZE)
		return m_swp_shadow[offset];
	return 0xff;
}

void llvm6309_state::swp_vec_w(offs_t offset, u8 data)
{
	if (offset >= SWP_WINDOW_SIZE)
		return;

	if (m_swp_armed)
	{
		// Block the store; assert NMI; don't auto-disarm here. The CPU
		// is about to vector to the snippet at $FFE0 (the swapped NMI
		// vector points there); the snippet copies the trapped frame
		// to SHARED ($FFD0) and spins. Disarm only on explicit write
		// of 0 to $FFCA, or on machine_reset().
		swp_trigger(u16(SWP_WINDOW_BASE + offset));
		return;
	}

	m_swp_shadow[offset] = data;
}

void llvm6309_state::swp_arm()
{
	if (m_swp_armed)
		return; // idempotent

	// Save the current NMI vector from the shadow.
	m_swp_saved_nmi_hi = m_swp_shadow[SWP_NMI_OFFSET];
	m_swp_saved_nmi_lo = m_swp_shadow[SWP_NMI_OFFSET + 1];

	// Copy the BREAK snippet over $FFE0-$FFEF.
	std::memcpy(m_swp_shadow, SWP_BREAK_SNIPPET, SWP_SNIPPET_LEN);

	// Repoint the NMI vector to $FFE0.
	m_swp_shadow[SWP_NMI_OFFSET]     = u8(SWP_WINDOW_BASE >> 8);
	m_swp_shadow[SWP_NMI_OFFSET + 1] = u8(SWP_WINDOW_BASE & 0xff);

	m_swp_armed = true;
}

void llvm6309_state::swp_disarm()
{
	if (!m_swp_armed)
	{
		// Even when never armed, release NMI in case we're being
		// called from machine_reset() right after a trigger.
		m_maincpu->set_input_line(INPUT_LINE_NMI, CLEAR_LINE);
		return;
	}

	// Restore the saved NMI vector. Snippet bytes are intentionally
	// left in place — matches the hardware where the Pico's BREAK
	// snippet stays resident until the next firmware-load cycle.
	m_swp_shadow[SWP_NMI_OFFSET]     = m_swp_saved_nmi_hi;
	m_swp_shadow[SWP_NMI_OFFSET + 1] = m_swp_saved_nmi_lo;

	m_swp_armed = false;
	m_maincpu->set_input_line(INPUT_LINE_NMI, CLEAR_LINE);
}

void llvm6309_state::swp_trigger(u16 offending_addr)
{
	// Visibility: emulator-side dump so users see the trap before the
	// snippet copies the frame to SHARED. Doesn't affect guest state.
	logerror("SystemWatchpoint: write to $%04X blocked while armed; "
		"asserting NMI (PC=$%04X)\n",
		offending_addr, m_maincpu->pc());

	// Assert NMI. The CPU latches NMI on the falling edge and services
	// it once; subsequent writes to the protected window within the
	// same armed window won't re-fire until disarm releases NMI and a
	// fresh arm + trigger occurs.
	m_maincpu->set_input_line(INPUT_LINE_NMI, ASSERT_LINE);
}

TIMER_CALLBACK_MEMBER(llvm6309_state::do_exit)
{
	// Write the exit-port value to a file so the wrapper script can use it
	// as the process exit code. The env var is set by mamellvm6309-headless.
	const char *path = getenv("LLVM6309_EXIT_FILE");
	if (path)
	{
		FILE *f = fopen(path, "w");
		if (f)
		{
			fprintf(f, "%d\n", m_exit_code);
			fclose(f);
		}
	}

	// Bug #194 Phase C: cycle counting parity with usim09batch.
	// usim emits `cycles=<N>` to stderr when --cycles is passed; the
	// picolibc bench harness (run-mc6809-tests) parses that line and
	// records it in the SQLite ledger. Mirror the same surface from
	// MAME by writing the HD6309's total_cycles() to a sidecar file
	// when LLVM6309_CYCLES_FILE is set; the headless wrapper reads
	// it post-exit and emits `cycles=<N>` to stderr — the existing
	// CYCLES_RE regex in run-mc6809-tests picks it up unchanged.
	const char *cyc_path = getenv("LLVM6309_CYCLES_FILE");
	if (cyc_path)
	{
		FILE *cf = fopen(cyc_path, "w");
		if (cf)
		{
			fprintf(cf, "%llu\n",
				(unsigned long long)m_maincpu->total_cycles());
			fclose(cf);
		}
	}

	machine().schedule_exit();
}

// Helpers for the inline ELF32 parser. The data byte-order is taken
// from e_ident[EI_DATA]: 1 = ELFDATA2LSB, 2 = ELFDATA2MSB. The
// llvm-mc6809 toolchain emits BE (matching the M6809's natural byte
// order for multi-byte values); USim's lwasm/lwlink emits BE too.
// We support both for portability.
static inline u16 read_le16(const u8 *p) { return u16(p[0]) | (u16(p[1]) << 8); }
static inline u32 read_le32(const u8 *p)
{
	return u32(p[0]) | (u32(p[1]) << 8) | (u32(p[2]) << 16) | (u32(p[3]) << 24);
}
static inline u16 read_be16(const u8 *p) { return (u16(p[0]) << 8) | u16(p[1]); }
static inline u32 read_be32(const u8 *p)
{
	return (u32(p[0]) << 24) | (u32(p[1]) << 16) | (u32(p[2]) << 8) | u32(p[3]);
}

// Lay out an ELF32 image into the 64 KB cart buffer. Walks PT_LOAD
// program headers, memcpy's each segment to its VMA, then patches the
// reset vector at $FFFE-$FFFF from `_start` if it's still zero. The
// cart is assumed pre-zeroed by the caller.
static std::pair<std::error_condition, std::string>
load_elf_into_cart(const u8 *elf, u32 elf_size, u8 *cart)
{
	if (elf_size < 52)
		return std::make_pair(image_error::INVALIDIMAGE, "ELF: header truncated");

	// e_ident sanity: ELFCLASS32 (1), data byte-order (1 LSB / 2 MSB)
	if (elf[4] != 1)
		return std::make_pair(image_error::INVALIDIMAGE, "ELF: not ELFCLASS32");
	bool const be = (elf[5] == 2);
	if (!be && elf[5] != 1)
		return std::make_pair(image_error::INVALIDIMAGE, "ELF: invalid EI_DATA");
	auto rd16 = [be](const u8 *p) { return be ? read_be16(p) : read_le16(p); };
	auto rd32 = [be](const u8 *p) { return be ? read_be32(p) : read_le32(p); };

	u32 const e_phoff     = rd32(elf + 28);
	u16 const e_phentsize = rd16(elf + 42);
	u16 const e_phnum     = rd16(elf + 44);
	u32 const e_shoff     = rd32(elf + 32);
	u16 const e_shentsize = rd16(elf + 46);
	u16 const e_shnum     = rd16(elf + 48);

	if (e_phentsize < 32 || e_shentsize < 40)
		return std::make_pair(image_error::INVALIDIMAGE, "ELF: unexpected header sizes");

	// Walk PT_LOAD program headers and place them in the cart.
	for (u16 i = 0; i < e_phnum; i++)
	{
		u32 const phoff = e_phoff + u32(i) * e_phentsize;
		if (phoff + 32 > elf_size)
			return std::make_pair(image_error::INVALIDIMAGE, "ELF: truncated program header");
		const u8 *ph = elf + phoff;

		u32 const p_type   = rd32(ph + 0);
		u32 const p_offset = rd32(ph + 4);
		u32 const p_vaddr  = rd32(ph + 8);
		u32 const p_filesz = rd32(ph + 16);

		if (p_type != 1 /* PT_LOAD */ || p_filesz == 0)
			continue;

		if (p_vaddr >= 0x10000 || p_filesz > 0x10000 || p_vaddr + p_filesz > 0x10000)
			return std::make_pair(image_error::INVALIDLENGTH,
				"ELF: PT_LOAD segment escapes 64 KB cart");
		if (p_offset > elf_size || p_offset + p_filesz > elf_size)
			return std::make_pair(image_error::INVALIDIMAGE, "ELF: PT_LOAD past EOF");

		std::memcpy(cart + p_vaddr, elf + p_offset, p_filesz);
	}

	// Reset vector at $FFFE-$FFFF: if still zero (no .vectors LOAD in
	// this ELF — typical for picolibc), patch from `_start`. Mirrors
	// the llvm-nm fallback in the legacy run-mc6809-mame wrapper.
	if (cart[0xfffe] == 0 && cart[0xffff] == 0 && e_shoff != 0 && e_shnum != 0)
	{
		// Find SHT_SYMTAB (sh_type == 2). At most one in a normal ELF.
		u32 symtab_off = 0, symtab_size = 0, symtab_link = 0, symtab_entsize = 0;
		for (u16 i = 0; i < e_shnum; i++)
		{
			u32 const shoff = e_shoff + u32(i) * e_shentsize;
			if (shoff + 40 > elf_size)
				break;
			const u8 *sh = elf + shoff;
			u32 const sh_type = rd32(sh + 4);
			if (sh_type == 2 /* SHT_SYMTAB */)
			{
				symtab_off     = rd32(sh + 16);
				symtab_size    = rd32(sh + 20);
				symtab_link    = rd32(sh + 24);
				symtab_entsize = rd32(sh + 36);
				break;
			}
		}

		if (symtab_off != 0 && symtab_entsize >= 16
				&& symtab_off + symtab_size <= elf_size
				&& symtab_link != 0 && symtab_link < e_shnum)
		{
			// Resolve the linked .strtab so we can match symbol names.
			u32 const strtab_shoff = e_shoff + u32(symtab_link) * e_shentsize;
			u32 strtab_off = 0, strtab_size = 0;
			if (strtab_shoff + 40 <= elf_size)
			{
				const u8 *sh = elf + strtab_shoff;
				strtab_off  = rd32(sh + 16);
				strtab_size = rd32(sh + 20);
			}

			if (strtab_off != 0 && strtab_off + strtab_size <= elf_size)
			{
				const char *strtab = reinterpret_cast<const char *>(elf + strtab_off);
				for (u32 off = 0; off + 16 <= symtab_size; off += symtab_entsize)
				{
					const u8 *sym = elf + symtab_off + off;
					u32 const st_name  = rd32(sym + 0);
					u32 const st_value = rd32(sym + 4);
					if (st_name == 0 || st_name >= strtab_size)
						continue;
					if (std::strcmp(strtab + st_name, "_start") != 0)
						continue;
					if (st_value < 0x10000)
					{
						cart[0xfffe] = u8((st_value >> 8) & 0xff);
						cart[0xffff] = u8(st_value & 0xff);
					}
					break;
				}
			}
		}
	}

	return std::make_pair(std::error_condition(), std::string());
}

std::pair<std::error_condition, std::string> llvm6309_state::load_cart(device_image_interface &image)
{
	generic_slot_device &slot = downcast<generic_slot_device &>(image.device());
	u32 const size = slot.common_get_size("rom");
	if (size == 0)
		return std::make_pair(image_error::INVALIDLENGTH, "Empty cartridge image");

	// Read the entire image into a temp buffer first so we can sniff
	// for ELF magic. Raw .bin/.rom carts are <= 64 KB; ELFs may be
	// many MB once `.debug_*` sections are included, so we DON'T
	// reject on size up-front (only on the post-parse layout check).
	std::vector<u8> raw(size);
	slot.common_load_rom(&raw[0], size, "rom");

	// Allocate the 64 KB cart and zero-fill. ELF segments leave gaps
	// (e.g. between .text at $0100 and .vectors at $FFE0); the gaps
	// must be zero so the IO slab at $FFCx works correctly.
	slot.rom_alloc(0x10000, GENERIC_ROM8_WIDTH, ENDIANNESS_BIG);
	u8 *const cart = slot.get_rom_base();
	std::memset(cart, 0, 0x10000);

	// ELF magic: 0x7F 'E' 'L' 'F'.
	if (size >= 4 && raw[0] == 0x7f && raw[1] == 'E' && raw[2] == 'L' && raw[3] == 'F')
		return load_elf_into_cart(&raw[0], size, cart);

	// Raw binary (legacy .bin/.rom path).
	if (size > 0x10000)
		return std::make_pair(image_error::INVALIDLENGTH, "Cartridge image must be no larger than 64K");
	std::memcpy(cart, &raw[0], size);
	return std::make_pair(std::error_condition(), std::string());
}

void llvm6309_state::mem_map(address_map &map)
{
	// Unified ~64 KB RAM. Test programs see a flat contiguous address
	// space from $0000 down to $FFC3 with no IO wedge in the middle.
	map(0x0000, 0xffff).ram().share("mem");

	// IO slab in the $FFCx zone, matching pico-thing's hardware layout
	// so a debug-build firmware can target both this driver and the
	// `picothing` driver with the same MMIO addresses. These .rw()/.w()
	// entries override the RAM mapping for their ranges (later wins).
	map(0xffc4, 0xffc5).rw(m_acia, FUNC(acia6850_device::read), FUNC(acia6850_device::write));
	map(0xffca, 0xffca).w(FUNC(llvm6309_state::swp_ctrl_w));
	map(0xffcb, 0xffcb).w(FUNC(llvm6309_state::tracectl_w));
	map(0xffcc, 0xffcc).w(FUNC(llvm6309_state::halt_w));

	// SystemWatchpoint vector + snippet shadow at $FFE0-$FFFF — the
	// device owns the 32-byte window. Reads return shadow bytes; writes
	// either land in the shadow (disarmed) or are blocked and assert
	// NMI (armed). Replaces the older write-protected vector ROM.
	map(0xffe0, 0xffff).rw(FUNC(llvm6309_state::swp_vec_r), FUNC(llvm6309_state::swp_vec_w));
}

// null_modem defaults to 9600 baud. Our ACIA drives at 115200, so the
// null_modem's device_serial_interface would bit-decode garbage unless
// we match the rates. Override its input-port defaults here.
static DEVICE_INPUT_DEFAULTS_START( null_modem )
	DEVICE_INPUT_DEFAULTS( "RS232_RXBAUD",   0xff, RS232_BAUD_115200 )
	DEVICE_INPUT_DEFAULTS( "RS232_TXBAUD",   0xff, RS232_BAUD_115200 )
	DEVICE_INPUT_DEFAULTS( "RS232_DATABITS", 0xff, RS232_DATABITS_8 )
	DEVICE_INPUT_DEFAULTS( "RS232_PARITY",   0xff, RS232_PARITY_NONE )
	DEVICE_INPUT_DEFAULTS( "RS232_STOPBITS", 0xff, RS232_STOPBITS_1 )
DEVICE_INPUT_DEFAULTS_END

// Minimal RS232 slot option set — only null_modem. `default_rs232_devices`
// from bus/rs232/rs232.cpp pulls in every emulated terminal card (h19, ie15,
// swtpc8212, sun_kbd, votraxtnt, ...) which in turn reference video/sound/CPU
// devices we don't want in this subtarget, so we roll our own list. The
// subtarget build (see scripts/target/mame/llvm6309.lua) does not compile
// bus/rs232/rs232.cpp at all — a local stripped copy in
// llvm6309_rs232_core.cpp provides the bus core symbols without
// default_rs232_devices().
static void llvm6309_rs232_devices(device_slot_interface &device)
{
	device.option_add("null_modem", NULL_MODEM);   // TX → -bitb file (FileCheck)
}

void llvm6309_state::llvm6309(machine_config &config)
{
	// CPU: HD6309 at 2 MHz (arbitrary — pick something the backend tests at).
	HD6309(config, m_maincpu, XTAL(8'000'000));  // /4 internally → 2 MHz E
	m_maincpu->set_addrmap(AS_PROGRAM, &llvm6309_state::mem_map);

	// ACIA (serial console). No IRQ wiring — polled only.
	ACIA6850(config, m_acia, 0);
	m_acia->txd_handler().set("rs232", FUNC(rs232_port_device::write_txd));

	// ACIA TX/RX clock = 1.8432 MHz → ÷16 in ACIA → 115200 baud.
	clock_device &acia_clock(CLOCK(config, "acia_clock", 1'843'200));
	acia_clock.signal_handler().set(m_acia, FUNC(acia6850_device::write_txc));
	acia_clock.signal_handler().append(m_acia, FUNC(acia6850_device::write_rxc));

	// RS232 slot — default to null_modem so `-bitb /path/to/out` works out
	// of the box for lit execution tests. Available options are limited to
	// null_modem / pty / loopback via llvm6309_rs232_devices above.
	rs232_port_device &rs232(RS232_PORT(config, "rs232", llvm6309_rs232_devices, "null_modem"));
	rs232.rxd_handler().set(m_acia, FUNC(acia6850_device::write_rxd));
	rs232.set_option_device_input_defaults("null_modem", DEVICE_INPUT_DEFAULTS_NAME(null_modem));

	// 64K raw-binary cart via generic cart slot — pass with `-rom file.bin`.
	// ELF32-LE images (the natural lld output) are accepted directly too;
	// load_cart() detects the magic and lays out PT_LOAD segments at their
	// VMAs. See bug #222.
	GENERIC_SOCKET(config, m_cart, generic_plain_slot, "llvm6309_rom", "bin,rom,elf");
	m_cart->set_device_load(FUNC(llvm6309_state::load_cart));
	m_cart->set_must_be_loaded(true);
}

ROM_START(llvm6309)
	// No baked ROM — the image comes from `-rom` at runtime.
ROM_END

} // anonymous namespace


//    YEAR  NAME      PARENT  COMPAT  MACHINE   INPUT  CLASS           INIT         COMPANY        FULLNAME                                 FLAGS
COMP( 2026, llvm6309, 0,      0,      llvm6309, 0,     llvm6309_state, empty_init,  "Mark Murray", "HD6309 LLVM backend validation SBC",    MACHINE_NO_SOUND_HW )
