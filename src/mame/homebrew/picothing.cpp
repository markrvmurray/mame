// license:BSD-3-Clause
// copyright-holders:Mark Murray

// Pico-Thing MC6809 SBC — MAME driver.
//
// This driver mirrors the `usim09pt` reference emulator (see
// ../usim/main_picothing.cpp) and closely matches the real Pico-Thing
// hardware. It is the heavyweight sibling of `llvm6309`: same IO
// layout in the $FFCx zone, plus a DAT MMU, 2 MB of paged RAM, dual
// MC6850 ACIAs, a 50 Hz tick timer, file-backed FRAM, and a stripped
// PATA IDE controller.
//
// Memory map (top-to-bottom — $0000 at the top, $FFFF at the bottom):
//
//   $0000-$DFFF   DAT-translated RAM (2 MB physical, 8 KB pages, 32 tasks)
//   $E000-$FDFF   Fixed never-remapped window into physical page $FF
//                 ($1FE000-$1FFDFF — the last 8 KB of the 2 MB store).
//                 Hardware bypasses the DAT table for this range; the
//                 board ROM and OS-9 use it for cross-task globals.
//   $FE00-$FEFF   DAT page table (256 bytes — 32 tasks × 8 entries)
//   $FF00-$FF09   PATA IDE controller (file-backed disk image; multi-sector)
//   $FFC0         DAT task register (writes set the 5-bit active task;
//                                    reads return the current task number)
//   $FFC4-$FFC5   Console MC6850 ACIA (status, data)
//   $FFC6-$FFC7   Auxiliary MC6850 ACIA (null peer — discards TX)
//   $FFC8-$FFC9   PicoTick (cycle-driven 50 Hz default; PICOTHING_TICK env)
//   $FFCA         SystemWatchpoint control register
//   $FFCB         TraceCtl (program-controlled host-side logging toggle)
//   $FFD0-$FFDF   FRAM SHARED area (16 bytes, file-backed)
//   $FFE0-$FFFF   SystemWatchpoint vector + snippet shadow (also file-backed
//                 via the same .fram file — bytes 16..47 of the 48-byte file)
//
// DAT-table entry value $FF is a "page unavailable" sentinel; any guest
// access through a $FF entry asserts NMI and the read returns $FF / the
// write is dropped. Tasks must not place $FF in their DAT table; the
// fixed window at $E000-$FDFF references physical page $FF directly.
//
// Unlike `llvm6309`, this driver has no halt port — Pico-Thing firmware
// has no host-exit mechanism. The emulator runs until interrupted,
// until the watchpoint asserts NMI and the snippet spins, or until
// `-seconds_to_run` caps the run.
//
// IRQ wiring: console ACIA, aux ACIA, and the 50 Hz tick all share the
// CPU's IRQ line (wired-OR via input_merger_device). The watchpoint
// drives NMI. FIRQ is unused. This matches real Pico-Thing routing.
//
// Firmware loading: the cart slot accepts raw 64 KB binary, Intel HEX,
// Motorola S-record, or ELF32. Contents are distributed to:
//   * $0000-$FEFF → DAT physical store (task 0 identity-mapped at boot)
//   * $FFD0-$FFDF → FRAM
//   * $FFE0-$FFFF → SystemWatchpoint shadow (vectors + reserved snippet)
//
// File-backed sidecars are addressed via env vars (so the headless
// wrapper can wire them without MAME's image_device plumbing):
//   PICOTHING_DISK_FILE   PATA IDE backing file (optional)
//   PICOTHING_FRAM_FILE   FRAM persistence file (default: picothing.fram)

#include "emu.h"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "bus/generic/carts.h"
#include "bus/generic/slot.h"
#include "bus/rs232/rs232.h"
#include "bus/rs232/null_modem.h"
#include "cpu/m6809/m6809.h"
#include "machine/6850acia.h"
#include "machine/clock.h"
#include "machine/input_merger.h"


namespace {

class picothing_state : public driver_device
{
public:
	picothing_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
		, m_acia_con(*this, "acia_con")
		, m_acia_aux(*this, "acia_aux")
		, m_irq_merger(*this, "irq_merger")
		, m_cart(*this, "cartslot")
	{ }

	void picothing(machine_config &config);

protected:
	virtual void machine_start() override ATTR_COLD;
	virtual void machine_reset() override ATTR_COLD;

private:
	// Pico-Thing sizing constants.
	static constexpr u32 DAT_VIRT_SIZE    = 0xFE00;       // $0000-$FDFF
	static constexpr u32 DAT_PAGE_SIZE    = 0x2000;       // 8 KB
	static constexpr u32 DAT_PAGES_PER_TASK = 8;          // 8 entries × 8 KB = 64 KB virt
	static constexpr u32 DAT_TASKS        = 32;
	static constexpr u32 DAT_TABLE_SIZE   = DAT_TASKS * DAT_PAGES_PER_TASK; // 256
	static constexpr u32 DAT_PHYS_SIZE    = 2 * 1024 * 1024;

	static constexpr u32 FRAM_BASE        = 0xFFD0;
	static constexpr u32 FRAM_SIZE        = 0x0010;     // SHARED area only

	static constexpr u16 SWP_WINDOW_BASE  = 0xFFE0;
	static constexpr u16 SWP_WINDOW_SIZE  = 0x0020;
	static constexpr u16 SWP_NMI_OFFSET   = 0xFFFC - SWP_WINDOW_BASE;
	static constexpr u16 SWP_SNIPPET_LEN  = 0x10;
	static const u8 SWP_BREAK_SNIPPET[SWP_SNIPPET_LEN];

	// On-disk FRAM is SHARED (16) + vector-shadow (32) = 48 bytes,
	// matching usim09pt's persistent backing. The vector-shadow bytes
	// at file offset 16..47 are tee'd through from m_swp_shadow on
	// every shadow mutation (arm / disarm / write).
	static constexpr u32 FRAM_FILE_SIZE   = FRAM_SIZE + SWP_WINDOW_SIZE;

	// PicoTick: default 50 Hz = 20000 cycles at 1 MHz E. PICOTHING_TICK
	// env var overrides for tick-rate sweeps (matches usim's --tick=N).
	// The emu_timer fires at attotime::from_ticks(N, cpu_clock) which is
	// equivalent to N CPU cycles at the constant 1 MHz E rate.
	static constexpr unsigned PICOTICK_DEFAULT_PERIOD = 20000;

	// Fixed never-remapped window: $E000-$FDFF -> physical page $FF
	// (last 8KB of the 2MB store at $1FE000-$1FFDFF). Board logic
	// hardwires this — accesses bypass the DAT table entirely. The
	// DAT-table value $FF is reserved as a "page unavailable" sentinel
	// (any access through it traps NMI).
	static constexpr u32 DAT_FIXED_WINDOW_BASE = 0xE000;
	static constexpr u32 DAT_FIXED_PHYS_BASE   = 0x1FE000;
	static constexpr u8  DAT_PAGE_UNAVAILABLE  = 0xFF;

	void mem_map(address_map &map) ATTR_COLD;

	// DAT
	u8   dat_r(offs_t offset);
	void dat_w(offs_t offset, u8 data);
	u8   dat_table_r(offs_t offset);
	void dat_table_w(offs_t offset, u8 data);
	u8   picotask_r();
	void picotask_w(u8 data);
	void dat_unavailable_trap(offs_t offset, u8 page_idx, bool is_write);

	// PicoTick
	u8   picotick_r(offs_t offset);
	void picotick_w(offs_t offset, u8 data);
	TIMER_CALLBACK_MEMBER(picotick_fire);

	// FRAM
	u8   fram_r(offs_t offset);
	void fram_w(offs_t offset, u8 data);
	void fram_save();
	void fram_load();

	// IDE
	u8   ide_r(offs_t offset);
	void ide_w(offs_t offset, u8 data);
	void ide_do_read();
	void ide_do_write();
	void ide_do_identify();
	void ide_complete_write();
	void ide_advance_lba();
	u32  ide_get_lba() const;

	// SystemWatchpoint
	void swp_ctrl_w(u8 data);
	u8   swp_vec_r(offs_t offset);
	void swp_vec_w(offs_t offset, u8 data);
	void swp_arm();
	void swp_disarm();
	void swp_trigger(u16 offending_addr);

	// TraceCtl
	void tracectl_w(u8 data);

	// Stage C: non-perturbing instrumentation.
	struct addr_spec {
		bool physical;
		u32  base;
		u32  len;       // for watch: 1; for wlog/dump: user-specified
		u8   last_value;
	};
	static bool parse_addr_spec(const std::string &s, addr_spec &out, u32 default_len);
	static std::vector<std::string> split_specs(const char *env, char sep);
	u8   peek_logical(u16 addr) const;
	u8   peek_physical(u32 phys) const;
	void on_physical_write(u32 phys, u8 old_val, u8 new_val, offs_t logical);
	void on_logical_write(offs_t logical, u8 old_val, u8 new_val);
	void install_instrumentation_taps();
	void instrumentation_dump_all(const char *trigger);
	void instrumentation_dump_one(const addr_spec &spec);
	void instrumentation_emit_cycles();
	u64  cur_cycles() const { return m_maincpu->total_cycles(); }
	u16  cur_pc() const { return u16(m_maincpu->state_int(M6809_PC) & 0xffff); }
	void machine_exit_notify();

	// Firmware loading
	std::pair<std::error_condition, std::string> load_cart(device_image_interface &image);
	bool parse_intel_hex(const u8 *src, u32 size, u8 *out64k);
	bool parse_srec(const u8 *src, u32 size, u8 *out64k);

	required_device<mc6809e_device> m_maincpu;
	required_device<acia6850_device> m_acia_con;
	required_device<acia6850_device> m_acia_aux;
	required_device<input_merger_device> m_irq_merger;
	required_device<generic_slot_device> m_cart;

	// DAT MMU state
	std::unique_ptr<u8[]> m_dat_phys;        // 2 MB physical store
	u8 m_dat_table[DAT_TABLE_SIZE]{};        // 256-byte page table
	u8 m_dat_task = 0;                       // active task (0-31)

	// PicoTick state
	u8 m_tick_ctrl = 0;
	u8 m_tick_status = 0;
	u32 m_tick_period = PICOTICK_DEFAULT_PERIOD;
	emu_timer *m_tick_timer = nullptr;

	// FRAM state
	u8 m_fram[FRAM_SIZE]{};
	std::string m_fram_path;

	// IDE state
	FILE *m_ide_disk = nullptr;
	u8 m_ide_buffer[512]{};
	u16 m_ide_buf_ptr = 0;
	bool m_ide_writing = false;
	u8 m_ide_error = 0;
	u8 m_ide_features = 0;
	u8 m_ide_sector_count = 1;
	u16 m_ide_sectors_remaining = 0;     // 0=256 expansion already applied
	u8 m_ide_lba_low = 0;
	u8 m_ide_lba_mid = 0;
	u8 m_ide_lba_high = 0;
	u8 m_ide_drive_head = 0xA0;
	u8 m_ide_status = 0x40;  // DRDY
	u8 m_ide_devctl = 0;

	static constexpr u8 IDE_SR_BSY  = 0x80;
	static constexpr u8 IDE_SR_DRDY = 0x40;
	static constexpr u8 IDE_SR_DRQ  = 0x08;
	static constexpr u8 IDE_SR_ERR  = 0x01;
	static constexpr u8 IDE_CMD_READ     = 0x20;
	static constexpr u8 IDE_CMD_WRITE    = 0x30;
	static constexpr u8 IDE_CMD_IDENTIFY = 0xEC;

	// SystemWatchpoint state
	u8 m_swp_shadow[SWP_WINDOW_SIZE]{};
	bool m_swp_armed = false;
	u8 m_swp_saved_nmi_hi = 0xff;
	u8 m_swp_saved_nmi_lo = 0xff;

	// TraceCtl state
	bool m_trace_enabled = false;

	// Stage C: non-perturbing instrumentation state.
	std::vector<addr_spec> m_watches;
	std::vector<addr_spec> m_wlogs;
	std::vector<addr_spec> m_dumps;
	std::string m_cycles_file_path;
	bool m_have_physical_taps = false;   // any p: spec in watches or wlogs?
};

// BREAK snippet — copied to $FFE0-$FFEF when the watchpoint is armed.
// Identical to llvm6309 / usim's `system_watchpoint.cpp`.
const u8 picothing_state::SWP_BREAK_SNIPPET[SWP_SNIPPET_LEN] = {
	0x30, 0xE4,                  // LEAX 0,S
	0xCE, 0xFF, 0xD0,            // LDU  #$FFD0
	0x86, 0x0C,                  // LDA  #12
	0xE6, 0x80,                  // LDB  ,X+
	0xE7, 0xC0,                  // STB  ,U+
	0x4A,                        // DECA
	0x26, 0xF9,                  // BNE  loop
	0x20, 0xFE                   // BRA  spin
};


//------------------------------------------------------------
// DAT MMU
//------------------------------------------------------------

void picothing_state::dat_unavailable_trap(offs_t offset, u8 page_idx, bool is_write)
{
	// usim memory.h: DAT-table entry $FF is the "page unavailable"
	// sentinel — any access through it asserts NMI so the firmware
	// can detect bad page references. Reads return $FF, writes drop.
	logerror("DAT: page-unavailable %s at $%04X (task=%u page_idx=$%02X)\n",
		is_write ? "write" : "read",
		offset, unsigned(m_dat_task), page_idx);
	m_maincpu->set_input_line(INPUT_LINE_NMI, ASSERT_LINE);
}

u8 picothing_state::dat_r(offs_t offset)
{
	// Fixed never-remapped window: $E000-$FDFF -> physical page $FF
	// (last 8 KB of the 2 MB store at $1FE000-$1FFDFF). Hardware
	// enforces this; the DAT table is not consulted for this range.
	if (offset >= DAT_FIXED_WINDOW_BASE)
	{
		return m_dat_phys[DAT_FIXED_PHYS_BASE + (offset - DAT_FIXED_WINDOW_BASE)];
	}

	// 5-bit task selects 8 entries of the 256-byte page table.
	// Each entry is a 1-byte physical-page index (0-255 → 8 KB chunks
	// of the 2 MB physical store).
	u8 const page_idx = (m_dat_task << 3) | (offset >> 13);
	u8 const phys_page = m_dat_table[page_idx];
	if (phys_page == DAT_PAGE_UNAVAILABLE)
	{
		dat_unavailable_trap(offset, page_idx, false);
		return 0xFF;
	}
	u32 const phys = (u32(phys_page) << 13) | (offset & 0x1FFF);
	return m_dat_phys[phys];
}

void picothing_state::dat_w(offs_t offset, u8 data)
{
	if (offset >= DAT_FIXED_WINDOW_BASE)
	{
		u32 const phys = DAT_FIXED_PHYS_BASE + (offset - DAT_FIXED_WINDOW_BASE);
		u8 const old = m_dat_phys[phys];
		m_dat_phys[phys] = data;
		if (m_have_physical_taps && (old != data || !m_watches.empty() || !m_wlogs.empty()))
			on_physical_write(phys, old, data, offset);
		return;
	}

	u8 const page_idx = (m_dat_task << 3) | (offset >> 13);
	u8 const phys_page = m_dat_table[page_idx];
	if (phys_page == DAT_PAGE_UNAVAILABLE)
	{
		dat_unavailable_trap(offset, page_idx, true);
		return;     // store is dropped
	}
	u32 const phys = (u32(phys_page) << 13) | (offset & 0x1FFF);
	u8 const old = m_dat_phys[phys];
	m_dat_phys[phys] = data;
	if (m_have_physical_taps && (old != data || !m_watches.empty() || !m_wlogs.empty()))
		on_physical_write(phys, old, data, offset);
}

u8 picothing_state::dat_table_r(offs_t offset)
{
	return (offset < DAT_TABLE_SIZE) ? m_dat_table[offset] : 0xff;
}

void picothing_state::dat_table_w(offs_t offset, u8 data)
{
	if (offset < DAT_TABLE_SIZE)
		m_dat_table[offset] = data;
}

u8 picothing_state::picotask_r()
{
	// usim: PicoTask reads return the current task number. Firmware
	// pokes the read register to learn which context is active
	// (required by NitrOS-9 SWICall).
	return m_dat_task;
}

void picothing_state::picotask_w(u8 data)
{
	// 5 bits — selects one of 32 task contexts.
	m_dat_task = data & 0x1F;
}


//------------------------------------------------------------
// PicoTick (50 Hz)
//------------------------------------------------------------

u8 picothing_state::picotick_r(offs_t offset)
{
	switch (offset)
	{
	case 0:
		return m_tick_ctrl;
	case 1:
	{
		// Reading status clears IRQ-pending and releases the IRQ line.
		u8 const v = m_tick_status;
		m_tick_status &= ~0x80;
		m_irq_merger->in_w<2>(0);
		return v;
	}
	default:
		return 0xff;
	}
}

void picothing_state::picotick_w(offs_t offset, u8 data)
{
	switch (offset)
	{
	case 0:
		m_tick_ctrl = data;
		if (!(m_tick_ctrl & 0x01))
		{
			m_tick_status &= ~0x80;
			m_irq_merger->in_w<2>(0);
		}
		break;
	case 1:
		// status is read-only
		break;
	}
}

TIMER_CALLBACK_MEMBER(picothing_state::picotick_fire)
{
	if (!(m_tick_ctrl & 0x01))
		return;
	m_tick_status |= 0x80;
	m_irq_merger->in_w<2>(1);
}


//------------------------------------------------------------
// FRAM
//------------------------------------------------------------

u8 picothing_state::fram_r(offs_t offset)
{
	return (offset < FRAM_SIZE) ? m_fram[offset] : 0xff;
}

void picothing_state::fram_w(offs_t offset, u8 data)
{
	if (offset >= FRAM_SIZE)
		return;
	m_fram[offset] = data;
	fram_save();
}

void picothing_state::fram_load()
{
	if (m_fram_path.empty())
		return;
	FILE *fp = std::fopen(m_fram_path.c_str(), "rb");
	if (!fp)
		return;
	std::fseek(fp, 0, SEEK_END);
	long const sz = std::ftell(fp);
	std::fseek(fp, 0, SEEK_SET);
	// Backward-compat: 16-byte files (old MAME format) load only SHARED;
	// 48-byte files (matches usim09pt) load SHARED + vector shadow.
	if (sz >= long(FRAM_SIZE))
		(void)std::fread(m_fram, 1, FRAM_SIZE, fp);
	if (sz >= long(FRAM_FILE_SIZE))
		(void)std::fread(m_swp_shadow, 1, SWP_WINDOW_SIZE, fp);
	std::fclose(fp);
}

void picothing_state::fram_save()
{
	if (m_fram_path.empty())
		return;
	FILE *fp = std::fopen(m_fram_path.c_str(), "wb");
	if (!fp)
		return;
	(void)std::fwrite(m_fram, 1, FRAM_SIZE, fp);
	(void)std::fwrite(m_swp_shadow, 1, SWP_WINDOW_SIZE, fp);
	std::fclose(fp);
}


//------------------------------------------------------------
// PATA IDE — stripped per usim's picoide.cpp
//------------------------------------------------------------

u32 picothing_state::ide_get_lba() const
{
	return (u32(m_ide_drive_head & 0x0F) << 24)
		| (u32(m_ide_lba_high) << 16)
		| (u32(m_ide_lba_mid) << 8)
		| u32(m_ide_lba_low);
}

void picothing_state::ide_do_read()
{
	if (!m_ide_disk)
	{
		m_ide_status = IDE_SR_DRDY | IDE_SR_ERR;
		m_ide_error = 0x04;
		return;
	}
	// usim picoide.cpp:84-85: sector_count==0 means 256 sectors.
	m_ide_sectors_remaining = m_ide_sector_count ? m_ide_sector_count : 256;
	long const offset = long(ide_get_lba()) * 512;
	std::fseek(m_ide_disk, offset, SEEK_SET);
	size_t n = std::fread(m_ide_buffer, 1, 512, m_ide_disk);
	if (n < 512)
		std::memset(m_ide_buffer + n, 0, 512 - n);
	m_ide_buf_ptr = 0;
	m_ide_writing = false;
	m_ide_sectors_remaining--;
	m_ide_status = IDE_SR_DRDY | IDE_SR_DRQ;
}

void picothing_state::ide_do_write()
{
	m_ide_sectors_remaining = m_ide_sector_count ? m_ide_sector_count : 256;
	std::memset(m_ide_buffer, 0, sizeof(m_ide_buffer));
	m_ide_buf_ptr = 0;
	m_ide_writing = true;
	m_ide_status = IDE_SR_DRDY | IDE_SR_DRQ;
}

void picothing_state::ide_advance_lba()
{
	u32 const lba = ide_get_lba() + 1;
	m_ide_lba_low  = u8(lba & 0xff);
	m_ide_lba_mid  = u8((lba >> 8) & 0xff);
	m_ide_lba_high = u8((lba >> 16) & 0xff);
	m_ide_drive_head = (m_ide_drive_head & 0xf0) | u8((lba >> 24) & 0x0f);
}

void picothing_state::ide_complete_write()
{
	if (!m_ide_disk)
	{
		m_ide_status = IDE_SR_DRDY | IDE_SR_ERR;
		m_ide_error = 0x04;
		return;
	}
	long const offset = long(ide_get_lba()) * 512;
	std::fseek(m_ide_disk, offset, SEEK_SET);
	(void)std::fwrite(m_ide_buffer, 1, 512, m_ide_disk);
	std::fflush(m_ide_disk);
	if (m_ide_sectors_remaining > 0)
		m_ide_sectors_remaining--;
	if (m_ide_sectors_remaining > 0)
	{
		// Advance LBA and prep buffer for next sector — DRQ stays set.
		ide_advance_lba();
		std::memset(m_ide_buffer, 0, sizeof(m_ide_buffer));
		m_ide_buf_ptr = 0;
		m_ide_status = IDE_SR_DRDY | IDE_SR_DRQ;
	}
	else
	{
		m_ide_writing = false;
		m_ide_status = IDE_SR_DRDY;
	}
}

void picothing_state::ide_do_identify()
{
	std::memset(m_ide_buffer, 0, sizeof(m_ide_buffer));
	// Word 0: general config (fixed disk).
	m_ide_buffer[0] = 0x00; m_ide_buffer[1] = 0x40;
	// Cylinders 1024 / heads 16 / sectors 63.
	m_ide_buffer[2] = 0x04; m_ide_buffer[3] = 0x00;
	m_ide_buffer[6] = 0x00; m_ide_buffer[7] = 0x10;
	m_ide_buffer[8] = 0x00; m_ide_buffer[9] = 0x3F;
	// Words 27-46: model string ("PicoThing Virtual HD", 40 chars, swapped pairs).
	const char *model = "PicoThing Virtual HD                    ";
	for (int i = 0; i < 40; i += 2)
	{
		m_ide_buffer[54 + i]     = u8(model[i + 1]);
		m_ide_buffer[54 + i + 1] = u8(model[i]);
	}
	// Word 49: LBA supported.
	m_ide_buffer[98] = 0x02; m_ide_buffer[99] = 0x00;
	// Words 60-61: LBA28 total sectors (1 GB).
	m_ide_buffer[120] = 0x20; m_ide_buffer[121] = 0x00;
	m_ide_buffer[122] = 0x00; m_ide_buffer[123] = 0x00;
	m_ide_buf_ptr = 0;
	m_ide_writing = false;
	m_ide_status = IDE_SR_DRDY | IDE_SR_DRQ;
}

u8 picothing_state::ide_r(offs_t offset)
{
	switch (offset)
	{
	case 0:
		return (m_ide_status & IDE_SR_DRQ) ? m_ide_buffer[m_ide_buf_ptr] : 0xff;
	case 1:
		if (m_ide_status & IDE_SR_DRQ)
		{
			u8 v = m_ide_buffer[m_ide_buf_ptr + 1];
			m_ide_buf_ptr += 2;
			if (m_ide_buf_ptr >= 512)
			{
				// usim picoide.cpp: multi-sector READ — if more sectors
				// remain, advance LBA and refill the buffer; DRQ stays set.
				if (m_ide_sectors_remaining > 0 && m_ide_disk && !m_ide_writing)
				{
					ide_advance_lba();
					long const off = long(ide_get_lba()) * 512;
					std::fseek(m_ide_disk, off, SEEK_SET);
					size_t n = std::fread(m_ide_buffer, 1, 512, m_ide_disk);
					if (n < 512)
						std::memset(m_ide_buffer + n, 0, 512 - n);
					m_ide_buf_ptr = 0;
					m_ide_sectors_remaining--;
				}
				else
				{
					m_ide_status &= ~IDE_SR_DRQ;
					m_ide_status |= IDE_SR_DRDY;
				}
			}
			return v;
		}
		return 0xff;
	case 2: return m_ide_error;
	case 3: return m_ide_sector_count;
	case 4: return m_ide_lba_low;
	case 5: return m_ide_lba_mid;
	case 6: return m_ide_lba_high;
	case 7: return m_ide_drive_head;
	case 8: return m_ide_status;
	case 9: return m_ide_status;
	default: return 0xff;
	}
}

void picothing_state::ide_w(offs_t offset, u8 data)
{
	switch (offset)
	{
	case 0:
		if ((m_ide_status & IDE_SR_DRQ) && m_ide_writing)
			m_ide_buffer[m_ide_buf_ptr] = data;
		break;
	case 1:
		if ((m_ide_status & IDE_SR_DRQ) && m_ide_writing)
		{
			m_ide_buffer[m_ide_buf_ptr + 1] = data;
			m_ide_buf_ptr += 2;
			if (m_ide_buf_ptr >= 512)
				ide_complete_write();
		}
		break;
	case 2: m_ide_features = data; break;
	case 3: m_ide_sector_count = data; break;
	case 4: m_ide_lba_low = data; break;
	case 5: m_ide_lba_mid = data; break;
	case 6: m_ide_lba_high = data; break;
	case 7: m_ide_drive_head = data; break;
	case 8:
		m_ide_error = 0;
		switch (data)
		{
		case IDE_CMD_READ:     ide_do_read();     break;
		case IDE_CMD_WRITE:    ide_do_write();    break;
		case IDE_CMD_IDENTIFY: ide_do_identify(); break;
		default:
			m_ide_status = IDE_SR_DRDY | IDE_SR_ERR;
			m_ide_error = 0x04;
			break;
		}
		break;
	case 9: m_ide_devctl = data; break;
	}
}


//------------------------------------------------------------
// SystemWatchpoint
//------------------------------------------------------------

void picothing_state::swp_ctrl_w(u8 data)
{
	if (data)
		swp_arm();
	else
		swp_disarm();
}

u8 picothing_state::swp_vec_r(offs_t offset)
{
	return (offset < SWP_WINDOW_SIZE) ? m_swp_shadow[offset] : 0xff;
}

void picothing_state::swp_vec_w(offs_t offset, u8 data)
{
	if (offset >= SWP_WINDOW_SIZE)
		return;
	if (m_swp_armed)
	{
		swp_trigger(u16(SWP_WINDOW_BASE + offset));
		return;
	}
	m_swp_shadow[offset] = data;
	fram_save();    // tee through to persistent FRAM backing
}

void picothing_state::swp_arm()
{
	if (m_swp_armed)
		return;
	m_swp_saved_nmi_hi = m_swp_shadow[SWP_NMI_OFFSET];
	m_swp_saved_nmi_lo = m_swp_shadow[SWP_NMI_OFFSET + 1];
	std::memcpy(m_swp_shadow, SWP_BREAK_SNIPPET, SWP_SNIPPET_LEN);
	m_swp_shadow[SWP_NMI_OFFSET]     = u8(SWP_WINDOW_BASE >> 8);
	m_swp_shadow[SWP_NMI_OFFSET + 1] = u8(SWP_WINDOW_BASE & 0xff);
	m_swp_armed = true;
	fram_save();
}

void picothing_state::swp_disarm()
{
	if (m_swp_armed)
	{
		m_swp_shadow[SWP_NMI_OFFSET]     = m_swp_saved_nmi_hi;
		m_swp_shadow[SWP_NMI_OFFSET + 1] = m_swp_saved_nmi_lo;
		m_swp_armed = false;
		fram_save();
	}
	m_maincpu->set_input_line(INPUT_LINE_NMI, CLEAR_LINE);
}

void picothing_state::swp_trigger(u16 offending_addr)
{
	logerror("SystemWatchpoint: write to $%04X blocked while armed; "
		"asserting NMI (PC=$%04X)\n",
		offending_addr, m_maincpu->pc());
	m_maincpu->set_input_line(INPUT_LINE_NMI, ASSERT_LINE);
}


//------------------------------------------------------------
// TraceCtl
//------------------------------------------------------------

void picothing_state::tracectl_w(u8 data)
{
	switch (data)
	{
	case 0x00:
		m_trace_enabled = false;
		break;
	case 0x01:
		m_trace_enabled = true;
		logerror("TraceCtl: enabled at PC=%04X (task=%u)\n",
			m_maincpu->pc(), unsigned(m_dat_task));
		break;
	case 0x02:
		m_trace_enabled = true;
		logerror("TraceCtl: verbose at PC=%04X task=%u A=%02X B=%02X X=%04X Y=%04X U=%04X S=%04X\n",
			m_maincpu->pc(), unsigned(m_dat_task),
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


//------------------------------------------------------------
// Stage C: non-perturbing instrumentation
//
// --watch=[p:]ADDR        monitor a byte for changes (repeatable)
// --wlog=[p:]ADDR[,LEN]   log every store into range (repeatable)
// --dump=[p:]ADDR[,LEN]   hex+ASCII dump at exit (default LEN=256)
// --cycles                emit cycles=<N> to sidecar file at exit
//
// Specs come in via PICOTHING_WATCH / _WLOG / _DUMP (semicolon-
// separated lists set by the headless wrapper). Logical-address
// (default) writes are caught via address_space::install_write_tap
// on $0000-$FDFF; physical-address (p:) writes flow through the
// dat_w post-write hook. Either path uses peek_logical/peek_physical
// to read the old value without perturbing device state.
//------------------------------------------------------------

std::vector<std::string> picothing_state::split_specs(const char *env, char sep)
{
	std::vector<std::string> out;
	if (!env || !*env) return out;
	std::string cur;
	for (const char *p = env; *p; p++)
	{
		if (*p == sep)
		{
			if (!cur.empty()) { out.push_back(cur); cur.clear(); }
		}
		else cur.push_back(*p);
	}
	if (!cur.empty()) out.push_back(cur);
	return out;
}

bool picothing_state::parse_addr_spec(const std::string &s, addr_spec &out, u32 default_len)
{
	if (s.empty()) return false;
	std::string body = s;
	out.physical = false;
	if (body.size() > 2 && body[0] == 'p' && body[1] == ':')
	{
		out.physical = true;
		body = body.substr(2);
	}
	// Split on first ','.
	std::string addr_part = body, len_part;
	auto const comma = body.find(',');
	if (comma != std::string::npos)
	{
		addr_part = body.substr(0, comma);
		len_part  = body.substr(comma + 1);
	}
	// Parse hex or decimal — strtoul handles "0x.." and "0...".
	char *end = nullptr;
	unsigned long const a = std::strtoul(addr_part.c_str(), &end, 0);
	if (end == addr_part.c_str()) return false;
	out.base = u32(a);
	out.len = default_len;
	if (!len_part.empty())
	{
		unsigned long const n = std::strtoul(len_part.c_str(), &end, 0);
		if (end == len_part.c_str() || n == 0) return false;
		out.len = u32(n);
	}
	// Sanity bound: logical addrs in 64K, physical in 2 MB.
	u32 const max_addr = out.physical ? DAT_PHYS_SIZE : 0x10000;
	if (out.base >= max_addr) return false;
	if (out.base + out.len > max_addr) out.len = max_addr - out.base;
	out.last_value = 0;
	return true;
}

u8 picothing_state::peek_logical(u16 addr) const
{
	// Non-perturbing logical read — bypasses ACIA/IDE/tick/watchpoint
	// side effects. Used by instrumentation only.
	if (addr >= 0xFE00 && addr < 0xFF00)
		return m_dat_table[addr - 0xFE00];
	if (addr >= DAT_FIXED_WINDOW_BASE && addr < 0xFE00)
		return m_dat_phys[DAT_FIXED_PHYS_BASE + (addr - DAT_FIXED_WINDOW_BASE)];
	if (addr < DAT_FIXED_WINDOW_BASE)
	{
		u8 const page_idx = (m_dat_task << 3) | (addr >> 13);
		u8 const pp = m_dat_table[page_idx];
		if (pp == DAT_PAGE_UNAVAILABLE) return 0xff;
		return m_dat_phys[(u32(pp) << 13) | (addr & 0x1fff)];
	}
	// IO zone $FF00+ has device-specific semantics; return 0xff so
	// peek-into-IO never disturbs device state.
	return 0xff;
}

u8 picothing_state::peek_physical(u32 phys) const
{
	return (phys < DAT_PHYS_SIZE) ? m_dat_phys[phys] : 0xff;
}

void picothing_state::on_physical_write(u32 phys, u8 old_val, u8 new_val, offs_t logical)
{
	for (auto const &w : m_wlogs)
	{
		if (w.physical && phys >= w.base && phys < w.base + w.len)
		{
			std::fprintf(stderr,
				"WLOG[p] cycle=%llu pc=$%04X phys=$%06X (virt=$%04X) old=$%02X new=$%02X\n",
				(unsigned long long)cur_cycles(), unsigned(cur_pc()),
				unsigned(phys), unsigned(logical), unsigned(old_val), unsigned(new_val));
		}
	}
	for (auto &w : m_watches)
	{
		if (w.physical && phys == w.base && old_val != new_val)
		{
			std::fprintf(stderr,
				"WATCH[p] cycle=%llu pc=$%04X phys=$%06X (virt=$%04X) old=$%02X new=$%02X\n",
				(unsigned long long)cur_cycles(), unsigned(cur_pc()),
				unsigned(phys), unsigned(logical), unsigned(old_val), unsigned(new_val));
			w.last_value = new_val;
		}
	}
}

void picothing_state::on_logical_write(offs_t logical, u8 old_val, u8 new_val)
{
	for (auto const &w : m_wlogs)
	{
		if (!w.physical && u32(logical) >= w.base && u32(logical) < w.base + w.len)
		{
			std::fprintf(stderr,
				"WLOG cycle=%llu pc=$%04X virt=$%04X old=$%02X new=$%02X\n",
				(unsigned long long)cur_cycles(), unsigned(cur_pc()),
				unsigned(logical), unsigned(old_val), unsigned(new_val));
		}
	}
	for (auto &w : m_watches)
	{
		if (!w.physical && u32(logical) == w.base && old_val != new_val)
		{
			std::fprintf(stderr,
				"WATCH cycle=%llu pc=$%04X virt=$%04X old=$%02X new=$%02X\n",
				(unsigned long long)cur_cycles(), unsigned(cur_pc()),
				unsigned(logical), unsigned(old_val), unsigned(new_val));
			w.last_value = new_val;
		}
	}
}

void picothing_state::install_instrumentation_taps()
{
	// Parse PICOTHING_WATCH / _WLOG / _DUMP into m_watches / m_wlogs /
	// m_dumps. Failures are logged but non-fatal (rest of the list
	// still gets installed).
	auto parse_list = [this](const char *env_name, std::vector<addr_spec> &dst, u32 default_len)
	{
		const char *env = std::getenv(env_name);
		if (!env || !*env) return;
		for (auto const &s : split_specs(env, ';'))
		{
			addr_spec spec;
			if (parse_addr_spec(s, spec, default_len))
			{
				spec.last_value = spec.physical
					? peek_physical(spec.base) : peek_logical(u16(spec.base));
				dst.push_back(spec);
			}
			else
			{
				logerror("picothing: %s: invalid spec '%s'\n", env_name, s.c_str());
			}
		}
	};

	parse_list("PICOTHING_WATCH", m_watches, 1);
	parse_list("PICOTHING_WLOG",  m_wlogs,   1);
	parse_list("PICOTHING_DUMP",  m_dumps,   256);

	// Any physical specs? Toggle the dat_w hot path's fast-skip flag.
	for (auto const &w : m_watches) if (w.physical) { m_have_physical_taps = true; break; }
	if (!m_have_physical_taps)
		for (auto const &w : m_wlogs) if (w.physical) { m_have_physical_taps = true; break; }

	// Install per-spec write taps for LOGICAL specs. Taps capture old
	// value via peek_logical (non-perturbing) and call into our
	// unified on_logical_write dispatcher.
	address_space &space = m_maincpu->space(AS_PROGRAM);
	auto install_one = [this, &space](const addr_spec &spec, const char *kind)
	{
		if (spec.physical) return;
		offs_t const lo = offs_t(spec.base);
		offs_t const hi = offs_t(spec.base + spec.len - 1);
		std::string name = std::string("picothing_") + kind;
		space.install_write_tap(lo, hi, name,
			[this](offs_t addr, u8 &data, u8 /*mask*/)
			{
				u8 const old = peek_logical(u16(addr));
				on_logical_write(addr, old, data);
			});
	};
	for (auto const &w : m_watches) install_one(w, "watch");
	for (auto const &w : m_wlogs)   install_one(w, "wlog");

	// --cycles sidecar path (mirrors LLVM6309_CYCLES_FILE convention).
	if (const char *cf = std::getenv("PICOTHING_CYCLES_FILE"))
		m_cycles_file_path = cf;
}

void picothing_state::instrumentation_dump_one(const addr_spec &spec)
{
	std::fprintf(stderr, "DUMP %s$%0*X len=%u  cycle=%llu pc=$%04X\n",
		spec.physical ? "phys " : "virt ",
		spec.physical ? 6 : 4, unsigned(spec.base), unsigned(spec.len),
		(unsigned long long)cur_cycles(), unsigned(cur_pc()));
	for (u32 row = 0; row < spec.len; row += 16)
	{
		std::fprintf(stderr, "  %0*X  ",
			spec.physical ? 6 : 4, unsigned(spec.base + row));
		// Hex.
		for (u32 i = 0; i < 16; i++)
		{
			if (row + i < spec.len)
			{
				u8 const v = spec.physical
					? peek_physical(spec.base + row + i)
					: peek_logical(u16(spec.base + row + i));
				std::fprintf(stderr, "%02X ", unsigned(v));
			}
			else std::fprintf(stderr, "   ");
		}
		// ASCII.
		std::fprintf(stderr, " |");
		for (u32 i = 0; i < 16 && row + i < spec.len; i++)
		{
			u8 const v = spec.physical
				? peek_physical(spec.base + row + i)
				: peek_logical(u16(spec.base + row + i));
			std::fputc((v >= 0x20 && v < 0x7f) ? char(v) : '.', stderr);
		}
		std::fprintf(stderr, "|\n");
	}
}

void picothing_state::instrumentation_dump_all(const char *trigger)
{
	if (m_dumps.empty()) return;
	std::fprintf(stderr, "--- picothing dump (%s) ---\n", trigger);
	for (auto const &spec : m_dumps)
		instrumentation_dump_one(spec);
}

void picothing_state::instrumentation_emit_cycles()
{
	if (m_cycles_file_path.empty()) return;
	if (FILE *fp = std::fopen(m_cycles_file_path.c_str(), "w"))
	{
		std::fprintf(fp, "%llu\n", (unsigned long long)cur_cycles());
		std::fclose(fp);
	}
}


//------------------------------------------------------------
// Firmware loading — raw / Intel HEX / S-record / ELF
//------------------------------------------------------------

static inline u16 pt_be16(const u8 *p) { return (u16(p[0]) << 8) | u16(p[1]); }
static inline u32 pt_be32(const u8 *p)
{
	return (u32(p[0]) << 24) | (u32(p[1]) << 16) | (u32(p[2]) << 8) | u32(p[3]);
}
static inline u16 pt_le16(const u8 *p) { return u16(p[0]) | (u16(p[1]) << 8); }
static inline u32 pt_le32(const u8 *p)
{
	return u32(p[0]) | (u32(p[1]) << 8) | (u32(p[2]) << 16) | (u32(p[3]) << 24);
}

static int hex_digit(char c)
{
	if (c >= '0' && c <= '9') return c - '0';
	if (c >= 'a' && c <= 'f') return c - 'a' + 10;
	if (c >= 'A' && c <= 'F') return c - 'A' + 10;
	return -1;
}

bool picothing_state::parse_intel_hex(const u8 *src, u32 size, u8 *out64k)
{
	u32 i = 0;
	while (i < size)
	{
		// Skip whitespace / non-record chars.
		while (i < size && src[i] != ':') i++;
		if (i >= size) break;
		i++; // consume ':'

		auto read_byte = [&](u8 &out) -> bool {
			if (i + 2 > size) return false;
			int hi = hex_digit(src[i]);
			int lo = hex_digit(src[i + 1]);
			if (hi < 0 || lo < 0) return false;
			out = u8((hi << 4) | lo);
			i += 2;
			return true;
		};

		u8 byte_count, addr_hi, addr_lo, rec_type;
		if (!read_byte(byte_count) || !read_byte(addr_hi)
			|| !read_byte(addr_lo) || !read_byte(rec_type))
			return false;
		u16 const addr = (u16(addr_hi) << 8) | u16(addr_lo);

		if (rec_type == 1)
			return true; // EOF
		if (rec_type != 0)
		{
			// Skip data + checksum we don't handle.
			i += u32(byte_count) * 2 + 2;
			continue;
		}

		for (u8 n = 0; n < byte_count; n++)
		{
			u8 b;
			if (!read_byte(b)) return false;
			u32 const a = addr + n;
			if (a < 0x10000)
				out64k[a] = b;
		}
		u8 checksum;
		if (!read_byte(checksum)) return false; // discard
		(void)checksum;
	}
	return true;
}

bool picothing_state::parse_srec(const u8 *src, u32 size, u8 *out64k)
{
	u32 i = 0;
	while (i < size)
	{
		while (i < size && src[i] != 'S') i++;
		if (i + 2 > size) break;
		char const type = src[i + 1];
		i += 2;

		auto read_byte = [&](u8 &out) -> bool {
			if (i + 2 > size) return false;
			int hi = hex_digit(src[i]);
			int lo = hex_digit(src[i + 1]);
			if (hi < 0 || lo < 0) return false;
			out = u8((hi << 4) | lo);
			i += 2;
			return true;
		};

		u8 byte_count;
		if (!read_byte(byte_count)) return false;

		if (type == '1')
		{
			// 2-byte address.
			u8 ah, al;
			if (!read_byte(ah) || !read_byte(al)) return false;
			u16 const addr = (u16(ah) << 8) | u16(al);
			u8 data_len = (byte_count >= 3) ? (byte_count - 3) : 0;
			for (u8 n = 0; n < data_len; n++)
			{
				u8 b;
				if (!read_byte(b)) return false;
				u32 const a = addr + n;
				if (a < 0x10000)
					out64k[a] = b;
			}
			u8 cksum;
			if (!read_byte(cksum)) return false;
			(void)cksum;
		}
		else if (type == '9')
		{
			return true; // end
		}
		else
		{
			// Skip remaining bytes of this record.
			for (u8 n = 0; n < byte_count; n++)
			{
				u8 b;
				if (!read_byte(b)) return false;
				(void)b;
			}
		}
	}
	return true;
}

static std::pair<std::error_condition, std::string>
load_elf_into_buf(const u8 *elf, u32 elf_size, u8 *out64k)
{
	if (elf_size < 52)
		return std::make_pair(image_error::INVALIDIMAGE, "ELF: header truncated");
	if (elf[4] != 1)
		return std::make_pair(image_error::INVALIDIMAGE, "ELF: not ELFCLASS32");
	bool const be = (elf[5] == 2);
	if (!be && elf[5] != 1)
		return std::make_pair(image_error::INVALIDIMAGE, "ELF: invalid EI_DATA");
	auto rd16 = [be](const u8 *p) { return be ? pt_be16(p) : pt_le16(p); };
	auto rd32 = [be](const u8 *p) { return be ? pt_be32(p) : pt_le32(p); };

	u32 const e_phoff     = rd32(elf + 28);
	u16 const e_phentsize = rd16(elf + 42);
	u16 const e_phnum     = rd16(elf + 44);
	if (e_phentsize < 32)
		return std::make_pair(image_error::INVALIDIMAGE, "ELF: ph too small");

	for (u16 i = 0; i < e_phnum; i++)
	{
		u32 const phoff = e_phoff + u32(i) * e_phentsize;
		if (phoff + 32 > elf_size)
			return std::make_pair(image_error::INVALIDIMAGE, "ELF: truncated ph");
		const u8 *ph = elf + phoff;
		u32 const p_type   = rd32(ph + 0);
		u32 const p_offset = rd32(ph + 4);
		u32 const p_vaddr  = rd32(ph + 8);
		u32 const p_filesz = rd32(ph + 16);
		if (p_type != 1 || p_filesz == 0) continue;
		if (p_vaddr >= 0x10000 || p_filesz > 0x10000 || p_vaddr + p_filesz > 0x10000)
			return std::make_pair(image_error::INVALIDLENGTH, "ELF: PT_LOAD escapes 64K");
		if (p_offset > elf_size || p_offset + p_filesz > elf_size)
			return std::make_pair(image_error::INVALIDIMAGE, "ELF: PT_LOAD past EOF");
		std::memcpy(out64k + p_vaddr, elf + p_offset, p_filesz);
	}
	return std::make_pair(std::error_condition(), std::string());
}

std::pair<std::error_condition, std::string> picothing_state::load_cart(device_image_interface &image)
{
	generic_slot_device &slot = downcast<generic_slot_device &>(image.device());
	u32 const size = slot.common_get_size("rom");
	if (size == 0)
		return std::make_pair(image_error::INVALIDLENGTH, "Empty cartridge image");

	std::vector<u8> raw(size);
	slot.common_load_rom(&raw[0], size, "rom");

	// Allocate a 64 KB scratch buffer (used to distribute records to
	// DAT physical, FRAM, and the watchpoint shadow), and a 64 KB cart
	// shadow that we keep around for machine_start.
	slot.rom_alloc(0x10000, GENERIC_ROM8_WIDTH, ENDIANNESS_BIG);
	u8 *const cart = slot.get_rom_base();
	std::memset(cart, 0, 0x10000);

	if (size >= 4 && raw[0] == 0x7f && raw[1] == 'E' && raw[2] == 'L' && raw[3] == 'F')
	{
		auto rc = load_elf_into_buf(&raw[0], size, cart);
		if (rc.first) return rc;
		return std::make_pair(std::error_condition(), std::string());
	}

	// Sniff for Intel HEX or S-record (ASCII).
	// Skip leading whitespace.
	u32 first = 0;
	while (first < size && std::isspace(int(raw[first]))) first++;
	if (first < size && raw[first] == ':')
	{
		if (!parse_intel_hex(&raw[0], size, cart))
			return std::make_pair(image_error::INVALIDIMAGE, "Intel HEX parse failed");
		return std::make_pair(std::error_condition(), std::string());
	}
	if (first < size && raw[first] == 'S')
	{
		if (!parse_srec(&raw[0], size, cart))
			return std::make_pair(image_error::INVALIDIMAGE, "S-record parse failed");
		return std::make_pair(std::error_condition(), std::string());
	}

	// Raw binary.
	if (size > 0x10000)
		return std::make_pair(image_error::INVALIDLENGTH, "Raw image larger than 64K");
	std::memcpy(cart, &raw[0], size);
	return std::make_pair(std::error_condition(), std::string());
}


//------------------------------------------------------------
// machine_start / machine_reset
//------------------------------------------------------------

void picothing_state::machine_start()
{
	// 2 MB physical store for the DAT MMU.
	m_dat_phys = std::make_unique<u8[]>(DAT_PHYS_SIZE);
	std::memset(m_dat_phys.get(), 0, DAT_PHYS_SIZE);

	// Identity-map the DAT page table on boot — task 0, pages 0-7 of
	// the table set to physical pages 0-7, and similarly for every
	// other task. usim's DATRAM ctor does `datram[i] = i`; mirror
	// that exactly so a freshly-booted firmware sees a 64 KB virtual
	// space that lines up byte-for-byte with the first 64 KB of
	// physical RAM, in whichever task it was started under.
	for (u32 i = 0; i < DAT_TABLE_SIZE; i++)
		m_dat_table[i] = u8(i);

	// Cycle-driven 50 Hz tick. PICOTHING_TICK env var overrides the
	// default 20000 cycles per tick (matches usim's --tick=N). At the
	// constant 1 MHz E clock, from_ticks(N, clock) is exactly N CPU
	// cycles between fires — equivalent to usim's instruction-cycle
	// counter for this fixed-clock device.
	if (const char *tick_env = std::getenv("PICOTHING_TICK"))
	{
		long const n = std::strtol(tick_env, nullptr, 0);
		m_tick_period = (n > 0) ? u32(n) : PICOTICK_DEFAULT_PERIOD;
	}
	m_tick_timer = timer_alloc(FUNC(picothing_state::picotick_fire), this);
	attotime const period = attotime::from_ticks(m_tick_period, m_maincpu->clock());
	m_tick_timer->adjust(period, 0, period);

	// FRAM persistence — env-var path with a sensible default.
	const char *fram_env = std::getenv("PICOTHING_FRAM_FILE");
	m_fram_path = fram_env ? fram_env : "picothing.fram";
	std::memset(m_fram, 0xff, FRAM_SIZE);
	fram_load();

	// IDE disk image — optional, env-var only. Open r+b; if the file
	// doesn't exist try w+b so a fresh disk can be initialised by the
	// guest. Read-only filesystems silently leave m_ide_disk null and
	// the controller reports DRDY|ERR on every command.
	const char *disk_env = std::getenv("PICOTHING_DISK_FILE");
	if (disk_env)
	{
		m_ide_disk = std::fopen(disk_env, "r+b");
		if (!m_ide_disk)
			m_ide_disk = std::fopen(disk_env, "w+b");
		if (!m_ide_disk)
			logerror("picothing: cannot open IDE disk image '%s'\n", disk_env);
	}

	// Distribute the cart image:
	//  $0000-$FEFF → DAT physical (identity-mapped at task 0)
	//  $FFD0-$FFDF → FRAM (16 bytes)
	//  $FFE0-$FFFF → SystemWatchpoint shadow (32 bytes)
	if (u8 const *src = m_cart->get_rom_base())
	{
		// Physical region: a fresh DAT table maps virtual page N to
		// physical page N, so virtual $XXXX = physical $XXXX for the
		// first task.
		for (u32 a = 0; a < 0xFF00; a++)
		{
			u8 const b = src[a];
			if (b)
				m_dat_phys[a] = b;
		}
		for (u32 i = 0; i < FRAM_SIZE; i++)
		{
			u8 const b = src[FRAM_BASE + i];
			if (b)
				m_fram[i] = b;
		}
		std::memcpy(m_swp_shadow, src + SWP_WINDOW_BASE, SWP_WINDOW_SIZE);
	}

	save_pointer(NAME(m_dat_phys), DAT_PHYS_SIZE);
	save_item(NAME(m_dat_table));
	save_item(NAME(m_dat_task));
	save_item(NAME(m_tick_ctrl));
	save_item(NAME(m_tick_status));
	save_item(NAME(m_tick_period));
	save_item(NAME(m_fram));
	save_item(NAME(m_ide_buffer));
	save_item(NAME(m_ide_buf_ptr));
	save_item(NAME(m_ide_writing));
	save_item(NAME(m_ide_error));
	save_item(NAME(m_ide_features));
	save_item(NAME(m_ide_sector_count));
	save_item(NAME(m_ide_sectors_remaining));
	save_item(NAME(m_ide_lba_low));
	save_item(NAME(m_ide_lba_mid));
	save_item(NAME(m_ide_lba_high));
	save_item(NAME(m_ide_drive_head));
	save_item(NAME(m_ide_status));
	save_item(NAME(m_ide_devctl));
	save_item(NAME(m_swp_shadow));
	save_item(NAME(m_swp_armed));
	save_item(NAME(m_swp_saved_nmi_hi));
	save_item(NAME(m_swp_saved_nmi_lo));
	save_item(NAME(m_trace_enabled));

	// Stage C: parse PICOTHING_WATCH / _WLOG / _DUMP / _CYCLES_FILE
	// and install non-perturbing memory taps. Must come AFTER cart
	// distribution so peek_logical reads the post-load values.
	install_instrumentation_taps();

	// At exit, run all --dump regions and emit the cycle count.
	machine().add_notifier(MACHINE_NOTIFY_EXIT,
		machine_notify_delegate(&picothing_state::machine_exit_notify, this));
}

void picothing_state::machine_exit_notify()
{
	instrumentation_dump_all("exit");
	instrumentation_emit_cycles();
}

void picothing_state::machine_reset()
{
	swp_disarm();
	m_tick_ctrl = 0;
	m_tick_status = 0;
	m_irq_merger->in_w<2>(0);
}


//------------------------------------------------------------
// Memory map
//------------------------------------------------------------

void picothing_state::mem_map(address_map &map)
{
	map(0x0000, 0xfdff).rw(FUNC(picothing_state::dat_r), FUNC(picothing_state::dat_w));
	map(0xfe00, 0xfeff).rw(FUNC(picothing_state::dat_table_r), FUNC(picothing_state::dat_table_w));
	map(0xff00, 0xff09).rw(FUNC(picothing_state::ide_r), FUNC(picothing_state::ide_w));
	map(0xffc0, 0xffc0).rw(FUNC(picothing_state::picotask_r), FUNC(picothing_state::picotask_w));
	map(0xffc4, 0xffc5).rw(m_acia_con, FUNC(acia6850_device::read), FUNC(acia6850_device::write));
	map(0xffc6, 0xffc7).rw(m_acia_aux, FUNC(acia6850_device::read), FUNC(acia6850_device::write));
	map(0xffc8, 0xffc9).rw(FUNC(picothing_state::picotick_r), FUNC(picothing_state::picotick_w));
	map(0xffca, 0xffca).w(FUNC(picothing_state::swp_ctrl_w));
	map(0xffcb, 0xffcb).w(FUNC(picothing_state::tracectl_w));
	map(0xffd0, 0xffdf).rw(FUNC(picothing_state::fram_r), FUNC(picothing_state::fram_w));
	map(0xffe0, 0xffff).rw(FUNC(picothing_state::swp_vec_r), FUNC(picothing_state::swp_vec_w));
}


//------------------------------------------------------------
// RS-232 plumbing — same workaround as llvm6309
//------------------------------------------------------------

static DEVICE_INPUT_DEFAULTS_START( picothing_null_modem )
	DEVICE_INPUT_DEFAULTS( "RS232_RXBAUD",   0xff, RS232_BAUD_115200 )
	DEVICE_INPUT_DEFAULTS( "RS232_TXBAUD",   0xff, RS232_BAUD_115200 )
	DEVICE_INPUT_DEFAULTS( "RS232_DATABITS", 0xff, RS232_DATABITS_8 )
	DEVICE_INPUT_DEFAULTS( "RS232_PARITY",   0xff, RS232_PARITY_NONE )
	DEVICE_INPUT_DEFAULTS( "RS232_STOPBITS", 0xff, RS232_STOPBITS_1 )
DEVICE_INPUT_DEFAULTS_END

static void picothing_rs232_devices(device_slot_interface &device)
{
	device.option_add("null_modem", NULL_MODEM);
}


//------------------------------------------------------------
// machine_config
//------------------------------------------------------------

void picothing_state::picothing(machine_config &config)
{
	// CPU: MC6809E at 1 MHz E (matches real Pico-Thing clock).
	MC6809E(config, m_maincpu, XTAL(4'000'000) / 4); // 1 MHz E
	m_maincpu->set_addrmap(AS_PROGRAM, &picothing_state::mem_map);

	// IRQ merger: console ACIA on input 0, aux ACIA on input 1,
	// tick timer on input 2. Wired-OR drives the CPU IRQ line.
	INPUT_MERGER_ANY_HIGH(config, m_irq_merger);
	m_irq_merger->output_handler().set_inputline(m_maincpu, M6809_IRQ_LINE);

	// Console ACIA → RS232 slot "rs232".
	ACIA6850(config, m_acia_con, 0);
	m_acia_con->txd_handler().set("rs232", FUNC(rs232_port_device::write_txd));
	m_acia_con->irq_handler().set(m_irq_merger, FUNC(input_merger_device::in_w<0>));

	// Aux ACIA → null peer. TX is dropped; RX never produces data.
	// Wired through input_merger input 1 even though it never fires —
	// this keeps the IRQ wiring symmetric with real PT hardware.
	ACIA6850(config, m_acia_aux, 0);
	m_acia_aux->irq_handler().set(m_irq_merger, FUNC(input_merger_device::in_w<1>));

	// ACIA TX/RX clock = 1.8432 MHz → /16 in ACIA → 115200 baud.
	clock_device &acia_clock(CLOCK(config, "acia_clock", 1'843'200));
	acia_clock.signal_handler().set(m_acia_con, FUNC(acia6850_device::write_txc));
	acia_clock.signal_handler().append(m_acia_con, FUNC(acia6850_device::write_rxc));
	acia_clock.signal_handler().append(m_acia_aux, FUNC(acia6850_device::write_txc));
	acia_clock.signal_handler().append(m_acia_aux, FUNC(acia6850_device::write_rxc));

	// RS232 slot for the console ACIA, default null_modem.
	rs232_port_device &rs232(RS232_PORT(config, "rs232", picothing_rs232_devices, "null_modem"));
	rs232.rxd_handler().set(m_acia_con, FUNC(acia6850_device::write_rxd));
	rs232.set_option_device_input_defaults("null_modem", DEVICE_INPUT_DEFAULTS_NAME(picothing_null_modem));

	// Firmware cart: raw .bin, Intel HEX (.hex), Motorola SREC (.s19/.srec),
	// or ELF (.elf). load_cart() sniffs the format.
	GENERIC_SOCKET(config, m_cart, generic_plain_slot, "picothing_rom", "bin,rom,hex,s19,srec,elf");
	m_cart->set_device_load(FUNC(picothing_state::load_cart));
	m_cart->set_must_be_loaded(true);
}

ROM_START(picothing)
ROM_END

} // anonymous namespace


//    YEAR  NAME       PARENT  COMPAT  MACHINE    INPUT  CLASS            INIT         COMPANY        FULLNAME                              FLAGS
COMP( 2026, picothing, 0,      0,      picothing, 0,     picothing_state, empty_init,  "Mark Murray", "Pico-Thing MC6809 SBC (emulated)",   MACHINE_NO_SOUND_HW )
