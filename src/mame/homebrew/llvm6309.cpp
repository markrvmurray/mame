// license:BSD-3-Clause
// copyright-holders:Mark Murray

// Minimal HD6309 single-board computer for validating the llvm-mc6809 backend.
//
// Memory map (top-to-bottom — $0000 at the top, $FFFF at the bottom):
// ----------
//   $0000-$FFCF   RAM (~64K contiguous — no IO wedge in the middle)
//   $FFD0-$FFD1   MC6850 ACIA (status, data)
//   $FFD2         Exit port (write any value → MAME exits cleanly, value
//                 captured as the process exit code)
//   $FFD3         TraceCtl (program-controlled host-side logging toggle)
//   $FFD4-$FFEF   RAM (reserved for future IO)
//   $FFF0-$FFFF   Vector ROM (reads come from RAM populated at boot, writes
//                 are dropped so a runaway program can't clobber the reset
//                 entry)
//
// This layout mirrors the usim09batch simulator (see ../usim) so the same
// test programs run unchanged on both emulators. The previous $C000 IO
// overlay with a $C003-$FFFF cart ROM is gone — the machine is now a flat
// RAM with a tiny IO slab and vector ROM stuck at the bottom, giving test
// programs ~64 KB of contiguous address space.
//
// The image is loaded at runtime from a generic cartridge slot; it is a
// raw binary up to 64 KB (typically a full 64 KB image produced by
// `lwasm --format=rawrel`). The bytes are memcpy'd into RAM at boot — the
// bottom 16 bytes ($FFF0-$FFFF) end up as the reset/IRQ vectors.
//
// Typical LLVM execution-test invocation (via the mamellvm6309-headless
// wrapper):
//
//   mamellvm6309-headless test.bin 2
//   FileCheck %s < /tmp/out.txt
//
// The test program ends with `sta $FFD2` which schedules a clean exit
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

private:
	void mem_map(address_map &map) ATTR_COLD;
	void exit_w(u8 data);
	void tracectl_w(u8 data);
	TIMER_CALLBACK_MEMBER(do_exit);
	std::pair<std::error_condition, std::string> load_cart(device_image_interface &image);

	required_device<hd6309_device> m_maincpu;
	required_device<acia6850_device> m_acia;
	required_device<generic_slot_device> m_cart;
	required_shared_ptr<u8> m_mem;
	emu_timer *m_exit_timer = nullptr;
	u8 m_exit_code = 0;
	bool m_trace_enabled = false;
};

void llvm6309_state::machine_start()
{
	m_exit_timer = timer_alloc(FUNC(llvm6309_state::do_exit), this);

	// Populate RAM from the cart image. The cart is a raw binary (up to
	// 64 KB); we memcpy the whole thing into the "mem" share so byte N of
	// the cart ends up at address $N. The bottom 16 bytes ($FFF0-$FFFF)
	// end up as the reset/IRQ vectors — reads go through normal RAM, but
	// the .nopw() overlay in mem_map drops any subsequent writes so a
	// runaway program can't clobber the reset entry.
	u32 const size = m_cart->common_get_size("rom");
	if (size > 0)
	{
		u8 const *src = m_cart->get_rom_base();
		std::memcpy(&m_mem[0], src, std::min<u32>(size, 0x10000));
	}

	save_item(NAME(m_exit_code));
	save_item(NAME(m_trace_enabled));
}

void llvm6309_state::exit_w(u8 data)
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
	machine().schedule_exit();
}

std::pair<std::error_condition, std::string> llvm6309_state::load_cart(device_image_interface &image)
{
	generic_slot_device &slot = downcast<generic_slot_device &>(image.device());
	u32 size = slot.common_get_size("rom");
	if (size == 0)
		return std::make_pair(image_error::INVALIDLENGTH, "Empty cartridge image");
	if (size > 0x10000)
		return std::make_pair(image_error::INVALIDLENGTH, "Cartridge image must be no larger than 64K");
	slot.rom_alloc(0x10000, GENERIC_ROM8_WIDTH, ENDIANNESS_BIG);
	slot.common_load_rom(slot.get_rom_base(), size, "rom");
	return std::make_pair(std::error_condition(), std::string());
}

void llvm6309_state::mem_map(address_map &map)
{
	// Unified ~64 KB RAM. Test programs see a flat contiguous address
	// space from $0000 down to $FFCF with no IO wedge in the middle.
	map(0x0000, 0xffff).ram().share("mem");

	// IO slab at the bottom, just above the vectors. These .rw()/.w()
	// entries override the RAM mapping for their ranges (later wins).
	map(0xffd0, 0xffd1).rw(m_acia, FUNC(acia6850_device::read), FUNC(acia6850_device::write));
	map(0xffd2, 0xffd2).w(FUNC(llvm6309_state::exit_w));
	map(0xffd3, 0xffd3).w(FUNC(llvm6309_state::tracectl_w));

	// Vectors at $FFF0-$FFFF: reads still come from the underlying RAM
	// (populated from the cart at machine_start), but writes are silently
	// dropped so a runaway program can't clobber the reset entry.
	map(0xfff0, 0xffff).nopw();
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
	GENERIC_SOCKET(config, m_cart, generic_plain_slot, "llvm6309_rom", "bin,rom");
	m_cart->set_device_load(FUNC(llvm6309_state::load_cart));
	m_cart->set_must_be_loaded(true);
}

ROM_START(llvm6309)
	// No baked ROM — the image comes from `-rom` at runtime.
ROM_END

} // anonymous namespace


//    YEAR  NAME      PARENT  COMPAT  MACHINE   INPUT  CLASS           INIT         COMPANY        FULLNAME                                 FLAGS
COMP( 2026, llvm6309, 0,      0,      llvm6309, 0,     llvm6309_state, empty_init,  "Mark Murray", "HD6309 LLVM backend validation SBC",    MACHINE_NO_SOUND_HW )
