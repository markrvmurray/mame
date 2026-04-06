// license:BSD-3-Clause
// copyright-holders:Mark Murray

// Minimal HD6309 single-board computer for validating the llvm-mc6809 backend.
//
// Memory map
// ----------
//   $0000-$DFFF   56K RAM
//   $E000-$E001   MC6850 ACIA (serial console — 2-byte I/O window)
//   $E002         Exit port (write any value → MAME exits cleanly)
//   $E003-$FFFF   8K ROM (less the 3 bytes shadowed by the ACIA and exit port
//                 at the start of the region; reset/IRQ vectors at $FFF0-$FFFF
//                 are ROM as normal)
//
// The ROM is loaded at runtime from a generic cartridge slot, so there is no
// hash-verified ROM file baked into the driver — point `-cart` at a freshly
// assembled/linked 8K image per test run. The image covers the full $E000-$FFFF
// range; the bytes at offsets 0, 1, 2 (i.e. $E000-$E002) are ignored because
// the ACIA and exit port are overlaid on top.
//
// Typical LLVM execution-test invocation (headless-ish, lit-friendly):
//
//   mamellvm6309 llvm6309 -cart test.bin \
//       -rs232 null_modem -bitb /tmp/out.txt \
//       -window -nothrottle -seconds_to_run 2
//
//   FileCheck %s < /tmp/out.txt
//
// The test program ends with `sta $E002` (or any store to $E002) which calls
// machine().schedule_exit() and terminates MAME — `-seconds_to_run` is then
// only a safety net for runaway programs.
//
// IRQ is intentionally not wired from the ACIA — polled TDRE/RDRF only.

#include "emu.h"

#include "bus/generic/carts.h"
#include "bus/generic/slot.h"
#include "bus/rs232/rs232.h"
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
	{ }

	void llvm6309(machine_config &config);

private:
	void mem_map(address_map &map) ATTR_COLD;
	void exit_w(u8 data);

	required_device<hd6309_device> m_maincpu;
	required_device<acia6850_device> m_acia;
	required_device<generic_slot_device> m_cart;
};

void llvm6309_state::exit_w(u8 data)
{
	// Any write here ends the emulation run cleanly.
	machine().schedule_exit();
}

void llvm6309_state::mem_map(address_map &map)
{
	map(0x0000, 0xdfff).ram();
	// Cart ROM covers $E000-$FFFF; ACIA and exit port below overlay the first
	// three bytes. Later address_map entries override earlier ones.
	map(0xe000, 0xffff).r(m_cart, FUNC(generic_slot_device::read_rom));
	map(0xe000, 0xe001).rw(m_acia, FUNC(acia6850_device::read), FUNC(acia6850_device::write));
	map(0xe002, 0xe002).w(FUNC(llvm6309_state::exit_w));
}

// Host-side defaults for the RS232 terminal option (used if someone runs
// with `-rs232 terminal` for interactive poking instead of null_modem).
static DEVICE_INPUT_DEFAULTS_START( terminal )
	DEVICE_INPUT_DEFAULTS( "RS232_RXBAUD",   0xff, RS232_BAUD_115200 )
	DEVICE_INPUT_DEFAULTS( "RS232_TXBAUD",   0xff, RS232_BAUD_115200 )
	DEVICE_INPUT_DEFAULTS( "RS232_DATABITS", 0xff, RS232_DATABITS_8 )
	DEVICE_INPUT_DEFAULTS( "RS232_PARITY",   0xff, RS232_PARITY_NONE )
	DEVICE_INPUT_DEFAULTS( "RS232_STOPBITS", 0xff, RS232_STOPBITS_1 )
DEVICE_INPUT_DEFAULTS_END

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
	// of the box for lit execution tests. Override with e.g. `-rs232 terminal`
	// for an interactive window, or `-rs232 pty` for a pseudo-terminal.
	rs232_port_device &rs232(RS232_PORT(config, "rs232", default_rs232_devices, "null_modem"));
	rs232.rxd_handler().set(m_acia, FUNC(acia6850_device::write_rxd));
	rs232.set_option_device_input_defaults("terminal", DEVICE_INPUT_DEFAULTS_NAME(terminal));

	// 8K ROM via generic cart slot — pass with `-cart file.bin`.
	GENERIC_SOCKET(config, m_cart, generic_plain_slot, "llvm6309_rom", "bin,rom");
	m_cart->set_device_load(NAME([] (device_image_interface &image) -> std::pair<std::error_condition, std::string>
	{
		generic_slot_device &slot = downcast<generic_slot_device &>(image.device());
		u32 size = slot.common_get_size("rom");
		if (size == 0)
			return std::make_pair(image_error::INVALIDLENGTH, "Empty cartridge image");
		if (size > 0x2000)
			return std::make_pair(image_error::INVALIDLENGTH, "Cartridge image must be no larger than 8K");
		slot.rom_alloc(0x2000, GENERIC_ROM8_WIDTH, ENDIANNESS_BIG);
		slot.common_load_rom(slot.get_rom_base(), size, "rom");
		return std::make_pair(std::error_condition(), std::string());
	}));
	m_cart->set_must_be_loaded(true);
}

ROM_START(llvm6309)
	// No baked ROM — the image comes from `-cart` at runtime.
ROM_END

} // anonymous namespace


//    YEAR  NAME      PARENT  COMPAT  MACHINE   INPUT  CLASS           INIT         COMPANY        FULLNAME                                 FLAGS
COMP( 2026, llvm6309, 0,      0,      llvm6309, 0,     llvm6309_state, empty_init,  "Mark Murray", "HD6309 LLVM backend validation SBC",    MACHINE_NO_SOUND_HW )
