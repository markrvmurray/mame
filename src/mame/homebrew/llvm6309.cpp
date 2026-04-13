// license:BSD-3-Clause
// copyright-holders:Mark Murray

// Minimal HD6309 single-board computer for validating the llvm-mc6809 backend.
//
// Memory map
// ----------
//   $0000-$BFFF   48K RAM
//   $C000-$C001   MC6850 ACIA (serial console — 2-byte I/O window)
//   $C002         Exit port (write any value → MAME exits cleanly)
//   $C003-$FFFF   16K ROM (less the 3 bytes shadowed by the ACIA and exit port
//                 at the start of the region; reset/IRQ vectors at $FFF0-$FFFF
//                 are ROM as normal)
//
// The ROM is loaded at runtime from a generic cartridge slot, so there is no
// hash-verified ROM file baked into the driver — point `-cart` at a freshly
// assembled/linked 16K image per test run. The image covers the full $C000-$FFFF
// range; the bytes at offsets 0, 1, 2 (i.e. $C000-$C002) are ignored because
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
// The test program ends with `sta $C002` (or any store to $C002) which calls
// machine().schedule_exit() and terminates MAME — `-seconds_to_run` is then
// only a safety net for runaway programs.
//
// IRQ is intentionally not wired from the ACIA — polled TDRE/RDRF only.

#include "emu.h"

#include <cstdio>
#include <cstdlib>

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
	{ }

	void llvm6309(machine_config &config);

protected:
	virtual void machine_start() override ATTR_COLD;

private:
	void mem_map(address_map &map) ATTR_COLD;
	void exit_w(u8 data);
	TIMER_CALLBACK_MEMBER(do_exit);
	std::pair<std::error_condition, std::string> load_cart(device_image_interface &image);

	required_device<hd6309_device> m_maincpu;
	required_device<acia6850_device> m_acia;
	required_device<generic_slot_device> m_cart;
	emu_timer *m_exit_timer = nullptr;
	u8 m_exit_code = 0;
};

void llvm6309_state::machine_start()
{
	m_exit_timer = timer_alloc(FUNC(llvm6309_state::do_exit), this);
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
	if (size > 0x4000)
		return std::make_pair(image_error::INVALIDLENGTH, "Cartridge image must be no larger than 16K");
	slot.rom_alloc(0x4000, GENERIC_ROM8_WIDTH, ENDIANNESS_BIG);
	slot.common_load_rom(slot.get_rom_base(), size, "rom");
	return std::make_pair(std::error_condition(), std::string());
}

void llvm6309_state::mem_map(address_map &map)
{
	map(0x0000, 0xbfff).ram();
	// Cart ROM covers $C000-$FFFF; ACIA and exit port below overlay the first
	// three bytes. Later address_map entries override earlier ones.
	map(0xc000, 0xffff).r(m_cart, FUNC(generic_slot_device::read_rom));
	map(0xc000, 0xc001).rw(m_acia, FUNC(acia6850_device::read), FUNC(acia6850_device::write));
	map(0xc002, 0xc002).w(FUNC(llvm6309_state::exit_w));
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

	// 16K ROM via generic cart slot — pass with `-cart file.bin`.
	GENERIC_SOCKET(config, m_cart, generic_plain_slot, "llvm6309_rom", "bin,rom");
	m_cart->set_device_load(FUNC(llvm6309_state::load_cart));
	m_cart->set_must_be_loaded(true);
}

ROM_START(llvm6309)
	// No baked ROM — the image comes from `-cart` at runtime.
ROM_END

} // anonymous namespace


//    YEAR  NAME      PARENT  COMPAT  MACHINE   INPUT  CLASS           INIT         COMPANY        FULLNAME                                 FLAGS
COMP( 2026, llvm6309, 0,      0,      llvm6309, 0,     llvm6309_state, empty_init,  "Mark Murray", "HD6309 LLVM backend validation SBC",    MACHINE_NO_SOUND_HW )
