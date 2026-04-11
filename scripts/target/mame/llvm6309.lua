-- license:BSD-3-Clause
-- copyright-holders:Mark Murray

---------------------------------------------------------------------------
--
--   llvm6309.lua
--
--   Minimal subtarget: builds only the llvm6309 HD6309 SBC driver used to
--   validate the llvm-mc6809 backend. Fast build, fast link.
--
--   Build with: make SUBTARGET=llvm6309 REGENIE=1 -j$(sysctl -n hw.ncpu)
--   Run with:   ./mamellvm6309 llvm6309 -cart test.bin \
--                   -rs232 null_modem -bitb /tmp/out.txt \
--                   -window -nothrottle -seconds_to_run 2
--
---------------------------------------------------------------------------


--------------------------------------------------
-- CPU cores (M6809 token covers MC6809, HD6309
-- and Konami variants — all built from the same
-- m6809 directory).
--------------------------------------------------

CPUS["M6809"] = true


--------------------------------------------------
-- No sound cores are needed, but some MAME core
-- code still references these symbols at link
-- time. Leave the sound list empty.
--------------------------------------------------


--------------------------------------------------
-- No video cores needed.
--------------------------------------------------


--------------------------------------------------
-- Machine peripherals: just the MC6850 ACIA.
-- (machine/clock is unconditionally included by
-- scripts/src/machine.lua, so no token for it.)
--------------------------------------------------

MACHINES["ACIA6850"] = true


--------------------------------------------------
-- Buses:
--
--   GENERIC is enabled normally — used by the runtime cartridge slot that
--   feeds `-cart file.bin` into the 8K ROM region.
--
--   RS232 is intentionally NOT enabled. If it were, bus.lua would compile
--   every rs232 card (heath_h19, ie15, mboardd, scorpion, sun_kbd, swtpc8212,
--   terminal, votraxtnt, ...) into liboptional.a, and rs232.cpp's
--   default_rs232_devices() function would drag all of them into the final
--   link via hard references — pulling in transitive dependencies on
--   beep_device, ay8910, mc6845, m6800/m6801, and more. We only need the
--   bus core plus null_modem, so:
--
--     - llvm6309_rs232_core.cpp (below) is a local stripped copy of
--       src/devices/bus/rs232/rs232.cpp without the terminal includes or
--       default_rs232_devices(), providing just the bus class symbols.
--     - null_modem.cpp is compiled directly into this subtarget's driver
--       library (it only depends on imagedev/bitbngr which is already in
--       machine.lua's unconditional section).
--   The driver defines its own minimal rs232 slot-option function
--   (llvm6309_rs232_devices) so it never calls default_rs232_devices().
--------------------------------------------------

BUSES["GENERIC"] = true


--------------------------------------------------
-- Source files that make up this subtarget.
-- Single driver — everything else comes from the
-- MAME core / devices libraries.
--------------------------------------------------

function createProjects_mame_llvm6309(_target, _subtarget)
	project ("mame_llvm6309")
	targetsubdir(_target .."_" .. _subtarget)
	kind (LIBTYPE)
	uuid (os.uuid("drv-mame-llvm6309"))
	addprojectflags()
	precompiledheaders_novs()

	includedirs {
		MAME_DIR .. "src/osd",
		MAME_DIR .. "src/emu",
		MAME_DIR .. "src/devices",
		MAME_DIR .. "src/mame/shared",
		MAME_DIR .. "src/lib",
		MAME_DIR .. "src/lib/util",
		MAME_DIR .. "3rdparty",
		GEN_DIR  .. "mame/layout",
	}

	files {
		MAME_DIR .. "src/mame/homebrew/llvm6309.cpp",
		-- Local stripped rs232 bus core — replaces src/devices/bus/rs232/rs232.cpp,
		-- which is not compiled because BUSES["RS232"] is not set.
		MAME_DIR .. "src/mame/homebrew/llvm6309_rs232_core.cpp",
		-- null_modem is the only rs232 card we actually use; compiled here so
		-- it does not pull in the full rs232 card zoo via liboptional.a.
		MAME_DIR .. "src/devices/bus/rs232/null_modem.cpp",
		MAME_DIR .. "src/devices/bus/rs232/null_modem.h",
	}
end

function linkProjects_mame_llvm6309(_target, _subtarget)
	links {
		"mame_llvm6309",
	}
end
