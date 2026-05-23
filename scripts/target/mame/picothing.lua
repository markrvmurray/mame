-- license:BSD-3-Clause
-- copyright-holders:Mark Murray

---------------------------------------------------------------------------
--
--   picothing.lua
--
--   Subtarget for the Pico-Thing MC6809 SBC emulator. Heavier than
--   llvm6309 — adds DAT MMU + 2 MB paged RAM + dual ACIA + 50 Hz tick
--   + PATA IDE + FRAM + SystemWatchpoint, all inlined in the driver.
--
--   Build with: make SUBTARGET=picothing REGENIE=1 -j$(sysctl -n hw.ncpu)
--   Run with:   ./mamepicothing picothing -cart firmware.s19 \
--                   -rs232 null_modem -bitb /tmp/out.txt
--
---------------------------------------------------------------------------


--------------------------------------------------
-- CPU cores (MC6809 — token covers MC6809E too).
--------------------------------------------------

CPUS["M6809"] = true


--------------------------------------------------
-- Machine peripherals:
--   ACIA6850     — both serial ports
--   INPUT_MERGER — wired-OR for the shared IRQ line
--   CLOCK        — drives the ACIA TX/RX clocks (always on, no token)
--------------------------------------------------

MACHINES["ACIA6850"] = true
MACHINES["INPUT_MERGER"] = true


--------------------------------------------------
-- Buses:
--
--   GENERIC — runtime cartridge slot for the firmware image.
--
--   RS232 is intentionally NOT enabled — same workaround as
--   llvm6309. A local stripped rs232 core (llvm6309_rs232_core.cpp)
--   provides the bus class symbols without dragging in the full
--   set of terminal cards from default_rs232_devices(). The
--   driver's own picothing_rs232_devices() function offers only
--   null_modem.
--------------------------------------------------

BUSES["GENERIC"] = true


--------------------------------------------------
-- Source files that make up this subtarget.
--------------------------------------------------

function createProjects_mame_picothing(_target, _subtarget)
	project ("mame_picothing")
	targetsubdir(_target .."_" .. _subtarget)
	kind (LIBTYPE)
	uuid (os.uuid("drv-mame-picothing"))
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
		MAME_DIR .. "src/mame/homebrew/picothing.cpp",
		-- Reuse the llvm6309 stripped rs232 core — same problem,
		-- same workaround. Identical bus-core API; if upstream
		-- ever drifts, update one file and both subtargets pick
		-- up the fix.
		MAME_DIR .. "src/mame/homebrew/llvm6309_rs232_core.cpp",
		MAME_DIR .. "src/devices/bus/rs232/null_modem.cpp",
		MAME_DIR .. "src/devices/bus/rs232/null_modem.h",
	}
end

function linkProjects_mame_picothing(_target, _subtarget)
	links {
		"mame_picothing",
	}
end
