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
-- Buses: RS232 for the serial console slot, and
-- GENERIC for the runtime cartridge slot that
-- feeds `-cart file.bin` into the 8K ROM region.
--------------------------------------------------

BUSES["RS232"] = true
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
	}
end

function linkProjects_mame_llvm6309(_target, _subtarget)
	links {
		"mame_llvm6309",
	}
end
