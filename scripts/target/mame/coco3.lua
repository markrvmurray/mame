-- license:BSD-3-Clause
-- copyright-holders:Mark Murray

---------------------------------------------------------------------------
--
--   coco3.lua
--
--   Subtarget for the Tandy Color Computer family — primarily the
--   CoCo 3 (GIME MMU + 512 KB RAM), used to boot and exercise
--   NitrOS-9 Level 2. The CoCo 1/2 and Dragon machines share the same
--   base classes (coco.cpp / coco12.cpp), so they are registered too;
--   they cost .lst lines, not compile time.
--
--   Unlike llvm6309 / picothing this is NOT a small subtarget. The
--   CoCo cartridge bus (BUSES["COCO"]) is all-or-nothing and drags in
--   the ATA / BBC-userport / Centronics / MIDI / RS232 buses plus a
--   TMS7000 CPU, a WD floppy controller and a fair spread of sound and
--   video devices — see the notes against each block below. It cannot
--   use the stripped-bus-core trick the other two subtargets use,
--   because the slot-option functions (coco_cart_add_basic_devices and
--   friends) are called from *upstream* src/mame/trs/coco12.cpp, not
--   from a driver of ours. Stripping them would mean patching an
--   upstream file and carrying the patch across every rebase.
--
--   Build with: make SUBTARGET=coco3 REGENIE=1 -j$(sysctl -n hw.ncpu)
--   Run with:   ./mamecoco3 coco3 -flop1 nitros9_l2.dsk
--
---------------------------------------------------------------------------


--------------------------------------------------
-- CPU cores.
--
--   M6809   — MC6809E (coco3) and HD6309 (coco3h); one token covers
--             both, they share a directory.
--   TMS7000 — required by the coco bus: dragon_msx2.cpp uses one as
--             the cartridge's onboard controller.
--------------------------------------------------

CPUS["M6809"] = true
CPUS["TMS7000"] = true


--------------------------------------------------
-- Sound cores.
--
--   DAC     — the CoCo's 6-bit DAC (coco.h)
--   AY8910  — coco12.h (Speech/Sound Pak era) and coco_psg.cpp
--   SID6581 — coco_xsid.cpp
--   SN76496 — coco_gmc.cpp (Game Master Cartridge)
--   SP0256  — coco_ssc.cpp (Speech/Sound Cartridge)
--   YM2413  — dragon_msx2.cpp
--------------------------------------------------

SOUNDS["DAC"] = true
SOUNDS["AY8910"] = true
SOUNDS["SID6581"] = true
SOUNDS["SN76496"] = true
SOUNDS["SP0256"] = true
SOUNDS["YM2413"] = true


--------------------------------------------------
-- Video cores.
--
--   MC6847  — the CoCo 1/2 VDG; also the base class the GIME derives
--             from, so it is needed even for a CoCo 3 only build
--   MC6845  — dragon.h (d64plus 80-column card) and coco_stecomp.cpp
--   TMS9928A / V9938 — dragon_msx2.cpp
--------------------------------------------------

VIDEOS["MC6847"] = true
VIDEOS["MC6845"] = true
VIDEOS["TMS9928A"] = true
VIDEOS["V9938"] = true


--------------------------------------------------
-- Machine peripherals.
--
--   6821PIA      — the two CoCo PIAs (keyboard, cassette, sound, IRQ)
--   6883SAM      — the SAM; the GIME is modelled as a SAM subclass
--   BANKDEV      — coco.h address-space banking
--   INPUT_MERGER — wired-OR onto the shared CPU IRQ/FIRQ lines
--   MOS6551      — coco12.h (deluxecoco serial) and coco_rs232.cpp
--   6522VIA      — coco bus (dragon_claw.cpp)
--   ACIA6850     — coco bus (dragon_serial.cpp)
--   INTELFLASH   — coco bus (coco_max.cpp, coco_wpk2p.cpp)
--   MC14411      — coco bus (dragon_serial.cpp bit-rate generator)
--   MSM6242      — coco bus (meb_rtime.cpp real-time clock)
--   WD_FDC       — coco_fdc.cpp / dragon_fdc.cpp — the disk controller
--                  NitrOS-9 actually boots from
--
--   RAM, TIMER and CLOCK are part of the always-built core and have
--   no token of their own.
--------------------------------------------------

MACHINES["6821PIA"] = true
MACHINES["6883SAM"] = true
MACHINES["BANKDEV"] = true
MACHINES["INPUT_MERGER"] = true
MACHINES["MOS6551"] = true
MACHINES["6522VIA"] = true
MACHINES["ACIA6850"] = true
MACHINES["INTELFLASH"] = true
MACHINES["MC14411"] = true
MACHINES["MSM6242"] = true
MACHINES["WD_FDC"] = true
MACHINES["FDC_PLL"] = true              -- wd_fdc's analog data separator

-- Pulled in transitively by BUSES["ATA"], not wanted directly: the
-- ATA bus compiles atapicdr.cpp and hdd.cpp, whose devices derive
-- from the high-level-emulation ATA base classes.
MACHINES["ATAHLE"] = true
MACHINES["ATASTORAGE"] = true


--------------------------------------------------
-- Buses.
--
--   COCO — the cartridge bus. This is the expensive one: enabling it
--          compiles every card in src/devices/bus/coco/, which is why
--          the ATA / BBC_USERPORT / CENTRONICS / MIDI / RS232 tokens
--          below are needed. They are not wanted for their own sake —
--          they are what the cards transitively reference.
--
--   Note that RS232 *is* enabled here, unlike llvm6309 / picothing.
--   coco_rs232.cpp includes bus/rs232/rs232.h directly, so the
--   stripped-core workaround those two subtargets use buys nothing
--   in this one. See README_MRVM.md for the full explanation.
--------------------------------------------------

BUSES["COCO"] = true
BUSES["ATA"] = true
BUSES["BBC_USERPORT"] = true
BUSES["CENTRONICS"] = true
BUSES["MIDI"] = true
BUSES["RS232"] = true

-- Transitive again: bus/bbc/userport/adaptator.cpp exposes a pair of
-- VCS-style joystick ports, so the VCS control-port bus core has to
-- be present for BBC_USERPORT to link.
BUSES["VCS_CTRL"] = true


--------------------------------------------------
-- Media formats.
--
--   COCO_CAS      — cassette images
--   DMK / JVC / OS9 / SDF / VDK — the floppy image formats the CoCo
--                   and Dragon controllers accept
--   FLEX_DSK      — dragon_claw.cpp
--   FS_COCO_OS9   — OS-9 / NitrOS-9 filesystem handler (floppy
--                   browsing and image creation from the UI)
--   FS_COCO_RSDOS — Disk BASIC filesystem handler
--------------------------------------------------

FORMATS["COCO_CAS"] = true
FORMATS["DMK_DSK"] = true
FORMATS["JVC_DSK"] = true
FORMATS["OS9_DSK"] = true
FORMATS["SDF_DSK"] = true
FORMATS["VDK_DSK"] = true
FORMATS["FLEX_DSK"] = true
FORMATS["FS_COCO_OS9"] = true
FORMATS["FS_COCO_RSDOS"] = true


--------------------------------------------------
-- Transitive closure — NOT wanted for their own sake.
--
-- Everything below exists only because BUSES["RS232"], ["CENTRONICS"],
-- ["BBC_USERPORT"] and ["ATA"] each expose a "default devices" slot
-- list that names every card anyone ever wrote for them. The linker
-- pulls those objects whole out of liboptional.a to satisfy the bus
-- core, so their dependencies have to be present even though a CoCo 3
-- will never instantiate a Heathkit terminal or an Epson printer.
--
-- This is the same failure mode README_MRVM.md documents for RS232 on
-- llvm6309 / picothing, except it cannot be worked around here: those
-- two subtargets dodge it by having the driver supply its own slot
-- option function, and this subtarget's drivers are upstream's.
--
-- Grouped by what drags them in. If a future MAME adds a card to one
-- of these buses, expect a fresh crop of undefined vtables here and
-- resolve them the same way.
--------------------------------------------------

-- RS232 default terminals: heath_h19, ie15, mboardd, scorpion,
-- sun_kbd, swtpc8212, terminal, votraxtnt.
CPUS["M6800"] = true                    -- m6803 / hd6303r / hd6303x
CPUS["MCS48"] = true                    -- i8042ah / i8050
CPUS["MCS51"] = true                    -- i80c32
SOUNDS["BEEP"] = true
MACHINES["IE15"] = true
MACHINES["SWTPC8212"] = true
MACHINES["VOTRAXTNT"] = true
MACHINES["TMS1024"] = true              -- tms1025
BUSES["HEATHZENITH_H19"] = true         -- heath_tlb and its eight variants

-- CENTRONICS printers: the Epson / NEC / Citizen / Oki family.
CPUS["UPD7810"] = true                  -- upd7810 / upd7801
MACHINES["BITMAP_PRINTER"] = true
MACHINES["E05A03"] = true
MACHINES["E05A30"] = true
MACHINES["EEPROMDEV"] = true            -- eeprom_93c06_16bit
MACHINES["OUTPUT_LATCH"] = true

-- BBC_USERPORT cards: sdcard, cx85, lcd, smartboard, nss_tvinterface.
VIDEOS["HD44780"] = true
MACHINES["SPISDCARD"] = true
MACHINES["CHESSMACHINE"] = true
MACHINES["TASC_SB30"] = true
CPUS["NEC"] = true                      -- v33, via chessmachine
CPUS["FR"] = true                       -- mb91f155a, via tasc_sb30
CPUS["MC68HC11"] = true                 -- mc68hc11a1

-- ATA devices: atapicdr, gdrom, t10mmc, zip100.
SOUNDS["CDDA"] = true

-- COCO bus cards in their own right, missed above: dragon_claw
-- (keypad decoder) and meb_rtime (Disto real-time clock).
MACHINES["MM74C922"] = true             -- mm74c923
MACHINES["PCF8573"] = true

-- wd_fdc references the CoCo raw-disk format unconditionally.
FORMATS["COCO_RAWDSK"] = true

-- Second-order closure: dependencies of the cards above, found by
-- rebuilding until the undefined-vtable list came back empty.
CPUS["Z80"] = true                      -- mboardd, scorpion terminals
CPUS["ARM"] = true                      -- via sensorboard / smartboard
CPUS["IE15"] = true                     -- the ie15 terminal's own core
SOUNDS["VOTRAX_SC01"] = true            -- votraxtnt
MACHINES["INS8250"] = true              -- scorpion terminal UART
MACHINES["MM5740"] = true               -- heath_tlb keyboard encoder
MACHINES["SENSORBOARD"] = true          -- smartboard
MACHINES["STEPPERS"] = true             -- epson printer carriage
BUSES["SUNKBD"] = true                  -- rs232 sun_kbd adaptor
FORMATS["WD177X_DSK"] = true            -- base class of the coco formats
MACHINES["Z80DAISY"] = true              -- z80 terminal cores


--------------------------------------------------
-- Source files that make up this subtarget.
--
-- All of these are upstream files taken verbatim from src/mame/trs/.
-- Nothing here is forked, so an upstream rebase should carry cleanly;
-- the only thing to keep in step is this file's token list, if a new
-- card lands in src/devices/bus/coco/ that references a device not
-- yet whitelisted above.
--
-- dgnalpha.cpp and dgn_beta.cpp are deliberately absent — they are
-- separate machines, not base classes of anything registered here.
--------------------------------------------------

function createProjects_mame_coco3(_target, _subtarget)
	project ("mame_coco3")
	targetsubdir(_target .."_" .. _subtarget)
	kind (LIBTYPE)
	uuid (os.uuid("drv-mame-coco3"))
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
		-- Shared CoCo base: PIAs, keyboard, cassette, joystick, the
		-- cartridge line plumbing. coco3_state derives from this.
		MAME_DIR .. "src/mame/trs/coco.cpp",
		MAME_DIR .. "src/mame/trs/coco.h",
		MAME_DIR .. "src/mame/trs/coco_vhd.cpp",
		MAME_DIR .. "src/mame/trs/coco_vhd.h",

		-- CoCo 1/2. Needed even for a CoCo 3 only build: coco3.h
		-- includes coco12.h, and coco_cart_slot_devices() — the
		-- function that populates the cartridge slot — lives in
		-- coco12.cpp.
		MAME_DIR .. "src/mame/trs/coco12.cpp",
		MAME_DIR .. "src/mame/trs/coco12.h",
		MAME_DIR .. "src/mame/trs/coco12_m.cpp",

		-- CoCo 3 proper, plus the GIME (MMU + video, modelled as a
		-- 6883 SAM subclass).
		MAME_DIR .. "src/mame/trs/coco3.cpp",
		MAME_DIR .. "src/mame/trs/coco3.h",
		MAME_DIR .. "src/mame/trs/coco3_m.cpp",
		MAME_DIR .. "src/mame/trs/gime.cpp",
		MAME_DIR .. "src/mame/trs/gime.h",

		-- Dragon 32/64 and clones — coco12_state subclasses, so they
		-- come almost free once the above is compiled.
		MAME_DIR .. "src/mame/trs/dragon.cpp",
		MAME_DIR .. "src/mame/trs/dragon.h",
	}
end

function linkProjects_mame_coco3(_target, _subtarget)
	links {
		"mame_coco3",
	}
end
