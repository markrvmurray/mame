// license:BSD-3-Clause
// copyright-holders:smf

// Stripped copy of src/devices/bus/rs232/rs232.cpp — the RS-232 bus core
// (device class + interface) with the terminal-card includes and the
// default_rs232_devices() factory function removed. Used by the llvm6309
// subtarget which does NOT set BUSES["RS232"] = true, so bus.lua does not
// compile the real rs232.cpp. This file supplies the same bus-core symbols
// without dragging in every terminal card's transitive dependencies
// (beep, mc6845, ay8910, m6800, ie15, heath h19, sun_kbd, ...).
//
// Upstream rs232.cpp comment header retained below for clarity. Keep this
// file in sync with the upstream bus-core portion if it changes.

/***************************************************************************

    Generic EIA RS-232/CCITT V.24 serial port emulation — bus core only.

***************************************************************************/

#include "emu.h"
#include "bus/rs232/rs232.h"

DEFINE_DEVICE_TYPE(RS232_PORT, rs232_port_device, "rs232", "RS-232 Port")

rs232_port_device::rs232_port_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	rs232_port_device(mconfig, RS232_PORT, tag, owner, clock)
{
}

rs232_port_device::rs232_port_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, type, tag, owner, clock),
	device_single_card_slot_interface<device_rs232_port_interface>(mconfig, *this),
	m_rxd(0),
	m_dcd(0),
	m_dsr(0),
	m_ri(0),
	m_si(0),
	m_cts(0),
	m_dce_rxc(0),
	m_dce_txc(0),
	m_rxd_handler(*this),
	m_dcd_handler(*this),
	m_dsr_handler(*this),
	m_ri_handler(*this),
	m_si_handler(*this),
	m_cts_handler(*this),
	m_rxc_handler(*this),
	m_txc_handler(*this),
	m_dev(nullptr)
{
}

rs232_port_device::~rs232_port_device()
{
}

void rs232_port_device::device_config_complete()
{
	m_dev = get_card_device();
}

void rs232_port_device::device_resolve_objects()
{
	m_rxd = 1;
	m_dcd = 1;
	m_dsr = 1;
	m_ri = 1;
	m_si = 1;
	m_cts = 1;
}

void rs232_port_device::device_reset()
{
	m_rxd_handler(m_rxd);
	m_dcd_handler(m_dcd);
	m_dsr_handler(m_dsr);
	m_ri_handler(m_ri);
	m_si_handler(m_si);
	m_cts_handler(m_cts);
}

void rs232_port_device::device_start()
{
	save_item(NAME(m_rxd));
	save_item(NAME(m_dcd));
	save_item(NAME(m_dsr));
	save_item(NAME(m_ri));
	save_item(NAME(m_si));
	save_item(NAME(m_cts));
	save_item(NAME(m_dce_rxc));
	save_item(NAME(m_dce_txc));
}

void rs232_port_device::write_txd(int state)
{
	if (m_dev)
		m_dev->input_txd(state);
}

void rs232_port_device::write_dtr(int state)
{
	if (m_dev)
		m_dev->input_dtr(state);
}

void rs232_port_device::write_rts(int state)
{
	if (m_dev)
		m_dev->input_rts(state);
}

void rs232_port_device::write_etc(int state)
{
	if (m_dev)
		m_dev->input_etc(state);
}

void rs232_port_device::write_spds(int state)
{
	if (m_dev)
		m_dev->input_spds(state);
}

device_rs232_port_interface::device_rs232_port_interface(const machine_config &mconfig, device_t &device) :
	device_interface(device, "rs232")
{
	m_port = dynamic_cast<rs232_port_device *>(device.owner());
}

device_rs232_port_interface::~device_rs232_port_interface()
{
}
