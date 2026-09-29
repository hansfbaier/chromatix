#
# This file is part of ChromatiX.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""
LiteUSB USB 2.0 device core integration (native Migen/LiteX port of LUNA).

LiteUSB (https://github.com/hansfbaier/liteusb, BSD-3-Clause) is a native Migen/LiteX port of
LUNA's USB 2.0 device stack: reset/High-Speed chirp, packets/CRC, handshakes/data toggles,
SET_ADDRESS/SET_CONFIGURATION/GET_STATUS/..., endpoint sequencing and high-bandwidth isochronous
IN. The Chromatic class logic stays in Migen: this module exposes flat ports for:

- EP0: a request bridge (GET_DESCRIPTOR, SET/GET_INTERFACE and class requests) presenting the setup
  fields and a byte interface (txdat = byte at cdata_ofs / txpop, rxdat/rxval/rxact) to Migen, on
  top of LiteUSB's control endpoint (USBControlEndpoint) and standard request handler.
- EP2/EP5: isochronous stream IN endpoints (UVC video, 2x1024 high-bandwidth / UAC audio).
- EP3: bulk stream IN/OUT endpoints (CDC-ACM data).
- EP1/EP4: interrupt IN endpoints that always NAK (UVC status / CDC notification, unused).
"""

from migen import *

from litex.gen import *

from migen.genlib.fsm import FSM, NextState, NextValue

from usb_protocol.types     import USBRequestType, USBStandardRequests
from usb_protocol.emitters  import DeviceDescriptorCollection

from liteusb.gateware.usb.usb2.device                          import USBDevice
from liteusb.gateware.usb.usb2.request                         import USBRequestHandler, StallOnlyRequestHandler
from liteusb.gateware.usb.usb2.endpoint                        import EndpointInterface
from liteusb.gateware.usb.usb2.endpoints.stream                import USBStreamInEndpoint, USBStreamOutEndpoint
from liteusb.gateware.usb.usb2.endpoints.isochronous_stream_in import USBIsochronousStreamInEndpoint
from liteusb.gateware.usb.request.standard                     import StandardRequestHandler
from liteusb.gateware.interface.utmi                           import UTMIInterface

# Constants ----------------------------------------------------------------------------------------

EP0_MAX_PACKET_SIZE = 64

# High-Speed USB Device ----------------------------------------------------------------------------

class HSUSBDevice(USBDevice):
    """LiteUSB USBDevice on a native UTMI bus at High-Speed (LiteUSB assumes FS-only for raw UTMI buses)."""
    def __init__(self, *, bus):
        super().__init__(bus=bus, handle_clocking=False)
        self.always_fs  = False
        self.data_clock = 60e6

# Request Bridge -----------------------------------------------------------------------------------

def _bridged(setup):
    """Requests handled by the bridge (the Migen handlers), the others by LiteUSB."""
    standard = setup.type == USBRequestType.STANDARD
    return (setup.type == USBRequestType.CLASS) | (standard & (
        (setup.request == USBStandardRequests.GET_DESCRIPTOR) |
        (setup.request == USBStandardRequests.SET_INTERFACE)  |
        (setup.request == USBStandardRequests.GET_INTERFACE)))

class ChromaticRequestBridge(USBRequestHandler):
    """
    EP0 request handler forwarding the bridged requests to the Migen handlers.

    IN data stages: txdat is the byte at cdata_ofs (registered sources update it on txpop, and
    reload byte 0 when cdata_ofs returns to 0); bytes are prefetched (2-byte buffer) so that the
    Migen sources have no combinatorial path to the LiteUSB TX logic; the response length is
    min(wLength, txlen); packets of up to 64 bytes are streamed on data requests, a packet not
    ACK'ed is resent (refetched from its first offset). No txval at the request start: STALL.
    OUT data stages: rxdat/rxval per byte while rxact, then the status stage is ZLP'ed.
    SET_INTERFACE: inf_set/inf_sel/inf_alt_o after the status stage; GET_INTERFACE returns inf_alt_i.
    """
    def __init__(self):
        super().__init__()
        # Setup (to Migen).
        self.header_ready  = Signal()
        self.bmRequestType = Signal(8)
        self.bRequest      = Signal(8)
        self.wValue        = Signal(16)
        self.wIndex        = Signal(16)
        self.wLength       = Signal(16)
        self.cdata_ofs     = Signal(16)
        # IN data (from Migen).
        self.txval         = Signal()
        self.txdat         = Signal(8)
        self.txlen         = Signal(16)
        self.txpop         = Signal()
        # OUT data (to Migen).
        self.rxdat         = Signal(8)
        self.rxval         = Signal()
        self.rxact         = Signal()
        # Interfaces alternate settings.
        self.inf_set       = Signal()
        self.inf_sel       = Signal(8)
        self.inf_alt_o     = Signal(8)
        self.inf_alt_i     = Signal(8)

        # # #

        interface = self.interface
        setup     = interface.setup
        tx        = interface.tx

        is_get_interface = Signal()
        is_set_interface = Signal()
        answered         = Signal()   # Migen handler answering the IN request (latched).
        txval_r          = Signal()   # Registered Migen inputs (timing: no combinatorial path from
        txlen_r          = Signal(16) # the Migen handlers to the LiteUSB TX logic).
        total            = Signal(16) # IN response length (latched with answered: the handlers
                                      # may drop txval/txlen once their last byte is popped).
        pkt_start        = Signal(16)
        pkt_len          = Signal(8)
        pkt_count        = Signal(8)
        expecting_ack    = Signal()
        settle           = Signal(3)  # Migen handlers latency (registered lookups/answer).
        pending          = Signal()   # Bridged SETUP received, to be started from IDLE.

        # IN data prefetch (2 bytes): bytes are popped from the Migen source (byte at cdata_ofs,
        # next one on txpop) ahead of the transmission; the packet data comes from these registers.
        fifo_data  = [Signal(8, name=f"fifo_data{i}") for i in range(2)]
        fifo_level = Signal(2)
        fifo_rd    = Signal()
        fifo_wr    = Signal()
        fetching   = Signal()
        flush      = Signal()
        txdat      = Signal(8)

        bridged = _bridged(setup)

        # FSM (states are entered/stayed as described in the state comments below).
        fsm = FSM(reset_state="IDLE")
        self.submodules.fsm = fsm = ClockDomainsRenamer("usb")(fsm)

        # FSM state strobes (used by the registered/comb logic outside the state bodies).
        in_idle   = fsm.ongoing("IDLE")
        in_in_ack = fsm.ongoing("IN_ACK")

        resend = in_in_ack & interface.data_requested & ~interface.handshakes_in.ack

        self.comb += [
            txdat.eq(Mux(is_get_interface, self.inf_alt_i, self.txdat)),
            fetching.eq((settle == 0) & (fifo_level < 2) & (self.cdata_ofs < total) & answered),
            fifo_wr.eq(fetching),
            self.txpop.eq(fetching & ~is_get_interface),
            flush.eq(in_idle | resend),
            pkt_len.eq(Mux((total - pkt_start) < EP0_MAX_PACKET_SIZE,
                (total - pkt_start)[:8], EP0_MAX_PACKET_SIZE)),
            # Interface selection: the current request only (inf_set pulses on SET_INTERFACE).
            self.inf_sel.eq(self.wIndex[:8]),
            # Request routing: the bridge handles the class requests and the bridged standard ones.
            interface.claim.eq((setup.type == USBRequestType.CLASS) |
                ((setup.type == USBRequestType.STANDARD) & bridged)),
        ]

        # IN data prefetch FIFO: flushed when idle (or when a packet has to be resent).
        self.sync.usb += [
            If(flush,
                fifo_level.eq(0),
            ).Else(
                If(fifo_rd,
                    fifo_data[0].eq(fifo_data[1]),
                ),
                If(fifo_wr,
                    # Write index: fifo_level - fifo_rd (0 or 1).
                    If(fifo_level - fifo_rd == 0,
                        fifo_data[0].eq(txdat),
                    ).Else(
                        fifo_data[1].eq(txdat),
                    ),
                ),
                fifo_level.eq(fifo_level + fifo_wr - fifo_rd),
            ),
            # Data stage byte offset: cleared when idle, rewound on a resend, advanced per byte.
            If(in_idle,
                self.cdata_ofs.eq(0),
            ).Elif(resend,
                self.cdata_ofs.eq(pkt_start),
            ).Elif(self.rxval | fifo_wr,
                self.cdata_ofs.eq(self.cdata_ofs + 1),
            ),
            # Migen handlers latency (registered lookup of the answer), decremented each cycle.
            If(in_idle & pending,
                settle.eq(6),
            ).Elif(resend,
                settle.eq(2),
            ).Elif(settle != 0,
                settle.eq(settle - 1),
            ),
            # Registered Migen inputs (reply available / reply length).
            txval_r.eq(Mux(is_get_interface, 1, self.txval)),
            txlen_r.eq(Mux(is_get_interface, 1, self.txlen)),
            # Bridged SETUP received: to be started from IDLE (IDLE wins if it starts one now).
            If(in_idle & pending,
                pending.eq(0),
            ).Elif(setup.received,
                pending.eq(bridged),
            ),
        ]

        # A SETUP aborts any control transfer in progress (the host may abandon one, e.g. after a
        # timeout): back to IDLE, which starts the new request when bridged.
        def abort_on_setup():
            return If(setup.received, NextState("IDLE"))

        # IDLE -- waits for a bridged SETUP, then presents it to the Migen handlers.
        fsm.act("IDLE",
            NextValue(self.rxval, 0),
            NextValue(self.header_ready, 0),
            NextValue(pkt_start, 0),
            NextValue(answered, 0),
            NextValue(expecting_ack, 0),
            NextValue(interface.tx_data_pid, 1), # Data stages start with DATA1.
            If(pending,
                NextValue(self.header_ready, 1),
                NextValue(self.bmRequestType, Cat(setup.recipient, setup.type, setup.is_in_request)),
                NextValue(self.bRequest, setup.request),
                NextValue(self.wValue, setup.value),
                NextValue(self.wIndex, setup.index),
                NextValue(self.wLength, setup.length),
                NextValue(is_get_interface, (setup.type == USBRequestType.STANDARD) &
                    (setup.request == USBStandardRequests.GET_INTERFACE)),
                NextValue(is_set_interface, (setup.type == USBRequestType.STANDARD) &
                    (setup.request == USBStandardRequests.SET_INTERFACE)),
                If(setup.is_in_request & (setup.length != 0),
                    NextState("IN_WAIT"),
                ).Elif(setup.length != 0,
                    NextState("OUT_DATA"),
                ).Else(
                    NextState("NO_DATA"),
                ),
            ),
        )

        # IN data stage: latch whether a Migen handler answers (registered txval), then data.
        fsm.act("IN_WAIT",
            If(settle == 0,
                NextValue(answered, txval_r),
                NextValue(total, Mux(txlen_r < self.wLength, txlen_r, self.wLength)),
                NextState("IN_DATA"),
            ),
            abort_on_setup(),
        )
        fsm.act("IN_DATA",
            If(interface.data_requested,
                If(~answered,
                    interface.handshakes_out.stall.eq(1),
                    NextState("IDLE"),
                ).Elif(pkt_len == 0,
                    tx.valid.eq(1),
                    tx.last.eq(1), # ZLP.
                    NextValue(expecting_ack, 1),
                ).Else(
                    NextValue(pkt_count, 0),
                    NextValue(expecting_ack, 1),
                    NextState("IN_SEND"),
                ),
            ).Elif(interface.handshakes_in.ack & expecting_ack,
                NextValue(pkt_start, pkt_start + pkt_len),
                NextValue(interface.tx_data_pid, ~interface.tx_data_pid),
                NextValue(expecting_ack, 0),
            ),
            If(interface.status_requested,
                interface.handshakes_out.ack.eq(1),
                NextState("IDLE"),
            ),
            abort_on_setup(),
        )
        fsm.act("IN_SEND",
            tx.valid.eq(fifo_level != 0),
            tx.payload.eq(fifo_data[0]),
            tx.first.eq(pkt_count == 0),
            tx.last.eq(pkt_count == (pkt_len - 1)),
            fifo_rd.eq(tx.ready & (fifo_level != 0)),
            If(fifo_rd,
                NextValue(pkt_count, pkt_count + 1),
                If(pkt_count == (pkt_len - 1),
                    NextState("IN_ACK"),
                ),
            ),
            abort_on_setup(),
        )
        fsm.act("IN_ACK",
            # ACK'ed: continue (prefetched data kept), else resend the packet (refetched).
            If(interface.handshakes_in.ack,
                NextValue(pkt_start, pkt_start + pkt_len),
                NextValue(interface.tx_data_pid, ~interface.tx_data_pid),
                NextValue(expecting_ack, 0),
                NextState("IN_DATA"),
            ).Elif(interface.data_requested,
                # Not ACK'ed, re-requested: resend (refetched from the packet start).
                NextValue(pkt_count, 0),
                NextState("IN_SEND"),
            ).Elif(interface.status_requested,
                interface.handshakes_out.ack.eq(1),
                NextState("IDLE"),
            ),
            abort_on_setup(),
        )

        # OUT data stage (then status: IN ZLP).
        fsm.act("OUT_DATA",
            # rxval/rxdat registered together, cdata_ofs advanced after the byte (rxdat is the
            # byte at cdata_ofs while rxval).
            self.rxact.eq(1),
            NextValue(self.rxval, interface.rx.valid & interface.rx.next),
            If(interface.rx.valid & interface.rx.next,
                NextValue(self.rxdat, interface.rx.payload),
            ),
            If(interface.rx_ready_for_response,
                interface.handshakes_out.ack.eq(1),
            ),
            If(interface.status_requested,
                *self.send_zlp(),
            ),
            If(interface.handshakes_in.ack,
                NextState("IDLE"),
            ),
            abort_on_setup(),
        )

        # No data stage (status: IN ZLP).
        fsm.act("NO_DATA",
            If(interface.status_requested,
                *self.send_zlp(),
            ),
            If(interface.handshakes_in.ack,
                If(is_set_interface,
                    self.inf_set.eq(1),
                    self.inf_alt_o.eq(self.wValue[:8]),
                ),
                NextState("IDLE"),
            ),
            abort_on_setup(),
        )

# NAK Endpoint -------------------------------------------------------------------------------------

class NAKEndpoint(Module):
    """IN endpoint that always NAKs (declared endpoint without data)."""
    def __init__(self, *, endpoint_number):
        self._endpoint_number = endpoint_number
        self.interface        = EndpointInterface()

        # # #

        tokenizer = self.interface.tokenizer
        self.comb += self.interface.handshakes_out.nak.eq(tokenizer.is_in & tokenizer.ready_for_response &
            (tokenizer.endpoint == self._endpoint_number))

# USB Device Core ----------------------------------------------------------------------------------

class USBDeviceCore(LiteXModule):
    """
    LiteUSB HS device on UTMI + Chromatic endpoints, with the same flat ports as the former LUNA core.

    iso_endpoints: {endpoint_number: max_packet_size} (isochronous stream IN).
    bulk_endpoints: {endpoint_number: max_packet_size} (stream IN + OUT).
    nak_endpoints: IN endpoints that always NAK.
    bus: UTMI bus used directly (simulation: the flat UTMI ports and the "usb" clock domain are unused).
    reset: additional reset of the core (start-up: kept in reset while the PHY is not running).
    """
    def __init__(self, iso_endpoints=None, bulk_endpoints=None, nak_endpoints=None, bus=None, reset=None):
        iso_endpoints  = {2: 1024, 5: 24} if iso_endpoints  is None else iso_endpoints
        bulk_endpoints = {3: 512}        if bulk_endpoints is None else bulk_endpoints
        nak_endpoints  = [1, 4]          if nak_endpoints  is None else nak_endpoints

        self.iso_endpoints  = iso_endpoints
        self.bulk_endpoints = bulk_endpoints
        self.nak_endpoints  = nak_endpoints
        self.simulation     = bus is not None
        self.endpoints      = {} # Debug/simulation access.

        # UTMI (flat ports: hardware only).
        self.utmi_rx_data     = Signal(8)
        self.utmi_rx_active   = Signal()
        self.utmi_rx_valid    = Signal()
        self.utmi_rx_error    = Signal()
        self.utmi_line_state  = Signal(2)
        self.utmi_tx_ready    = Signal()
        self.utmi_tx_data     = Signal(8)
        self.utmi_tx_valid    = Signal()
        self.utmi_op_mode     = Signal(2)
        self.utmi_xcvr_select = Signal(2)
        self.utmi_term_select = Signal()
        # Device status.
        self.sof              = Signal()
        self.bus_reset        = Signal()
        self.high_speed       = Signal()
        # EP0 bridge.
        self.bridge           = ChromaticRequestBridge()
        for name, width in [
            ("header_ready",  1), ("bmRequestType", 8), ("bRequest", 8),
            ("wValue",       16), ("wIndex",       16), ("wLength",  16),
            ("cdata_ofs",    16), ("txval",         1), ("txdat",     8),
            ("txlen",        16), ("txpop",         1), ("rxdat",     8),
            ("rxval",         1), ("rxact",         1), ("inf_set",   1),
            ("inf_sel",       8), ("inf_alt_o",     8), ("inf_alt_i", 8)]:
            setattr(self, f"ep0_{name}", Signal(width))
        # Isochronous IN endpoints.
        for n in iso_endpoints:
            for name, width in [("data", 8), ("valid", 1), ("ready", 1), ("bytes", 12),
                ("requested", 1), ("finished", 1)]:
                setattr(self, f"ep{n}_{name}", Signal(width))
        # Bulk IN/OUT endpoints.
        for n in bulk_endpoints:
            for name, width in [("in_data", 8), ("in_valid", 1), ("in_ready", 1), ("in_flush", 1),
                ("out_data", 8), ("out_valid", 1), ("out_ready", 1)]:
                setattr(self, f"ep{n}_{name}", Signal(width))

        # # #

        # UTMI bus (hardware: connected to the flat ports; simulation: driven by the test bench).
        self.utmi = utmi = UTMIInterface() if bus is None else bus

        # Device.
        self.device = device = HSUSBDevice(bus=utmi)
        self.comb += [
            device.connect.eq(1),
            self.sof.eq(device.sof_detected),
            self.bus_reset.eq(device.reset_detected),
            self.high_speed.eq(device.speed == 0),
        ]

        # EP0: LiteUSB standard requests (except the bridged ones), bridge, stall on the others.
        control = device.add_control_endpoint()
        control.add_request_handler(StandardRequestHandler(DeviceDescriptorCollection(),
            skiplist=[_bridged]))
        control.add_request_handler(self.bridge)
        control.add_request_handler(StallOnlyRequestHandler(
            stall_condition=lambda setup: (setup.type == USBRequestType.VENDOR)))

        # EP0 bridge <-> flat ports.
        b = self.bridge
        for name in ["header_ready", "bmRequestType", "bRequest", "wValue", "wIndex", "wLength",
            "cdata_ofs", "txpop", "rxdat", "rxval", "rxact", "inf_set", "inf_sel", "inf_alt_o"]:
            self.comb += getattr(self, "ep0_" + name).eq(getattr(b, name))
        for name in ["txval", "txdat", "txlen", "inf_alt_i"]:
            self.comb += getattr(b, name).eq(getattr(self, "ep0_" + name))

        # Isochronous IN endpoints.
        for n, max_packet_size in iso_endpoints.items():
            ep = USBIsochronousStreamInEndpoint(endpoint_number=n, max_packet_size=max_packet_size)
            device.add_endpoint(ep)
            self.endpoints[f"iso{n}"] = ep
            self.comb += [
                ep.stream.payload.eq(getattr(self, f"ep{n}_data")),
                ep.stream.valid.eq(getattr(self, f"ep{n}_valid")),
                getattr(self, f"ep{n}_ready").eq(ep.stream.ready),
                ep.bytes_in_frame.eq(getattr(self, f"ep{n}_bytes")),
                getattr(self, f"ep{n}_requested").eq(ep.data_requested),
                getattr(self, f"ep{n}_finished").eq(ep.frame_finished),
            ]

        # Bulk IN/OUT endpoints.
        for n, max_packet_size in bulk_endpoints.items():
            ep_in  = USBStreamInEndpoint(endpoint_number=n, max_packet_size=max_packet_size)
            ep_out = USBStreamOutEndpoint(endpoint_number=n, max_packet_size=max_packet_size)
            device.add_endpoint(ep_in)
            device.add_endpoint(ep_out)
            self.endpoints[f"bulk{n}_in"]  = ep_in
            self.endpoints[f"bulk{n}_out"] = ep_out
            self.comb += [
                ep_in.stream.payload.eq(getattr(self, f"ep{n}_in_data")),
                ep_in.stream.valid.eq(getattr(self, f"ep{n}_in_valid")),
                getattr(self, f"ep{n}_in_ready").eq(ep_in.stream.ready),
                ep_in.flush.eq(getattr(self, f"ep{n}_in_flush")),
                getattr(self, f"ep{n}_out_data").eq(ep_out.stream.payload),
                getattr(self, f"ep{n}_out_valid").eq(ep_out.stream.valid),
                ep_out.stream.ready.eq(getattr(self, f"ep{n}_out_ready")),
            ]

        # NAK-only IN endpoints.
        for n in nak_endpoints:
            device.add_endpoint(NAKEndpoint(endpoint_number=n))

        # Hardware: UTMI flat ports and core clock domain (the UTMI clock, with the core reset).
        if not self.simulation:
            self.comb += [
                utmi.rx_data.eq(self.utmi_rx_data),
                utmi.rx_active.eq(self.utmi_rx_active),
                utmi.rx_valid.eq(self.utmi_rx_valid),
                utmi.rx_error.eq(self.utmi_rx_error),
                utmi.line_state.eq(self.utmi_line_state),
                utmi.tx_ready.eq(self.utmi_tx_ready),
                utmi.vbus_valid.eq(1),
                utmi.session_valid.eq(1),
                self.utmi_tx_data.eq(utmi.tx_data),
                self.utmi_tx_valid.eq(utmi.tx_valid),
                self.utmi_op_mode.eq(utmi.op_mode),
                self.utmi_xcvr_select.eq(utmi.xcvr_select),
                self.utmi_term_select.eq(utmi.term_select),
            ]
            self.cd_usb = ClockDomain("usb")
            self.comb += [
                self.cd_usb.clk.eq(ClockSignal("phy")),
                self.cd_usb.rst.eq(reset if reset is not None else ResetSignal("phy")),
            ]

# Descriptors Request Handler ----------------------------------------------------------------------

class USBDescriptorRequest(LiteXModule):
    """
    GET_DESCRIPTOR from the descriptors ROM (USBDescriptors): device, configuration (same for FS/HS),
    device qualifier and strings; other types are not answered (STALL). The descriptor lookup is
    registered (setup fields are stable during the request), the ROM is read asynchronously at the
    descriptor address + cdata_ofs.
    """
    def __init__(self, setup, desc):
        self.reset     = Signal()
        self.txval     = Signal()
        self.txdat     = Signal(8)
        self.txdat_len = Signal(12)
        self.txpop     = Signal() # Unused (the ROM is addressed by cdata_ofs).

        # # #

        l = desc.layout
        s = setup
        addr   = Signal(16)
        length = Signal(16)
        found  = Signal()
        table  = [
            # (type, index, address, length).
            (1, 0, l.dev_addr,        l.dev_len),
            (2, 0, l.hscfg_addr,      l.hscfg_len),
            (6, 0, l.qual_addr,       l.qual_len),
            (3, 0, l.strlang_addr,    4),
            (3, 1, l.strvendor_addr,  l.strvendor_len),
            (3, 2, l.strproduct_addr, l.strproduct_len),
            (3, 3, l.strserial_addr,  l.strserial_len),
        ]
        cases = {"default": [addr.eq(0), length.eq(0), found.eq(0)]}
        for dtype, index, a, n in table:
            cases[(dtype << 8) | index] = [addr.eq(a), length.eq(n), found.eq(1)]
        is_get_descriptor = Signal()
        self.sync += [
            Case(s.wValue, cases),
            is_get_descriptor.eq(s.header_ready & (s.bmRequestType == 0x80) & (s.bRequest == 0x06)),
        ]
        self.comb += [
            self.txval.eq(s.header_ready & is_get_descriptor & found),
            self.txdat_len.eq(length),
            desc.descrom_raddr.eq(addr + s.cdata_ofs),
            self.txdat.eq(desc.descrom_rdat),
        ]
