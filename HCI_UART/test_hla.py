# Regression tests for the UART HCI high level analyzer.
#
# Runs the real HighLevelAnalyzer.py against a stub of the saleae.analyzers API, so no
# Logic 2 install or hardware is needed:  python HCI_UART/test_hla.py

import os
import sys
import types
import importlib


def _install_saleae_stub():
    saleae = types.ModuleType("saleae")
    analyzers = types.ModuleType("saleae.analyzers")

    class HighLevelAnalyzer:
        pass

    class AnalyzerFrame:
        def __init__(self, frame_type, start_time, end_time, data):
            self.type = frame_type
            self.start_time = start_time
            self.end_time = end_time
            self.data = data

    def setting(**kwargs):
        return None

    analyzers.HighLevelAnalyzer = HighLevelAnalyzer
    analyzers.AnalyzerFrame = AnalyzerFrame
    analyzers.ChoicesSetting = setting
    analyzers.StringSetting = setting
    analyzers.NumberSetting = setting

    saleae.analyzers = analyzers
    sys.modules["saleae"] = saleae
    sys.modules["saleae.analyzers"] = analyzers


_install_saleae_stub()
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
HLA = importlib.import_module("HighLevelAnalyzer")


class UartFrame:
    """Stand-in for one Async Serial byte frame. Pass byte=None for a UART error."""

    def __init__(self, byte, t):
        self.start_time = t
        self.end_time = t
        if byte is None:
            self.data = {'error': True}
        else:
            self.data = {'data': bytes([byte])}


def make_hla(role="Host->Controller", mode="Always", trigger="",
             direction="Auto", sync=3, rearm="On Resync"):
    HLA.Hla.s1_role_choice = role
    HLA.Hla.s2_decode_mode = mode
    HLA.Hla.s3_decode_trigger_frame = trigger
    HLA.Hla.s4_direction_mode = direction
    HLA.Hla.s5_sync_packets = sync
    HLA.Hla.s6_trigger_rearm = rearm
    return HLA.Hla()


def drive(hla, stream):
    """Feed a byte stream (ints, or None for UART errors) and collect emitted packets."""
    out = []
    for i, byte in enumerate(stream):
        result = hla.decode(UartFrame(byte, i))
        if result is None:
            continue
        if isinstance(result, list):
            out.extend(f.data['data'] for f in result)
        else:
            out.append(result.data['data'])
    return out


# packet builders --------------------------------------------------------------

def cmd(opcode_lo, opcode_hi, *params):
    return [0x01, opcode_lo, opcode_hi, len(params), *params]

def evt(code, *params):
    return [0x04, code, len(params), *params]

def acl(handle_lo, handle_hi, *payload):
    n = len(payload)
    return [0x02, handle_lo, handle_hi, n & 0xFF, (n >> 8) & 0xFF, *payload]

RESET_CMD = cmd(0x03, 0x0C)                     # 01 03 0C 00
RESET_EVT = evt(0x0E, 0x01, 0x03, 0x0C, 0x00)   # 04 0E 04 01 03 0C 00
GARBAGE = [0x00, 0xFF, 0x7F, 0x80, 0xAB, 0x13, 0xC4, 0x55, 0x99, 0x2A, 0xEE, 0x37]


# tests ------------------------------------------------------------------------

RESULTS = []

def check(name, got, expected):
    ok = got == expected
    RESULTS.append(ok)
    mark = "ok  " if ok else "FAIL"
    print(f"[{mark}] {name}")
    if not ok:
        print(f"       expected: {expected}")
        print(f"       got:      {got}")


def test_normal_h_c():
    hla = make_hla(role="Host->Controller")
    stream = RESET_CMD + cmd(0x01, 0x10) + acl(0x40, 0x20, 1, 2, 3) + cmd(0x05, 0x10)
    out = drive(hla, stream)
    check("normal H->C wiring",
          out,
          ["H->C:01 03 0C 00", "H->C:01 01 10 00",
           "H->C:02 40 20 03 00 01 02 03", "H->C:01 05 10 00"])


def test_reversed_probe_auto():
    # Event stream captured by an analyzer whose role is (wrongly) Host->Controller.
    # Auto direction must relabel it C->H off the packet type.
    hla = make_hla(role="Host->Controller", direction="Auto")
    out = drive(hla, RESET_EVT + evt(0x0E, 0x01, 0x01, 0x10, 0x00) + evt(0xFF, 0x11))
    check("reversed probe corrected by Auto",
          out,
          ["C->H:04 0E 04 01 03 0C 00", "C->H:04 0E 04 01 01 10 00", "C->H:04 FF 01 11"])


def test_manual_keeps_role():
    hla = make_hla(role="Host->Controller", direction="Manual")
    out = drive(hla, RESET_EVT + evt(0x0E, 0x01, 0x01, 0x10, 0x00) + evt(0xFF, 0x11))
    check("Manual mode trusts the role label",
          out,
          ["H->C:04 0E 04 01 03 0C 00", "H->C:04 0E 04 01 01 10 00", "H->C:04 FF 01 11"])


def test_midstream_start():
    # Capture begins inside a packet; the leading garbage must not become a packet.
    stream = GARBAGE + RESET_EVT + evt(0x0E, 0x01, 0x01, 0x10, 0x00) + evt(0xFF, 0xAA)
    hla = make_hla(role="Controller->Host")
    out = drive(hla, stream)
    check("garbage before first packet is skipped",
          out,
          ["C->H:04 0E 04 01 03 0C 00", "C->H:04 0E 04 01 01 10 00", "C->H:04 FF 01 AA"])


def test_trigger_filters_pre_reset():
    # Vendor commands at a different baud rate precede Reset; only Reset onward is kept.
    pre = cmd(0x00, 0xFC, 0xDE, 0xAD) + cmd(0x01, 0xFC, 0xBE, 0xEF)
    post = RESET_CMD + cmd(0x01, 0x10) + cmd(0x05, 0x10) + cmd(0x06, 0x10)
    hla = make_hla(mode="Trigger", trigger="01 03 0C 00")
    out = drive(hla, pre + post)
    check("trigger drops pre-Reset vendor commands",
          out,
          ["H->C:01 03 0C 00", "H->C:01 01 10 00",
           "H->C:01 05 10 00", "H->C:01 06 10 00"])


def test_trigger_packet_level():
    # Trigger on the Command Complete event, whose tail (01 03 0C 00) also happens to be
    # a valid-looking Command. Packet-level matching must anchor on the whole event, not
    # on that tail, and must not manufacture a Command from inside the event's payload.
    post = RESET_EVT + evt(0x0E, 0x01, 0x01, 0x10, 0x00) + evt(0xFF, 0x11) + evt(0xFF, 0x22)
    hla = make_hla(mode="Trigger", trigger="04 0E 04 01 03 0C 00")
    out = drive(hla, GARBAGE + post)
    check("trigger matches whole packets, not payload bytes",
          out,
          ["C->H:04 0E 04 01 03 0C 00", "C->H:04 0E 04 01 01 10 00",
           "C->H:04 FF 01 11", "C->H:04 FF 01 22"])


def test_trigger_wildcard_and_alternatives():
    # num_HCI_Command_Packets (the byte after the event length) is 0x02 here, so a
    # literal 01 would miss; '??' matches it. The alternative also lets one setting
    # catch either direction of the pair.
    hla = make_hla(mode="Trigger", trigger="01 03 0C 00 | 04 0E 04 ?? 03 0C 00")
    out = drive(hla, GARBAGE + evt(0x0E, 0x02, 0x03, 0x0C, 0x00)
                     + evt(0xFF, 0x11) + evt(0xFF, 0x22) + evt(0xFF, 0x33))
    check("trigger wildcard + alternative",
          out,
          ["C->H:04 0E 04 02 03 0C 00", "C->H:04 FF 01 11",
           "C->H:04 FF 01 22", "C->H:04 FF 01 33"])


def test_multi_session_reset():
    # bad bytes / good frames / bad bytes / good frames -- board reset mid-capture.
    # Each good stretch has its own pre-Reset vendor traffic that must be filtered,
    # which only works if the trigger re-arms on every resync.
    session = (cmd(0x00, 0xFC, 0x11) + RESET_CMD + cmd(0x01, 0x10) + cmd(0x05, 0x10))
    stream = GARBAGE + session + GARBAGE + [None, None] + session + GARBAGE
    hla = make_hla(mode="Trigger", trigger="01 03 0C 00", rearm="On Resync")
    out = drive(hla, stream)
    one = ["H->C:01 03 0C 00", "H->C:01 01 10 00", "H->C:01 05 10 00"]
    check("multi-session reset filters every pre-Reset block", out, one + one)


def test_uart_error_resyncs():
    block = RESET_EVT + evt(0x0E, 0x01, 0x01, 0x10, 0x00) + evt(0xFF, 0x11)
    stream = block + [None] + GARBAGE + block
    hla = make_hla(role="Controller->Host")
    out = drive(hla, stream)
    one = ["C->H:04 0E 04 01 03 0C 00", "C->H:04 0E 04 01 01 10 00", "C->H:04 FF 01 11"]
    check("UART error frame forces a resync", out, one + one)


def test_large_acl():
    payload = list(range(256)) * 2         # 512-byte payload, length field 0x0200
    big = acl(0x01, 0x20, *payload)
    hla = make_hla(role="Host->Controller")
    out = drive(hla, RESET_CMD + big + cmd(0x01, 0x10) + cmd(0x05, 0x10))
    expected = "H->C:" + " ".join("%02X" % b for b in big)
    check("large ACL packet parses", out[1], expected)


def main():
    for test in (test_normal_h_c, test_reversed_probe_auto, test_manual_keeps_role,
                 test_midstream_start, test_trigger_filters_pre_reset,
                 test_trigger_packet_level, test_trigger_wildcard_and_alternatives,
                 test_multi_session_reset, test_uart_error_resyncs, test_large_acl):
        test()

    passed = sum(RESULTS)
    print(f"\n{passed}/{len(RESULTS)} passed")
    sys.exit(0 if passed == len(RESULTS) else 1)


if __name__ == "__main__":
    main()
