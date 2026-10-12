#!/usr/bin/env python3
"""Hardware checks for dual CDC. Needs pyserial; --usb also needs PyUSB access.

Default: production firmware (CDC2 echoes). --echo-both: usb_cdc_echo firmware.
The USB checks temporarily detach Linux CDC drivers, then reattach them.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import random
import struct
import time

import serial
from serial.tools import list_ports


def ports():
    found = sorted(p.device for p in list_ports.comports()
                   if (p.vid, p.pid) == (0x16C0, 0x27DD))
    assert len(found) == 2, f"Expected two CDC ports, found {found}"
    return found


def exchange(port, data):
    port.write(data)
    port.flush()
    received = bytearray()
    deadline = time.monotonic() + 5
    while len(received) < len(data) and time.monotonic() < deadline:
        received.extend(port.read(len(data) - len(received)))
    assert received == data, f"{port.port}: {len(data)} sent, {len(received)} received/mismatch"


def serial_checks(echo_both):
    names = ports()
    with serial.Serial(names[0], 9600, timeout=0.2, write_timeout=2) as first, \
         serial.Serial(names[1], 230400, timeout=0.2, write_timeout=2) as second:
        first.reset_input_buffer()
        second.reset_input_buffer()
        first.rts = False
        second.rts = True
        rng = random.Random(35)
        for port in ([first, second] if echo_both else [second]):
            for size in (1, 7, 8, 63, 64, 65, 127, 128, 129, 255, 256, 257, 1024):
                exchange(port, rng.randbytes(size))
        print("PASS serial: packet boundaries 1..1024, binary data")
        if echo_both:
            def stream(port, seed):
                rng = random.Random(seed)
                for _ in range(128):
                    exchange(port, rng.randbytes(256))
            with ThreadPoolExecutor(max_workers=2) as pool:
                jobs = [pool.submit(stream, first, 1), pool.submit(stream, second, 2)]
                for job in jobs:
                    job.result()
            print("PASS serial: concurrent 32768 bytes per CDC, no loss/cross-talk")
        # Reopening one port must leave the other operational.
        first.close()
        exchange(second, b"other port closed\r\n")
        first.open()
        if echo_both:
            exchange(first, b"reopened\r\n")
        exchange(second, b"other port reopened\r\n")
        print("PASS serial: independent close/reopen")


def usb_checks(echo_both):
    import usb.core
    import usb.util
    dev = usb.core.find(idVendor=0x16C0, idProduct=0x27DD)
    assert dev is not None
    detached, claimed = [], []
    # A virtual USBIP host can acknowledge configuration/reset operations without
    # delivering the same operation to the physical device. Keep those checks
    # for direct USB connections; normal CDC and class transfers work over USBIP.
    usbip = any('vhci_hcd' in str(p.resolve()) for p in
                Path('/sys/bus/usb/devices').glob(f'{dev.bus}-*')
                if (p / 'devnum').is_file() and
                (p / 'devnum').read_text().strip() == str(dev.address))

    def claim_interfaces():
        for interface in range(4):
            if dev.is_kernel_driver_active(interface):
                dev.detach_kernel_driver(interface)
                if interface not in detached:
                    detached.append(interface)
            usb.util.claim_interface(dev, interface)
            claimed.append(interface)

    def release_interfaces():
        for interface in reversed(claimed):
            usb.util.release_interface(dev, interface)
        claimed.clear()

    def configure(value):
        release_interfaces()
        dev.set_configuration(value)
        if value:
            claim_interfaces()

    def control(typ, request, value=0, index=0, data=0):
        return dev.ctrl_transfer(typ, request, value, index, data, timeout=1000)

    def stall(typ, request, value, index, data):
        try:
            control(typ, request, value, index, data)
        except usb.core.USBError as error:
            assert error.errno == 32, error
        else:
            raise AssertionError("Malformed control request did not STALL")

    def read_exact(endpoint, size):
        data = bytearray()
        deadline = time.monotonic() + 5
        while len(data) < size and time.monotonic() < deadline:
            try:
                data.extend(dev.read(endpoint, 2048, timeout=500))
            except usb.core.USBTimeoutError:
                pass
        assert len(data) == size, (hex(endpoint), size, len(data))
        return bytes(data)

    def exchange_raw(endpoint, payload):
        assert dev.write(endpoint, payload, timeout=1000) == len(payload)
        assert read_exact(endpoint | 0x80, len(payload)) == payload

    try:
        if bytes(control(0x80, 8, 0, 0, 1)) == b'\0':
            dev.set_configuration(1)
        claim_interfaces()
        assert (dev.bDeviceClass, dev.bDeviceSubClass, dev.bDeviceProtocol) == (0xEF, 2, 1)
        cfg = bytes(control(0x80, 6, 0x0200, 0, 255))
        assert len(cfg) == 141 and struct.unpack_from('<H', cfg, 2)[0] == 141 and cfg[4] == 4
        assert {ep.bEndpointAddress for interface in dev.get_active_configuration()
                for ep in interface} == {0x81, 0x02, 0x82, 0x85, 0x03, 0x83}
        print("PASS USB: IAD composite descriptor, 4 interfaces, 6 endpoints")
        # Start without data left over from the serial checks. Changing 0 -> 1
        # guarantees host and device toggles are reset, including on Linux USBFS.
        if not usbip:
            configure(0)
            configure(1)
        coding = [struct.pack('<IBBB', 9600, 0, 0, 8), struct.pack('<IBBB', 230400, 2, 2, 7)]
        for i in range(2):
            assert control(0x21, 0x20, 0, i * 2, coding[i]) == 7
        for i in range(2):
            assert bytes(control(0xA1, 0x21, 0, i * 2, 7)) == coding[i]
            assert bytes(control(0xA1, 0x21, 0, i * 2, 64)) == coding[i]
            assert bytes(control(0xA1, 0x21, 0, i * 2, 3)) == coding[i][:3]
        print("PASS USB: independent 7-byte Line Coding, short/long reads")
        for endpoint in ([2, 3] if echo_both else [3]):
            # Read larger than one packet: a 64-byte reply must finish with a ZLP.
            dev.write(endpoint, bytes(range(64)), timeout=1000)
            assert read_exact(endpoint | 0x80, 64) == bytes(range(64))
            exchange_raw(endpoint, b'after ZLP\r\n')
        print("PASS USB: full 64-byte packets, ZLP termination, following short packet")
        if echo_both:
            # Stop consuming CDC1 IN until its DMA and software queue are full.
            dev.write(2, b'A' * 256, timeout=1000)
            time.sleep(0.1)
            dev.write(2, b'B' * 64, timeout=1000)
            time.sleep(0.1)
            exchange_raw(3, b'CDC2 while CDC1 blocked\r\n')
            assert read_exact(0x82, 320) == b'A' * 256 + b'B' * 64
            exchange_raw(2, b'CDC1 resumed\r\n')
            print("PASS USB: stopped reader on CDC1 does not block CDC2; queued data preserved")
        if usbip:
            print("SKIP USBIP: configuration changes and physical bus reset; run on direct USB")
        else:
            # Pending short OUT and unread IN must not survive configuration changes.
            dev.write(3, b'old pending data', timeout=1000)
            time.sleep(0.05)
            configure(0)
            assert bytes(control(0x80, 8, 0, 0, 1)) == b'\0'
            # PyUSB cannot claim an interface while configuration 0 is selected.
            configure(1)
            # Linux re-probes the CDC driver and writes its own initial Line Coding.
            # Verify both interfaces can be independently configured again instead
            # of assuming the firmware default survives the host driver's requests.
            for i in range(2):
                control(0x21, 0x20, 0, i * 2, coding[i])
                assert bytes(control(0xA1, 0x21, 0, i * 2, 7)) == coding[i]
            exchange_raw(3, b'fresh after configuration\r\n')
            print("PASS USB: unconfigure/reconfigure clears pending data, both ports usable")
            # Reset the physical USB device and confirm both endpoints recover.
            release_interfaces()
            dev.reset()
            time.sleep(0.5)
            # Explicitly reselect configuration; USBIP/Linux may retain the previous
            # configuration number in the host cache across a reset.
            configure(0)
            configure(1)
            for endpoint in ([2, 3] if echo_both else [3]):
                exchange_raw(endpoint, b'fresh after bus reset\r\n')
            print("PASS USB: physical bus reset and resumed communication")
        stall(0xA1, 0x21, 0, 1, 7)       # data interface
        stall(0xA1, 0x21, 0, 0x100, 7)   # high wIndex
        stall(0x21, 0x20, 0, 2, b'12345678')
        stall(0xA1, 0x20, 0, 0, 7)       # wrong direction
        for i in range(4):
            assert bytes(control(0x81, 10, 0, i, 1)) == b'\0'
        if not usbip:
            for i in range(4):
                dev.set_interface_altsetting(interface=i, alternate_setting=0)
            stall(0x01, 11, 1, 0, 0)
        print("PASS USB: invalid class requests STALL, GET_INTERFACE")
        for endpoint in (0x81, 0x02, 0x82, 0x85, 0x03, 0x83):
            control(0x02, 3, 0, endpoint)
            assert bytes(control(0x82, 0, 0, endpoint, 2)) == b'\1\0'
            # libusb also resets the host-side DATA toggle when clearing halt.
            dev.clear_halt(endpoint)
            assert bytes(control(0x82, 0, 0, endpoint, 2)) == b'\0\0'
        print("PASS USB: SET/CLEAR_FEATURE and endpoint halt status, all 6 endpoints")
        for i, endpoint in ((0, 0x81), (2, 0x85)):
            if not usbip:
                dev.set_interface_altsetting(interface=i, alternate_setting=0)
            control(0x21, 0x22, 3, i)
            notification = bytes(dev.read(endpoint, 16, timeout=1000))
            assert notification == bytes([0xA1, 0x20, 0, 0, i, 0, 2, 0, 3, 0])
        print("PASS USB: SERIAL_STATE notifications on both control interfaces")
    finally:
        for interface in reversed(claimed):
            try:
                usb.util.release_interface(dev, interface)
            except usb.core.USBError:
                pass
        try:
            if bytes(control(0x80, 8, 0, 0, 1)) == b'\0':
                dev.set_configuration(1)
        except usb.core.USBError:
            pass
        for interface in detached:
            try:
                dev.attach_kernel_driver(interface)
            except usb.core.USBError:
                pass
        usb.util.dispose_resources(dev)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--echo-both', action='store_true', help='test usb_cdc_echo firmware')
    parser.add_argument('--usb', action='store_true', help='also run raw USB control/reset tests')
    args = parser.parse_args()
    serial_checks(args.echo_both)
    if args.usb:
        usb_checks(args.echo_both)
