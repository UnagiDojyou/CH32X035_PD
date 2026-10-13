#!/usr/bin/env python3
"""Decode/check CDC2 PD records with a one-byte SOP prefix (SOP in bit7..6; bit5..0 zero).

Hardware test: python3 test/test_pd_cdc.py --seconds 30 --output /tmp/pd-capture.bin
Decoder test only: python3 test/test_pd_cdc.py --self-test
CRC validates record boundaries; no length field or USB packet framing is assumed.
"""
import argparse
from collections import Counter
from pathlib import Path
import time
import subprocess
import zlib


class Decoder:
    def __init__(self):
        self.buffer = bytearray()
        self.skipped = 0

    def feed(self, data):
        self.buffer.extend(data)
        records = []
        while len(self.buffer) >= 7:
            for offset in range(len(self.buffer) - 6):
                wire = self.buffer[offset:offset + 265]
                sop = wire[0] >> 6
                if not sop or wire[0] & 63:
                    continue
                decoded = bytes(wire[1:])
                header = int.from_bytes(decoded[:2], 'little')
                length = 6 + 4 * ((header >> 12) & 7)  # Header + objects + CRC
                if header & 0x8000:
                    ext = int.from_bytes(decoded[2:4], 'little')
                    if not ext & 0x8000:  # Unchunked Extended Message
                        length = 8 + (ext & 0x1ff)
                if length > 264 or len(wire) < length + 1:
                    continue
                raw = decoded[:length]
                crc = int.from_bytes(raw[-4:], 'little')
                if zlib.crc32(raw[:-4]) != crc:
                    continue
                self.skipped += offset
                records.append((sop, raw))
                del self.buffer[:offset + length + 1]
                break
            else:
                # Keep enough for a fragmented maximum-sized record.
                extra = max(0, len(self.buffer) - 264)
                self.skipped += extra
                del self.buffer[:extra]
                break
        return records


def self_test():
    records = []
    for sop in (1, 2, 3):
        for objects in range(8):
            header = (objects << 12) | 0x81
            payload = header.to_bytes(2, 'little') + bytes(range(objects * 4))
            raw = payload + zlib.crc32(payload).to_bytes(4, 'little')
            wire = bytes([sop << 6]) + raw
            records.append((wire, (sop, raw)))
    decoder = Decoder()
    actual = []
    stream = b''.join(wire for wire, _ in records)
    for byte in stream:
        actual.extend(decoder.feed(bytes([byte])))
    assert actual == [record for _, record in records]
    assert not decoder.buffer and not decoder.skipped
    corrupted = bytearray(records[0][0])
    corrupted[2] ^= 1
    decoder = Decoder()
    assert decoder.feed(corrupted + records[1][0]) == [records[1][1]]
    assert decoder.skipped == len(corrupted)
    bad_prefix = bytearray(records[0][0])
    bad_prefix[0] |= 1
    decoder = Decoder()
    assert decoder.feed(bad_prefix + records[1][0]) == [records[1][1]]
    assert decoder.skipped == len(bad_prefix)
    print('PASS decoder: aligned SOP prefix, reserved bits, fragmented/concatenated records, CRC and resynchronization')


def hardware(args):
    import serial
    decoder = Decoder()
    counts = Counter()
    capture = bytearray()
    log = bytearray()
    # Include all bytes pending when opened: these may be the negotiation log.
    with serial.Serial(args.cdc1, 115200, timeout=0, write_timeout=2) as first, \
         serial.Serial(args.cdc2, 115200, timeout=0, write_timeout=2) as second:
        print('CDC1/CDC2 open; capturing PD packets', flush=True)
        probe = b'CDC2_ECHO_MUST_BE_DISABLED\n'
        second.write(probe)
        second.flush()
        restart = subprocess.Popen(['bash', args.source_reset, 'restart']) if args.source_reset else None
        deadline = time.monotonic() + args.seconds
        while time.monotonic() < deadline:
            if restart and restart.poll() not in (None, 0):
                raise RuntimeError('PD source reset failed')
            log.extend(first.read(4096))
            data = second.read(4096)
            capture.extend(data)
            for sop, raw in decoder.feed(data):
                header = int.from_bytes(raw[:2], 'little')
                kind = 'extended' if header & 0x8000 else ('data' if header & 0x7000 else 'control')
                counts[(sop, kind, header & 31)] += 1
                print(f'SOP={sop} {kind} type={header & 31} length={len(raw)} CRC=OK {raw.hex()}', flush=True)
            if (args.packets and sum(counts.values()) >= args.packets and not decoder.buffer
                    and (not restart or restart.poll() == 0)):
                break
            time.sleep(0.005)
        if restart:
            assert restart.wait(timeout=20) == 0, 'PD source reset failed'
    if args.output:
        Path(args.output).write_bytes(capture)
        Path(args.output + '.cdc1.txt').write_bytes(log)
    print('CDC1:', log.decode(errors='replace'), flush=True)
    print(f'{len(capture)} bytes, {sum(counts.values())} valid packets, '
          f'{decoder.skipped} skipped bytes, {len(decoder.buffer)} pending bytes', flush=True)
    print('Counts:', dict(counts), flush=True)
    assert probe not in capture, 'CDC2 still echoes OUT data'
    assert counts, 'No CRC-valid PD records received (connect a PD source)'
    assert not decoder.skipped and not decoder.buffer, 'Unexpected/truncated CDC2 record'
    print('PASS hardware: CDC2 PD packets, CRC/padding, no echo, CDC1 remains readable')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--self-test', action='store_true')
    parser.add_argument('--cdc1', default='/dev/ttyACM0')
    parser.add_argument('--cdc2', default='/dev/ttyACM1')
    parser.add_argument('--seconds', type=float, default=30)
    parser.add_argument('--packets', type=int, default=0, help='finish after at least this many complete packets')
    parser.add_argument('--output')
    parser.add_argument('--source-reset', help='run this source_reset.sh restart after both CDC ports open')
    args = parser.parse_args()
    if args.self_test:
        self_test()
    else:
        hardware(args)
