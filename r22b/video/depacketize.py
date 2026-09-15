"""
H.264 Depacketizer

Reads packet files produced by h264_packetize.py from tmp_packets
and reconstructs a single raw H.264 bitstream (.h264) using start codes.

Supported RTP payload formats:
- Single NAL (types 1..23)
- STAP-A (type 24)
- FU-A (type 28)

Usage:
    python3 h264_depacketize.py --input tmp_packets --output output.h264
"""

import os
import re
import glob
import argparse
from typing import Iterator, List

START_CODE = b"\x00\x00\x00\x01"

NAL_TYPE_STAP_A = 24
NAL_TYPE_FU_A = 28


def parse_packet_files(input_dir: str) -> List[str]:
    """Return packet file paths sorted by (frame_num, packet_num)."""
    files = glob.glob(os.path.join(input_dir, "frame_*_packet_*.bin"))
    def sort_key(path: str):
        name = os.path.basename(path)
        m = re.match(r"frame_(\d+)_packet_(\d+)\.bin", name)
        if m:
            return (int(m.group(1)), int(m.group(2)))
        # fallback: lexicographic
        return (name, name)
    return sorted(files, key=sort_key)


def depayload_stap_a(packet: bytes) -> Iterator[bytes]:
    """Yield NAL units from a STAP-A packet (type 24)."""
    pos = 1  # skip STAP-A header
    end = len(packet)
    while pos + 2 <= end:
        size = (packet[pos] << 8) | packet[pos + 1]
        pos += 2
        if pos + size <= end:
            yield packet[pos:pos + size]
            pos += size
        else:
            break


def depacketize(input_dir: str) -> Iterator[bytes]:
    """Yield complete NAL units reconstructed from packet files."""
    fu_buffer = None  # type: bytearray | None
    files = parse_packet_files(input_dir)

    for path in files:
        with open(path, "rb") as f:
            data = f.read()
        if not data:
            continue
        nal_type = data[0] & 0x1F
        if nal_type == NAL_TYPE_STAP_A:
            # flush any ongoing FU-A assembly
            if fu_buffer is not None:
                # incomplete sequence; skip/reset
                fu_buffer = None
            for nal in depayload_stap_a(data):
                yield nal
        elif nal_type == NAL_TYPE_FU_A:
            # FU-A: indicator + header + payload
            if len(data) < 3:
                continue
            f_nri = data[0] & 0xE0
            fu_header = data[1]
            s_bit = bool(fu_header & 0x80)
            e_bit = bool(fu_header & 0x40)
            original_type = fu_header & 0x1F
            payload = data[2:]

            if s_bit:
                # start a new buffer with original NAL header
                fu_buffer = bytearray([f_nri | original_type])
                fu_buffer.extend(payload)
            else:
                if fu_buffer is None:
                    # missing start; skip
                    continue
                fu_buffer.extend(payload)

            if e_bit and fu_buffer is not None:
                yield bytes(fu_buffer)
                fu_buffer = None
        else:
            # Single NAL: the payload is already the NAL unit
            # flush any ongoing FU-A assembly
            if fu_buffer is not None:
                fu_buffer = None
            yield data

    # if fu_buffer remains, it means an incomplete FU-A sequence; ignore


def write_h264(nals: Iterator[bytes], output_path: str) -> None:
    """Write NAL units to a raw .h264 file with start codes."""
    with open(output_path, "wb") as out:
        count = 0
        total = 0
        for nal in nals:
            if not nal:
                continue
            out.write(START_CODE)
            out.write(nal)
            count += 1
            total += len(nal)
    print(f"Wrote {count} NAL units, {total} bytes of payload to {output_path}")


def main():
    parser = argparse.ArgumentParser(description="Depacketize H.264 packets to raw bitstream")
    parser.add_argument("--input", default="tmp_packets", help="Directory containing packet .bin files")
    parser.add_argument("--output", default="output.h264", help="Output raw H.264 file path")
    args = parser.parse_args()

    if not os.path.isdir(args.input):
        raise SystemExit(f"Input directory not found: {args.input}")

    nals = depacketize(args.input)
    write_h264(nals, args.output)


if __name__ == "__main__":
    main()
