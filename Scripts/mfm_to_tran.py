import argparse
import struct
import sys

try:
    import numpy as np
    _HAVE_NUMPY = True
except ImportError:
    _HAVE_NUMPY = False

# ---------------------------------------------------------------------------
# CRC used by the mfm tools (crc_ecc.c: crc64() called with length=32).
# MSB-first, bit-serial, non-reflected CRC-32, poly 0x140a0445, init 0xffffffff.
# ---------------------------------------------------------------------------
CRC_POLY = 0x140a0445
CRC_INIT = 0xffffffff
CRC_LEN = 32
CRC_MASK = (1 << CRC_LEN) - 1


def mfm_crc32(data: bytes, crc: int = CRC_INIT) -> int:
    for byte in data:
        crc ^= byte << (CRC_LEN - 8)
        for _ in range(8):
            if crc & (1 << (CRC_LEN - 1)):
                crc = ((crc << 1) ^ CRC_POLY) & CRC_MASK
            else:
                crc = (crc << 1) & CRC_MASK
    return crc


_CRC_TABLE = [mfm_crc32(bytes([0]), crc=(i << (CRC_LEN - 8))) for i in range(256)]


def mfm_crc32_fast(data: bytes, crc: int = CRC_INIT) -> int:
    for byte in data:
        top = ((crc >> (CRC_LEN - 8)) ^ byte) & 0xff
        crc = ((crc << 8) ^ _CRC_TABLE[top]) & CRC_MASK
    return crc


# ---------------------------------------------------------------------------
# File format constants (from emu_tran_file.c)
# ---------------------------------------------------------------------------
HEADER_ID = bytes([0xee, 0x4d, 0x46, 0x4d, 0x0d, 0x0a, 0x1a, 0x00])
TRAN_FILE_VERSION = 0x01020200
SAMPLE_RATE_HZ = 200_000_000     # fixed 200MHz transition-count clock; the
                                 # only rate tran_file_read_header() accepts
TRACK_HEADER_SIZE_BYTES = 4 * 3  # cyl + head + num_bytes, fixed per track
NS_PER_TICK = 1e9 / SAMPLE_RATE_HZ

MAX_BYTE_DELTAS = 400_000  # mfm_util's own compiled-in per-track buffer
                           # (emu_tran_file.c: #define MAX_BYTE_DELTAS 400000)


def encode_deltas(deltas):
    """Byte-encode a list of tick-count deltas, mirroring the loop in
    tran_file_write_track_deltas(): values 0-253 as a single byte, 254-65535
    as marker byte 254 + 16-bit LE value, and (for completeness -- the
    reference writer's uint16_t deltas[] can never produce this case, but
    the reader supports it) values above that as marker byte 255 + 24-bit
    LE value."""
    out = bytearray()
    for d in deltas:
        if d < 0:
            raise ValueError(f"delta must be >= 0, got {d}")
        if d > 0xffff:
            out.append(255)
            out += struct.pack('<I', d)[:3]
        elif d >= 254:
            out.append(254)
            out += struct.pack('<H', d)
        else:
            out.append(d)
    return bytes(out)


def build_header(num_cyl, num_head, cmdline, note, start_time_ns):
    cmdline_b = cmdline.encode('utf-8') + b'\x00'
    note_b = (note or '').encode('utf-8') + b'\x00'

    # Matches: sizeof(expected_header_id) + 4*10 + strlen(cmdline)+1 + strlen(note)+1
    file_header_size = len(HEADER_ID) + 4 * 10 + len(cmdline_b) + len(note_b)

    body = struct.pack('<I', TRAN_FILE_VERSION)
    body += struct.pack('<I', file_header_size)
    body += struct.pack('<I', TRACK_HEADER_SIZE_BYTES)
    body += struct.pack('<I', num_cyl)
    body += struct.pack('<I', num_head)
    body += struct.pack('<I', SAMPLE_RATE_HZ)
    body += struct.pack('<I', len(cmdline_b))
    body += cmdline_b
    body += struct.pack('<I', len(note_b))
    body += note_b
    body += struct.pack('<I', start_time_ns)

    full = HEADER_ID + body
    full += struct.pack('<I', mfm_crc32_fast(full))

    assert len(full) == file_header_size, (
        f"header size mismatch: computed {file_header_size}, "
        f"actual {len(full)} -- constants above are out of sync with the "
        f"layout, don't ship this")
    return full


def build_track(cyl, head, deltas):
    encoded = encode_deltas(deltas)
    if len(encoded) > MAX_BYTE_DELTAS:
        print(f"warning: cyl {cyl} head {head} encodes to "
              f"{len(encoded):,} bytes, over mfm_util's own "
              f"{MAX_BYTE_DELTAS:,}-byte MAX_BYTE_DELTAS limit -- its C "
              f"reader will hard-exit on this track. For reference, one "
              f"physical MFM track's worth of data is normally a few tens "
              f"of thousands of bytes; this size usually means the input "
              f"covers many tracks/revolutions and needs to be split into "
              f"one track record per revolution instead of one giant one.",
              file=sys.stderr)
    rec = struct.pack('<iiI', cyl, head, len(encoded)) + encoded
    return rec + struct.pack('<I', mfm_crc32_fast(rec))


def build_eof_marker():
    rec = struct.pack('<iiI', -1, -1, 0)
    return rec + struct.pack('<I', mfm_crc32_fast(rec))


# ---------------------------------------------------------------------------
# Input parsing: binary, each byte packs 4 Ck,D pairs -- bit0=Ck1, bit1=D1,
# bit2=Ck2, bit3=D2, bit4=Ck3, bit5=D3, bit6=Ck4, bit7=D4, in MSB-first order
# ---------------------------------------------------------------------------
_BYTE_TO_BITS_MSB = [bytes((b >> (7 - i)) & 1 for i in range(8)) for b in range(256)]


def read_bits(path):
    table = _BYTE_TO_BITS_MSB
    with open(path, 'rb') as f:
        data = f.read()
    bits = bytearray()
    for byte in data:
        bits += table[byte]
    return bits


def check_mfm_validity(bits):
    count = bits.count(b'\x01\x01')
    first = bits.find(b'\x01\x01')
    return count, first


def bits_to_deltas(bits, channel_rate_hz):
    ns_per_cell = 1e9 / channel_rate_hz
    bit_time = 0.0      # fractional ns carried into the next cell's rounding
    delta_time = 0       # accumulated whole ticks since the last '1' bit
    deltas = []
    for bit in bits:
        delta = round(bit_time / NS_PER_TICK)
        delta_time += delta
        bit_time += ns_per_cell - delta * NS_PER_TICK
        if bit:
            deltas.append(delta_time)
            delta_time = 0
    return deltas


def precompute_cum(n_cells, channel_rate_hz):
    ns_per_cell = 1e9 / channel_rate_hz
    bit_time = 0.0
    cum = 0
    out = [0] * (n_cells + 1)
    for n in range(n_cells):
        delta = round(bit_time / NS_PER_TICK)
        cum += delta
        bit_time += ns_per_cell - delta * NS_PER_TICK
        out[n + 1] = cum
    return out


def bits_to_deltas_fast(track_bits, cum_arr):
    arr = np.frombuffer(bytes(track_bits), dtype=np.uint8)
    ones = np.flatnonzero(arr)
    if ones.size == 0:
        return []
    vals = cum_arr[ones + 1]
    return np.diff(vals, prepend=np.int64(0)).tolist()


def resolve_geometry(ap, args, total_tracks, track_bytes, total_input_bytes):
    if args.num_cyl is not None and args.num_head is not None:
        if args.num_cyl * args.num_head != total_tracks:
            ap.error(f"--num-cyl {args.num_cyl} x --num-head {args.num_head} "
                      f"= {args.num_cyl * args.num_head} tracks, but the "
                      f"input is {total_tracks:,} tracks of "
                      f"{track_bytes:,} bytes each ({total_input_bytes:,} "
                      f"bytes total). Fix one of them.")
        return args.num_cyl, args.num_head
    elif args.num_cyl is not None:
        if total_tracks % args.num_cyl != 0:
            ap.error(f"{total_tracks:,} tracks doesn't divide evenly by "
                      f"--num-cyl {args.num_cyl}")
        return args.num_cyl, total_tracks // args.num_cyl
    elif args.num_head is not None:
        if total_tracks % args.num_head != 0:
            ap.error(f"{total_tracks:,} tracks doesn't divide evenly by "
                      f"--num-head {args.num_head}")
        return total_tracks // args.num_head, args.num_head
    else:
        ap.error(f"--track-bytes given but neither --num-cyl nor "
                  f"--num-head specified. Input is {total_tracks:,} tracks "
                  f"of {track_bytes:,} bytes each ({total_input_bytes:,} "
                  f"bytes total) -- say how to factor that into "
                  f"cylinders x heads.")


def report_track(cyl, head, deltas):
    if deltas:
        extra = ""
        if max(deltas) > 0xffff:
            extra = " (some >65535, using 3-byte encoding -- check rate)"
        print(f"  cyl={cyl} head={head}: {len(deltas)} transitions, range "
              f"{min(deltas)}-{max(deltas)} ticks{extra}", file=sys.stderr)
    else:
        print(f"  cyl={cyl} head={head}: warning, zero transitions",
              file=sys.stderr)


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('input', help='Binary file, 4 Ck,D pairs per byte')
    ap.add_argument('output', help='Transition file to write')
    ap.add_argument('--data-rate', type=float, default=5_000_000,
                     help='Decoded MFM data rate in bits/sec (default '
                          '5,000,000 -- standard 5Mbps ST506 MFM). The '
                          'channel/cell rate used for timing is 2x this, '
                          'since MFM has one clock bit-cell per data '
                          'bit-cell. Ignored if --channel-rate is given.')
    ap.add_argument('--channel-rate', type=float, default=None,
                     help='Override: encoded bit-cell rate in Hz directly '
                          '(bypasses the 2x-data-rate MFM assumption).')
    ap.add_argument('--cyl', type=int, default=0,
                     help='Single-track mode only (no --track-bytes): the '
                          'cylinder this file represents.')
    ap.add_argument('--head', type=int, default=0,
                     help='Single-track mode only (no --track-bytes): the '
                          'head this file represents.')
    ap.add_argument('--num-cyl', type=int, default=None,
                     help='Number of cylinders on the disk. Single-track '
                          'mode: defaults to --cyl + 1. Slicing mode: give '
                          'this, --num-head, or both -- the missing one is '
                          'derived from the input file size.')
    ap.add_argument('--num-head', type=int, default=None,
                     help='Number of heads on the disk. Same rules as '
                          '--num-cyl.')
    ap.add_argument('--track-bytes', type=int, default=None,
                     help='Fixed size in bytes of one track in the input '
                          'file. When given, the input is treated as a '
                          'whole-disk capture and sliced into one track '
                          'record per --track-bytes chunk, walked across '
                          '--num-cyl x --num-head tracks in --order. When '
                          'omitted (default), the whole input file is '
                          'written as a single track at --cyl/--head.')
    ap.add_argument('--order', choices=['cyl-major', 'head-major'],
                     default='cyl-major',
                     help="Slicing mode only: how consecutive --track-bytes "
                          "chunks map to (cyl, head). 'cyl-major' (default) "
                          "walks all heads at cyl 0, then all heads at cyl "
                          "1, etc -- matches emu_file_seek_track()'s layout "
                          "in this same codebase, and how a real controller "
                          "normally captures (head switches are electronic, "
                          "cylinder seeks aren't, so captures naturally "
                          "cycle heads before seeking). This is an "
                          "assumption, not confirmed against your capture "
                          "setup -- if tracks land at the wrong cyl/head, "
                          "try 'head-major' first.")
    ap.add_argument('--start-time-ns', type=int, default=0,
                     help='Start time of data from index, in nanoseconds')
    ap.add_argument('--note', default='')
    ap.add_argument('--cmdline', default=None,
                     help="Stored in the file header's command-line field; "
                          "defaults to this script's own invocation")
    args = ap.parse_args()

    cmdline = args.cmdline if args.cmdline is not None else ' '.join(sys.argv)
    channel_rate_hz = args.channel_rate if args.channel_rate else 2 * args.data_rate
    print(f"channel rate {channel_rate_hz/1e6:.4f} MHz "
          f"({SAMPLE_RATE_HZ/channel_rate_hz:.4f} ticks/cell @200MHz)",
          file=sys.stderr)

    bits = read_bits(args.input)
    total_input_bytes = len(bits) // 8
    print(f"read {total_input_bytes:,} bytes ({len(bits):,} bits) from "
          f"{args.input}", file=sys.stderr)

    if args.track_bytes is None:
        # ---- single-track mode ----
        num_cyl = args.num_cyl if args.num_cyl is not None else args.cyl + 1
        num_head = args.num_head if args.num_head is not None else args.head + 1

        deltas = bits_to_deltas(bits, channel_rate_hz)
        report_track(args.cyl, args.head, deltas)

        with open(args.output, 'wb') as f:
            f.write(build_header(num_cyl, num_head, cmdline, args.note,
                                  args.start_time_ns))
            f.write(build_track(args.cyl, args.head, deltas))
            f.write(build_eof_marker())

    else:
        # ---- whole-disk slicing mode ----
        track_bytes = args.track_bytes
        if total_input_bytes % track_bytes != 0:
            leftover = total_input_bytes % track_bytes
            print(f"warning: input is {total_input_bytes:,} bytes, not a "
                  f"whole multiple of --track-bytes {track_bytes:,} "
                  f"({leftover:,} leftover bytes at the end ignored)",
                  file=sys.stderr)
        total_tracks = total_input_bytes // track_bytes

        num_cyl, num_head = resolve_geometry(ap, args, total_tracks,
                                              track_bytes, total_input_bytes)

        print(f"slicing into {num_cyl} cyl x {num_head} head = "
              f"{num_cyl * num_head} tracks of {track_bytes:,} bytes "
              f"({track_bytes * 8:,} bits) each, {args.order} order",
              file=sys.stderr)

        if args.order == 'cyl-major':
            pairs = [(c, h) for c in range(num_cyl) for h in range(num_head)]
        else:
            pairs = [(c, h) for h in range(num_head) for c in range(num_cyl)]

        track_bits_len = track_bytes * 8
        if _HAVE_NUMPY:
            print("numpy available -- using the precomputed-CUM fast path",
                  file=sys.stderr)
            cum_arr = np.asarray(precompute_cum(track_bits_len, channel_rate_hz),
                                  dtype=np.int64)
        else:
            print("numpy not available -- falling back to the plain-Python "
                  "per-bit loop, which will be considerably slower on a "
                  "whole-disk capture (pip install numpy to speed this up)",
                  file=sys.stderr)

        with open(args.output, 'wb') as f:
            f.write(build_header(num_cyl, num_head, cmdline, args.note,
                                  args.start_time_ns))
            for idx, (cyl, head) in enumerate(pairs):
                start = idx * track_bits_len
                track_bits = bits[start:start + track_bits_len]
                if _HAVE_NUMPY:
                    deltas = bits_to_deltas_fast(track_bits, cum_arr)
                else:
                    deltas = bits_to_deltas(track_bits, channel_rate_hz)
                f.write(build_track(cyl, head, deltas))
                if not deltas:
                    print(f"  warning: cyl={cyl} head={head} has zero "
                          f"transitions", file=sys.stderr)
                if (idx + 1) % 25 == 0 or idx + 1 == len(pairs):
                    print(f"  {idx + 1}/{len(pairs)} tracks written "
                          f"(last: cyl={cyl} head={head}, {len(deltas)} "
                          f"transitions)", file=sys.stderr)
            f.write(build_eof_marker())

    print(f"wrote {args.output}", file=sys.stderr)


if __name__ == '__main__':
    main()
