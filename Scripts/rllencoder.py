import sys
import argparse

WD_ENCODE_TABLE = [
    ("0011", "00001000"),
    ("0010", "00100100"),
    ("000",  "100100"),
    ("010",  "000100"),
    ("011",  "001000"),
    ("11",   "1000"),
    ("10",   "0100"),
]

IBM_SEAGATE_ENCODE_TABLE = [
    ("0011", "00001000"),
    ("0010", "00100100"),
    ("000",  "000100"),
    ("010",  "100100"),
    ("011",  "001000"),
    ("11",   "1000"),
    ("10",   "0100"),
]

SYNC_PATTERN_BITS = "1000000010010000"
SYNC_PATTERN_VAL = int(SYNC_PATTERN_BITS, 2)
SYNC_PATTERN_LEN = len(SYNC_PATTERN_BITS)


def _build_encode_lut(encode_table):
    lut = [None] * 16
    for v in range(16):
        s = format(v, "04b")
        for data_str, code_str in encode_table:
            if s.startswith(data_str):
                lut[v] = (len(data_str), int(code_str, 2), len(code_str))
                break
    return lut


class _BitWriter:
    __slots__ = ("out", "val", "nbits")

    def __init__(self):
        self.out = bytearray()
        self.val = 0
        self.nbits = 0

    def write(self, value, nbits):
        self.val = (self.val << nbits) | (value & ((1 << nbits) - 1))
        self.nbits += nbits
        while self.nbits >= 8:
            self.nbits -= 8
            self.out.append((self.val >> self.nbits) & 0xFF)
        self.val &= (1 << self.nbits) - 1

    def flush(self):
        if self.nbits:
            self.out.append((self.val << (8 - self.nbits)) & 0xFF)
            self.val = 0
            self.nbits = 0


def encode_nrz_to_rll27(input_path, output_path, mode, sync_offsets):
    if mode.lower() == "wd":
        encode_table = WD_ENCODE_TABLE
        print("Encoding using Western Digital (WD) RLL 2,7 table.")
    elif mode.lower() == "ibm":
        encode_table = IBM_SEAGATE_ENCODE_TABLE
        print("Encoding using IBM/Seagate RLL 2,7 table.")
    else:
        print(f"Error: Unknown mode '{mode}'. Choose 'wd' or 'ibm'.")
        return

    try:
        with open(input_path, "rb") as f:
            nrz_bytes = f.read()
    except FileNotFoundError:
        print(f"Error: Input file '{input_path}' not found.")
        return

    lut = _build_encode_lut(encode_table)
    total_bytes = len(nrz_bytes)
    total_bits = total_bytes * 8
    writer = _BitWriter()

    def get_bit(pos):
        if pos >= total_bits:
            return 0
        return (nrz_bytes[pos >> 3] >> (7 - (pos & 7))) & 1

    def peek4(pos):
        byte_idx = pos >> 3
        if byte_idx >= total_bytes:
            return 0
        b0 = nrz_bytes[byte_idx]
        b1 = nrz_bytes[byte_idx + 1] if byte_idx + 1 < total_bytes else 0
        return ((b0 << 8) | b1) >> (12 - (pos & 7)) & 0xF

    sync_bit_offsets = sorted(o * 8 for o in sync_offsets)
    next_sync_idx = 0
    next_sync = sync_bit_offsets[0] if sync_bit_offsets else None

    pos = 0
    failed = False
    while pos < total_bits:
        if next_sync is not None and pos >= next_sync:
            next_sync_idx += 1
            next_sync = (
                sync_bit_offsets[next_sync_idx]
                if next_sync_idx < len(sync_bit_offsets)
                else None
            )
            writer.write(SYNC_PATTERN_VAL, SYNC_PATTERN_LEN)

        window = peek4(pos)
        consumed, code_val, code_len = lut[window]

        if next_sync is None or pos + consumed <= next_sync:
            writer.write(code_val, code_len)
            pos += consumed
            continue

        is_zero_preamble = next_sync is not None
        p = pos
        while is_zero_preamble and p < next_sync:
            if get_bit(p):
                is_zero_preamble = False
            p += 1

        if is_zero_preamble:
            pos = next_sync
            continue

        print(
            f"Error: RLL encoding failed at bit {pos}: non-zero data between "
            f"here and the pending sync mark at bit {next_sync}. Sync marks "
            f"must be preceded by a zero preamble."
        )
        failed = True
        break

    writer.flush()

    with open(output_path, "wb") as f:
        f.write(writer.out)

    if failed:
        print(f"Wrote {len(writer.out)} bytes (partial output up to the encoding failure) to '{output_path}'.")
    else:
        print(f"Success! Generated RLL stream ({len(writer.out)} bytes) with active hardware sync injections.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Variable-Width RLL (2,7) Encoder with Precise Offset Sync Control.")
    parser.add_argument("input", help="Path to input NRZ binary file")
    parser.add_argument("output", help="Path to write output RLL binary bitstream")
    parser.add_argument("mode", choices=["wd", "ibm"], help="Select table scheme: 'wd' or 'ibm'")
    parser.add_argument("-s", "--sync-offsets", default="", help="Comma-separated byte offsets before which the RLL sync mark is inserted; each must be preceded by an all-zero preamble (e.g. 0,12,512)")

    args = parser.parse_args()

    offsets_set = set()
    if args.sync_offsets:
        try:
            offsets_set = {int(x.strip()) for x in args.sync_offsets.split(",")}
        except ValueError:
            print("Error: Sync offsets must be an integer or comma-separated list of integers.")
            sys.exit(1)

    encode_nrz_to_rll27(args.input, args.output, args.mode, offsets_set)
