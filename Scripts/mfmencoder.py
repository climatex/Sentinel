# Creates an MFM bitstream from NRZ data. Drops clock bit 2 at each sync_offset.

import sys
import argparse


def _build_mfm_encode_table():
    table = [[0] * 256 for _ in range(2)]
    for byte in range(256):
        for prev in range(2):
            word = 0
            prev_bit = prev
            for bit_idx in range(7, -1, -1):
                data_bit = (byte >> bit_idx) & 1
                clock_bit = 1 if (prev_bit == 0 and data_bit == 0) else 0
                word = (word << 2) | (clock_bit << 1) | data_bit
                prev_bit = data_bit
            table[prev][byte] = word
    return table


MFM_ENCODE_TABLE = _build_mfm_encode_table()

SYNC_VIOLATION_WORD = 0b0100010010001001


def encode_nrz_to_mfm(input_path, output_path, sync_offsets):
    try:
        with open(input_path, "rb") as f:
            nrz_bytes = f.read()
    except FileNotFoundError:
        print(f"Error: Input file '{input_path}' not found.")
        return

    out = bytearray(2 * len(nrz_bytes))
    prev_data_bit = 0

    for byte_idx, current_byte in enumerate(nrz_bytes):
        if byte_idx in sync_offsets:
            if current_byte != 0xA1:
                print(f"Note: dropping Ck2 bit at sync offset {byte_idx} which is not A1 (0x{current_byte:02X})")
            word = SYNC_VIOLATION_WORD
            prev_data_bit = 1
        else:
            word = MFM_ENCODE_TABLE[prev_data_bit][current_byte]
            prev_data_bit = current_byte & 1

        out[2 * byte_idx] = word >> 8
        out[2 * byte_idx + 1] = word & 0xFF

    with open(output_path, "wb") as f:
        f.write(out)

    print(f"Success! Generated MFM stream ({len(out)} bytes) saved to '{output_path}'.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Standard MFM Pulse Encoder with Address Mark Clock Dropping.")
    parser.add_argument("input", help="Path to raw NRZ binary input file")
    parser.add_argument("output", help="Path to write output MFM binary bitstream")
    parser.add_argument("-s", "--sync-offsets", default="", help="Comma-separated byte offsets where 0xA1 is an address sync mark (e.g. 0,1,2,512)")

    args = parser.parse_args()

    offsets_set = set()
    if args.sync_offsets:
        try:
            offsets_set = {int(x.strip()) for x in args.sync_offsets.split(",")}
        except ValueError:
            print("Error: Sync offsets must be valid, comma-separated integers.")
            sys.exit(1)

    encode_nrz_to_mfm(args.input, args.output, offsets_set)
