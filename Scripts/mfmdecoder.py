# Synchronizes an MFM bitstream to each occurence of 0xA1 with clock bit 2 dropped.

import sys

SYNC_PATTERN = 0x4489
SYNC_BYTE = 0xA1


def _build_nrz_extract_table():
    table = [0] * 65536
    for word in range(65536):
        nrz = 0
        for i in range(8):
            data_bit = (word >> (14 - 2 * i)) & 1
            nrz = (nrz << 1) | data_bit
        table[word] = nrz
    return table


NRZ_EXTRACT_TABLE = _build_nrz_extract_table()


def decode_mfm_to_nrz(input_file_path, output_file_path):
    try:
        with open(input_file_path, "rb") as f:
            mfm_bytes = f.read()
    except FileNotFoundError:
        print(f"Error: File '{input_file_path}' not found.")
        return

    output_bytes = bytearray()
    n_bits = len(mfm_bytes) * 8

    if n_bits == 0:
        with open(output_file_path, "wb") as f:
            pass
        print(f"Decoding complete. Saved 0 bytes to '{output_file_path}'.")
        return

    bits = bin(int.from_bytes(mfm_bytes, "big"))[2:].zfill(n_bits)

    sync_str = format(SYNC_PATTERN, "016b")
    sync_starts = []
    sync_ends = []
    search_from = 0
    while True:
        idx = bits.find(sync_str, search_from)
        if idx == -1:
            break
        sync_starts.append(idx)
        sync_ends.append(idx + 15)
        search_from = idx + 1

    def process_segment(seg):
        n = len(seg)
        complete = (n // 16) * 16
        for i in range(0, complete, 16):
            output_bytes.append(NRZ_EXTRACT_TABLE[int(seg[i:i + 16], 2)])

    seg_start = 0
    for start, end in zip(sync_starts, sync_ends):
        process_segment(bits[seg_start:start])
        output_bytes.append(SYNC_BYTE)
        seg_start = end + 1
    process_segment(bits[seg_start:])

    with open(output_file_path, "wb") as f:
        f.write(output_bytes)

    print(f"Decoding complete. Saved {len(output_bytes)} bytes to '{output_file_path}'.")


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python mfm_decoder.py <input_mfm_bin> <output_nrz_bin>")
    else:
        decode_mfm_to_nrz(sys.argv[1], sys.argv[2])
