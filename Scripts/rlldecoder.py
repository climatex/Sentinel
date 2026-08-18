import sys

WD_DECODE_TABLE = {
    "00001000": "0011",
    "00100100": "0010",
    "100100": "000",
    "000100": "010",
    "001000": "011",
    "1000": "11",
    "0100": "10",
}

IBM_SEAGATE_DECODE_TABLE = {
    "00001000": "0011",
    "00100100": "0010",
    "000100": "000",
    "100100": "010",
    "001000": "011",
    "1000": "11",
    "0100": "10",
}

SYNC_PATTERN = 0x8090  # 1000 0000 1001 0000
SYNC_BYTE = 0xA1

def _build_decode_lut(decode_table):
    by_len = {}
    for code_str, data_str in decode_table.items():
        L = len(code_str)
        data_len = len(data_str)
        data_val = int(data_str, 2) if data_len else 0
        by_len.setdefault(L, {})[int(code_str, 2)] = (data_len, data_val)

    lengths = sorted(by_len)
    max_len = lengths[-1]
    lut = [None] * (1 << max_len)
    for window in range(1 << max_len):
        for L in lengths:
            prefix = window >> (max_len - L)
            hit = by_len[L].get(prefix)
            if hit is not None:
                data_len, data_val = hit
                lut[window] = (L, data_val, data_len)
                break
    return lut, max_len, by_len, lengths


def decode_rll_27(input_path, output_path, mode):
    if mode.lower() == "wd":
        decode_table = WD_DECODE_TABLE
        print("Using Western Digital (WD) RLL 2,7 table.")
    elif mode.lower() in ["ibm", "seagate", "ibm/seagate"]:
        decode_table = IBM_SEAGATE_DECODE_TABLE
        print("Using IBM/Seagate RLL 2,7 table.")
    else:
        print(f"Error: Unknown mode '{mode}'. Choose 'wd' or 'ibm'.")
        return

    try:
        with open(input_path, "rb") as f:
            raw_bytes = f.read()
    except FileNotFoundError:
        print(f"Error: File '{input_path}' not found.")
        return

    lut, max_len, by_len, lengths = _build_decode_lut(decode_table)

    output_bytes = bytearray()
    n_bits = len(raw_bytes) * 8

    if n_bits == 0:
        with open(output_path, "wb") as f:
            pass
        print(f"Success! Processed 0 NRZ bytes into '{output_path}'.")
        return

    bits = bin(int.from_bytes(raw_bytes, "big"))[2:].zfill(n_bits)

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
        data_val = 0
        data_len = 0
        pos = 0
        n = len(seg)
        while pos < n:
            hit = None
            consumed = 0
            if n - pos >= max_len:
                window = int(seg[pos:pos + max_len], 2)
                hit = lut[window]
                if hit is not None:
                    consumed, dval, dlen = hit
            if hit is None:
                for L in lengths:
                    if n - pos >= L:
                        prefix_val = int(seg[pos:pos + L], 2)
                        cand = by_len[L].get(prefix_val)
                        if cand is not None:
                            consumed = L
                            dlen, dval = cand
                            break
                else:
                    consumed = 0
                if consumed == 0:
                    return
            pos += consumed
            if dlen:
                data_val = (data_val << dlen) | dval
                data_len += dlen
                while data_len >= 8:
                    data_len -= 8
                    output_bytes.append((data_val >> data_len) & 0xFF)
                data_val &= (1 << data_len) - 1

    seg_start = 0
    for start, end in zip(sync_starts, sync_ends):
        process_segment(bits[seg_start:start])
        output_bytes.append(SYNC_BYTE)
        seg_start = end + 1
    process_segment(bits[seg_start:])

    with open(output_path, "wb") as f:
        f.write(output_bytes)

    print(f"Success! Processed {len(output_bytes)} NRZ bytes into '{output_path}'.")


if __name__ == "__main__":
    if len(sys.argv) < 4:
        print("Usage: python rlldecoder.py <input_bin> <output_bin> <wd|ibm>")
    else:
        input_file_path = sys.argv[1]
        output_file_path = sys.argv[2]
        table_mode = sys.argv[3]
        decode_rll_27(input_file_path, output_file_path, table_mode)
