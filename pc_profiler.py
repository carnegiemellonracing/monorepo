#!/usr/bin/env python3
"""
pc_sample_stats.py

Parse raw ITM/DWT trace data (e.g. from OpenOCD's itm.fifo) and compute
simple statistics on where PC samples land.

Usage:
    python pc_sample_stats.py itm.fifo
    python pc_sample_stats.py itm.fifo --elf firmware.elf --toolchain-prefix arm-none-eabi-
    python pc_sample_stats.py itm.fifo --top 20 --bucket 16
    python pc_sample_stats.py itm.fifo --elf firmware.elf --group-by-function

Notes:
    - This is a minimal parser targeting the specific packet type you're
      generating: DWT PC-sample hardware packets, header byte 0x17,
      4-byte little-endian PC payload. It does not implement the full
      ITM/DWT protocol (sync packets, timestamps, overflow, other
      hardware source packets, etc). If your stream has those mixed in,
      this script will likely desync on them -- see comments below for
      where to extend it.
    - If unexpected/unknown header bytes are encountered, the script
      just advances one byte and tries again (resync), so a corrupted
      or mixed-content stream will still produce output for the parts
      it can read, along with a count of skipped bytes.
"""

import argparse
import subprocess
import sys
from collections import Counter


PC_SAMPLE_HEADER = 0x17
PC_SAMPLE_LEN = 4  # bytes of payload after the header


def parse_pc_samples(data: bytes):
    """Scan a byte stream for 0x17 <4-byte little-endian PC> packets.

    Returns (list_of_pcs, skipped_byte_count).
    """
    pcs = []
    skipped = 0
    i = 0
    n = len(data)
    while i < n:
        if data[i] == PC_SAMPLE_HEADER and i + 1 + PC_SAMPLE_LEN <= n:
            pc = int.from_bytes(data[i + 1 : i + 1 + PC_SAMPLE_LEN], "little")
            pcs.append(pc)
            i += 1 + PC_SAMPLE_LEN
        else:
            skipped += 1
            i += 1
    return pcs, skipped


def addr2line_lookup(pc: int, elf_path: str, addr2line: str):
    """Run addr2line once for a PC. Returns (func_name, file_line) or ('??', '??:?') on failure."""
    try:
        out = subprocess.run(
            [addr2line, "-f", "-C", "-e", elf_path, f"0x{pc:08x}"],
            capture_output=True,
            text=True,
            check=True,
        ).stdout.strip().splitlines()
        func = out[0] if len(out) > 0 else "??"
        loc = out[1] if len(out) > 1 else "??:?"
        return func, loc
    except Exception:
        return "??", "?? (addr2line failed)"


def resolve_symbol(pc: int, elf_path: str, addr2line: str) -> str:
    """Resolve a PC to 'function (file:line)' using addr2line."""
    func, loc = addr2line_lookup(pc, elf_path, addr2line)
    return f"{func} ({loc})"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("fifo", help="Path to itm.fifo or a captured binary file of ITM data")
    ap.add_argument("--top", type=int, default=15, help="Show top N most-sampled addresses/buckets (default 15)")
    ap.add_argument(
        "--bucket",
        type=int,
        default=1,
        help="Group addresses into buckets of this many bytes before counting "
        "(e.g. 16 groups nearby instructions together; default 1 = exact address)",
    )
    ap.add_argument("--elf", help="Optional ELF file to resolve addresses to function/file/line via addr2line")
    ap.add_argument(
        "--toolchain-prefix",
        default="arm-none-eabi-",
        help="Prefix for addr2line binary (default: arm-none-eabi-)",
    )
    ap.add_argument(
        "--group-by-function",
        action="store_true",
        help="Aggregate samples by containing function (via addr2line) instead of by address/bucket. "
        "Requires --elf. Ignores --bucket.",
    )
    args = ap.parse_args()

    if args.group_by_function and not args.elf:
        print("Error: --group-by-function requires --elf (function boundaries come from the symbol table).", file=sys.stderr)
        sys.exit(1)

    try:
        with open(args.fifo, "rb") as f:
            data = f.read()
    except OSError as e:
        print(f"Error reading {args.fifo}: {e}", file=sys.stderr)
        sys.exit(1)

    pcs, skipped = parse_pc_samples(data)

    if not pcs:
        print("No PC-sample packets found. Is this really raw ITM/DWT data?")
        sys.exit(1)

    total = len(pcs)

    print(f"File:            {args.fifo}")
    print(f"Bytes read:      {len(data)}")
    print(f"PC samples:      {total}")
    print(f"Bytes skipped:   {skipped} (non-PC-sample bytes; unparsed headers/other packets)")

    addr2line = args.toolchain_prefix + "addr2line"

    if args.group_by_function:
        # Resolve each unique PC once, then group counts by function name.
        unique_addrs = set(pcs)
        addr_to_func = {}
        for addr in unique_addrs:
            func, _loc = addr2line_lookup(addr, args.elf, addr2line)
            addr_to_func[addr] = func

        func_counts = Counter(addr_to_func[pc] for pc in pcs)

        print(f"Unique funcs:    {len(func_counts)}")
        print()
        print(f"Top {min(args.top, len(func_counts))} hottest functions:")
        print(f"{'Function':<40} {'Count':>7} {'%':>6}")
        print("-" * 60)
        for func, count in func_counts.most_common(args.top):
            pct = 100.0 * count / total
            print(f"{func:<40} {count:>7} {pct:5.1f}%")
    else:
        bucketed = Counter((pc // args.bucket) * args.bucket for pc in pcs)
        symbol_cache = {}

        print(f"Unique addrs:    {len(bucketed)} (bucket size = {args.bucket})")
        print()
        print(f"Top {min(args.top, len(bucketed))} hottest locations:")
        print(f"{'Address':<12} {'Count':>7} {'%':>6}   Symbol")
        print("-" * 70)
        for addr, count in bucketed.most_common(args.top):
            pct = 100.0 * count / total
            symbol = ""
            if args.elf:
                if addr not in symbol_cache:
                    symbol_cache[addr] = resolve_symbol(addr, args.elf, addr2line)
                symbol = symbol_cache[addr]
            print(f"0x{addr:08x}  {count:>7} {pct:5.1f}%   {symbol}")


if __name__ == "__main__":
    main()
