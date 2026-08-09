#!/usr/bin/env python3
"""
flash_boards.py

Scan a directory tree for ELF firmware files, and for any ELF that is
up to date with respect to its source code, invoke FlashCli.exe's
"flash-board" command to flash it.

Only standard-library modules are used.

--------------------------------------------------------------------
ASSUMPTIONS (adjust the constants below to match your repo layout):
--------------------------------------------------------------------
1. ELF files are discovered by recursively searching INPUT_DIR for
   files ending in ".elf".
2. "Up to date with source" is determined by comparing the ELF's
   last-modified time against the newest last-modified time of any
   file under INPUT_DIR that looks like source code (see
   SOURCE_EXTENSIONS / SOURCE_FILENAMES below). If nothing on disk
   newer than the ELF exists, the ELF is considered up to date.
   Adjust SOURCE_EXTENSIONS/SOURCE_FILENAMES if your source lives
   somewhere else or uses different extensions.
3. The board_id to flash is derived from the ELF's file name (stem),
   matched case-insensitively against the BoardId enum below, e.g.
   "RAM.elf" -> BoardId.RAM.
--------------------------------------------------------------------
"""

import argparse
import enum
import logging
import subprocess
import sys
from pathlib import Path

# ---------------------------------------------------------------------------
# Hard-coded board_id enum. Extend this as new boards are added.
# ---------------------------------------------------------------------------
class BoardId(enum.IntEnum):
    RAM = 1
    VSM = 2


# ---------------------------------------------------------------------------
# Which files count as "source" when checking whether an ELF is stale.
# ---------------------------------------------------------------------------
SOURCE_EXTENSIONS = {
    ".c", ".h", ".cpp", ".hpp", ".cc", ".cxx", ".s", ".S", ".ld", ".cmake",
}
SOURCE_FILENAMES = {
    "CMakeLists.txt",
    "Makefile",
}

logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
log = logging.getLogger(__name__)


def find_elf_files(input_dir: Path) -> list[Path]:
    """Recursively find all .elf files under input_dir."""
    return sorted(input_dir.rglob("*.elf"))


def newest_source_mtime(input_dir: Path, exclude: Path) -> float:
    """
    Return the newest mtime among all files under input_dir that look
    like source files, ignoring the ELF file itself. Returns 0.0 if no
    source files are found.
    """
    newest = 0.0
    for path in input_dir.rglob("*"):
        if not path.is_file() or path == exclude:
            continue
        if path.suffix in SOURCE_EXTENSIONS or path.name in SOURCE_FILENAMES:
            mtime = path.stat().st_mtime
            if mtime > newest:
                newest = mtime
    return newest


def is_up_to_date(elf_path: Path, input_dir: Path) -> bool:
    """An ELF is up to date if no source file is newer than it."""
    elf_mtime = elf_path.stat().st_mtime
    src_mtime = newest_source_mtime(input_dir, exclude=elf_path)
    return elf_mtime >= src_mtime


def resolve_board_id(elf_path: Path) -> BoardId | None:
    """Match the ELF's file name (stem) to a BoardId enum member."""
    name = elf_path.stem.upper()
    try:
        return BoardId[name]
    except KeyError:
        return None


def flash_board(flash_cli: Path, elf_path: Path, board_id: BoardId,
                 device: str, channel: int) -> bool:
    """Invoke FlashCli.exe flash-board with the given parameters."""
    cmd = [
        str(flash_cli),
        "flash-board",
        "--file", str(elf_path),
        "--device", device,
        "--board_id", str(int(board_id)),
        "--channel", str(channel),
    ]
    log.info("Running: %s", " ".join(cmd))
    result = subprocess.run(cmd)
    return result.returncode == 0


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Scan a directory for ELF files and flash any that are "
                     "up to date with their source using FlashCli.exe."
    )
    parser.add_argument(
        "input_dir",
        type=Path,
        help="Directory to scan for ELF files (and their source).",
    )
    parser.add_argument(
        "--flash-cli",
        type=Path,
        required=True,
        help="Path to FlashCli.exe.",
    )
    # Mirrors FlashCli's -d/--device option.
    parser.add_argument(
        "-d", "--device",
        default="peak_pcanusb",
        help="Name of the CAN device to use, e.g. peak_pcanusb or can0 "
             "(Default = peak_pcanusb).",
    )
    # Mirrors FlashCli's -c/--channel option.
    parser.add_argument(
        "-c", "--channel",
        type=int,
        default=0,
        help="Zero based index of the CAN channel to use, for adapters "
             "with multiple channels (Default = 0).",
    )
    args = parser.parse_args()

    input_dir: Path = args.input_dir
    flash_cli: Path = args.flash_cli

    if not input_dir.is_dir():
        log.error("Input directory does not exist: %s", input_dir)
        return 1
    if not flash_cli.is_file():
        log.error("FlashCli executable not found: %s", flash_cli)
        return 1

    elf_files = find_elf_files(input_dir)
    if not elf_files:
        log.warning("No .elf files found under %s", input_dir)
        return 0

    print(elf_files)
    exit_code = 0
    for elf_path in elf_files:
        board_id = resolve_board_id(elf_path)
        if board_id is None:
            log.warning("Skipping %s: no matching BoardId enum member for "
                        "name '%s'", elf_path, elf_path.stem)
            continue

        if not is_up_to_date(elf_path, input_dir):
            log.info("Skipping %s: out of date with source", elf_path)
            continue

        log.info("Flashing %s as board '%s' (id=%d)",
                  elf_path, board_id.name, int(board_id))
        success = flash_board(flash_cli, elf_path, board_id,
                               args.device, args.channel)
        if not success:
            log.error("Flashing failed for %s", elf_path)
            exit_code = 1

    return exit_code


if __name__ == "__main__":
    sys.exit(main())