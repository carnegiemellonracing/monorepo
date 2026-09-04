#!/usr/bin/env python3
"""
flash_boards.py

Find the ELF firmware file for a given board inside a directory tree,
and if it is up to date with respect to its source code, invoke
FlashCli.exe's "flash-board" command to flash it.

Only standard-library modules are used.

--------------------------------------------------------------------
ASSUMPTIONS (adjust the constants below to match your repo layout):
--------------------------------------------------------------------
1. The ELF is discovered by recursively searching INPUT_DIR for a file
   named "<board_id>.elf" (case-insensitive), e.g. board_id "RAM"
   matches "RAM.elf". If several match, the most recently modified one
   is used.
2. "Up to date with source" is determined by comparing the ELF's
   last-modified time against the newest last-modified time of any
   file under INPUT_DIR that looks like source code (see
   SOURCE_EXTENSIONS / SOURCE_FILENAMES below). If nothing on disk
   newer than the ELF exists, the ELF is considered up to date.
   Adjust SOURCE_EXTENSIONS/SOURCE_FILENAMES if your source lives
   somewhere else or uses different extensions.
3. The board_id is given on the command line and must match a member
   of the BoardId enum below (case-insensitively), e.g. "RAM" or "VSM".
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


def parse_board_id(name: str) -> BoardId:
    """Match a command line board name to a BoardId enum member."""
    try:
        return BoardId[name.upper()]
    except KeyError:
        valid = ", ".join(member.name for member in BoardId)
        raise argparse.ArgumentTypeError(
            f"unknown board_id '{name}' (expected one of: {valid})"
        )


def find_elf_file(input_dir: Path, board_id: BoardId) -> Path | None:
    """
    Recursively find "<board_id>.elf" (case-insensitive) under input_dir.
    If several match, return the most recently modified one; return None
    if nothing matches.
    """
    target = f"{board_id.name.lower()}.elf"
    matches = [
        path for path in input_dir.rglob("*.elf")
        if path.is_file() and path.name.lower() == target
    ]
    if not matches:
        return None
    if len(matches) > 1:
        log.warning("Found %d candidates for '%s'; using the most recently "
                    "modified one", len(matches), target)
    return max(matches, key=lambda p: p.stat().st_mtime)


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
        description="Find the ELF for a board in a directory and flash it "
                     "using FlashCli.exe, if it is up to date with its source."
    )
    parser.add_argument(
        "input_dir",
        type=Path,
        help="Directory to scan for the ELF file (and its source).",
    )
    parser.add_argument(
        "board_id",
        type=parse_board_id,
        help="Board to flash, e.g. RAM or VSM. Selects '<board_id>.elf'.",
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
    board_id: BoardId = args.board_id
    flash_cli: Path = args.flash_cli

    if not input_dir.is_dir():
        log.error("Input directory does not exist: %s", input_dir)
        return 1
    if not flash_cli.is_file():
        log.error("FlashCli executable not found: %s", flash_cli)
        return 1

    elf_path = find_elf_file(input_dir, board_id)
    if elf_path is None:
        log.error("No '%s.elf' found under %s", board_id.name, input_dir)
        return 1

    if not is_up_to_date(elf_path, input_dir):
        log.error("%s is out of date with source", elf_path)
        return 1

    log.info("Flashing %s as board '%s' (id=%d)",
              elf_path, board_id.name, int(board_id))
    if not flash_board(flash_cli, elf_path, board_id,
                        args.device, args.channel):
        log.error("Flashing failed for %s", elf_path)
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
