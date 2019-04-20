#!/usr/bin/env python3

# Paste RAM_DL text from EVE Screen Editor.
# (It can be found in the 'Inspector' tab at the bottom left)
# Ensure the pasted text has line breaks after each command.
# Ensure that the last line is a DISPLAY() cmd; preferably the first one.
# Press CTRL-D to indicate end-of-file.

import sys

# Write display list start command.
sys.stdout.write("0xffffff00,\n")

# Parse input.
for line in sys.stdin:
    i = line.find("0x")
    if (i < 0):
        continue

    # Print display list value.
    sys.stdout.write(line[i:i + 10] + ",\n")

# Write display list end command.
sys.stdout.write('0xffffff01,')

