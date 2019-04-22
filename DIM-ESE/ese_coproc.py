#!/usr/bin/env python3
#
# ese_coproc.py
# Conversion from ESE project files to coprocessor command lists.
#
# Author: Carnegie Mellon Racing

import sys
import json
import re

#
# FT81x commands.
#
# Chapter, section, and page numbers are relative to the FT81X Series
# Programmer's Guide:
#
# https://brtchip.com/wp-content/uploads/Support/Documentation/Programming_Guides/ICs/EVE/FT81X_Series_Programmer_Guide.pdf
#
# Each command returns an array of 32-bit values, representing the encoded
# command as specified in the Programmer's Guide.
#
# Please sort the command implementations in the same order as the guide.
#
class FT81x(object):
    #
    # Main processor commands (chapter 4).
    #

    # 4.5 (94)
    PRIMITIVES = {
            'BITMAPS': 1,
            'POINTS': 2,
            'LINES': 3,
            'LINE_STRIP': 4,
            'EDGE_STRIP_R': 5,
            'EDGE_STRIP_L': 6,
            'EDGE_STRIP_A': 7,
            'EDGE_STRIP_B': 8,
            'RECTS': 9
            }

    # 4.5 (94)
    def BEGIN(self, prim):
        prim = FT81x.PRIMITIVES[prim]

        data = (0x1f << 24) | prim
        return [data]

    # 4.21 (118)
    def CLEAR(self, color, stencil, tag):
        color = int(color)
        stencil = int(stencil)
        tag = int(tag)

        data = (0x26 << 24) | (color << 2) | (stencil << 1) | (tag << 0)
        return [data]

    # 4.28 (126)
    def COLOR_RGB(self, red, green, blue):
        red = int(red)
        green = int(green)
        blue = int(blue)

        data = (0x4 << 24) | (red << 16) | (blue << 8) | (green << 0)
        return [data]

    # 4.48 (146)
    def VERTEX2II(self, x, y, handle, cell):
        x = int(x)
        y = int(y)
        handle = int(handle)
        cell = int(cell)

        data = (0x2 << 24) | (x << 21) | (y << 12) | (handle << 7) | (cell << 0)
        return [data]


    #
    # Coprocessor commands (chapter 5).
    #

    string_pattern = re.compile(r'"(.*)"')

    # 5.8 (158-159)
    OPTIONS = {
            'OPT_3D': 0,
            'OPT_RGB565': 0,
            'OPT_MONO': 1,
            'OPT_NODL': 2,
            'OPT_FLAT': 256,
            'OPT_SIGNED': 256,
            'OPT_CENTERX': 512,
            'OPT_CENTERY': 1024,
            'OPT_CENTER': 1536,
            'OPT_RIGHTX': 2048,
            'OPT_NOBACK': 4096,
            'OPT_NOTICKS': 8192,
            'OPT_NOHM': 16384,
            'OPT_NOPOINTER': 16384,
            'OPT_NOSECS': 32768,
            'OPT_NOHANDS': 49152,
            'OPT_NOTEAR': 4,
            'OPT_FULLSCREEN': 8,
            'OPT_MEDIAFIFO': 16,
            'OPT_SOUND': 32
            }

    # 5.41 (213)
    def CMD_TEXT(self, x, y, font, options, s):
        x = int(x)
        y = int(y)
        font = int(font)
        s = bytearray(FT81x.string_pattern.match(s).group(1), 'utf8')
        s.append(0)     # NUL-terminate

        # Parse option flags
        opts = 0
        for option in options.split('|'):
            opts |= FT81x.OPTIONS[option.strip()]

        data = [0xffffff0c, (y << 16) | (x << 0), (opts << 16) | (x << 0)]

        # Encode string into 4-byte word groups
        for i in range(0, len(s), 4):
            word = 0
            for j in range(4):
                char = s[i + j] if i + j < len(s) else 0
                word |= (char << (j * 8))
            data.append(word)

        return data

    # 5.59 (238)
    def CMD_SETFONT2(self, font, ptr, firstchar):
        font = int(font)
        ptr = int(ptr)
        firstchar = int(firstchar)

        return [0xffffff3b, font, ptr, firstchar]

ft81x = FT81x()

if len(sys.argv) < 2:
    print('Usage: %s <ESE project file>' % (sys.argv[0]), file=sys.stderr)
    sys.exit(1)

command_pattern = re.compile(r'(.+)\((.+)\)')

with open(sys.argv[1], 'r') as ese_project_file:
    ese_project = json.load(ese_project_file)

    # Write display list start command.
    print("0xffffff00, // CMD_DLSTART()")

    for command in ese_project['coprocessor']:
        command = command.strip()
        matches = command_pattern.match(command)
        if (matches == None):
            continue

        name = matches.group(1).strip()
        args = list(map(str.strip, matches.group(2).split(',')))
        try:
            method = getattr(ft81x, name)
        except AttributeError:
            print('UNIMPLEMENTED: %s' % command)

        for i, word in enumerate(method(*args)):
            print('0x%s,%s' % (
                format(word, '08x'),
                (' // %s' % (command)) if i == 0 else ''
                ))

# Write display list swap command.
print('0xffffff01, // CMD_DLSWAP()')

