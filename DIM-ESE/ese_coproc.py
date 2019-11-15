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

    # 4.6 (96)
    def BITMAP_HANDLE(self, handle):
        handle = int(handle)

        data = (0x5 << 24) | (handle << 0)
        return [data]

    # 4.7 (97)
    BITMAP_FORMATS = {
            'ARGB1555': 0,
            'L1': 1,
            'L4': 2,
            'L8': 3,
            'RGB332': 4,
            'ARGB2': 5,
            'ARGB4': 6,
            'RGB565': 7,
            'TEXT8X8': 9,
            'TEXTVGA': 10,
            'BARGRAPH': 11,
            'PALETTED565': 14,
            'PALETTED4444': 15,
            'PALETTED8': 16,
            'L2': 17
            }

    # 4.18 (114)
    BLEND_FUNCS = {
            'ZERO': 0,
            'ONE': 1,
            'SRC_ALPHA': 2,
            'DST_ALPHA': 3,
            'ONE_MINUS_SRC_ALPHA': 4,
            'ONE_MINUS_DST_ALPHA': 5
            }

    # 4.18 (114)
    def BLEND_FUNC(self, src, dst):
        src = FT81x.BLEND_FUNCS[src]
        dst = FT81x.BLEND_FUNCS[dst]

        data = (0xB << 24) | (src << 3) | (dst << 0)
        return [data]

    # 4.21 (118)
    def CLEAR(self, color, stencil, tag):
        color = int(color)
        stencil = int(stencil)
        tag = int(tag)

        data = (0x26 << 24) | (color << 2) | (stencil << 1) | (tag << 0)
        return [data]

    # 4.23 (121)
    def CLEAR_COLOR_RGB(self, red, green, blue):
        red = int(red)
        green = int(green)
        blue = int(blue)

        data = (0x02 << 24) | (red << 16) | (blue << 8) | green
        return [data]

    # 4.27 (125)
    def COLOR_MASK(self, r, g, b, a):
        r = int(r)
        g = int(g)
        b = int(b)
        a = int(a)

        data = (0x20 << 24) | (r << 3) | (g << 2) | (b << 1) | (a << 0)
        return [data]

    # 4.28 (126)
    # The guide specifies (R, B, G) in the encoding, but (R, G, B) is apparently
    # correct from testing.
    def COLOR_RGB(self, red, green, blue):
        red = int(red)
        green = int(green)
        blue = int(blue)

        data = (0x4 << 24) | (red << 16) | (green << 8) | (blue << 0)
        return [data]

    # 4.30 (128)
    def END(self, _):
        data = (0x21 << 24)
        return [data]

    # 4.35 (132)
    def PALETTE_SOURCE(self, addr):
        addr = int(addr)
        data = (0x2A << 24) | (addr << 0)
        return [data]

    # 4.37 (134)
    def RESTORE_CONTEXT(self, _):
        data = (0x23 << 24)
        return [data]

    # 4.39 (136)
    def SAVE_CONTEXT(self, _):
        data = (0x22 << 24)
        return [data]

    # 4.48 (146)
    def VERTEX2II(self, x, y, handle, cell):
        x = int(x)
        y = int(y)
        handle = int(handle)
        cell = int(cell)

        data = (0x2 << 30) | (x << 21) | (y << 12) | (handle << 7) | (cell << 0)
        return [data]


    #
    # Coprocessor commands (chapter 5).
    #
    
    def get_options(self, options):
        # Parse option flags
        opts = 0
        for option in options.split('|'):
            opts |= FT81x.OPTIONS[option.strip()]
        return opts

    string_pattern = re.compile(r'"(.*)"')

    # 5.8 (158-159)
    
    OPTIONS  = {
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

    # 5.19 (169)
    def CMD_LOADIMAGE(self, ptr, options):
        ptr = int(ptr)
        options = int(options)
        opts = get_options(options)

        return [0xffffff24, ptr, opts]
        
    # 5.30 (183)
    def CMD_FGCOLOR(self, c):
        c = int(c)
        return [0xffffff0a, c]

    # 5.31 (184)
    def CMD_BGCOLOR(self):
        c = int(c)
        return [0xffffff09, c]

    # 5.33 (187)
    def CMD_GAUGE(self, x, y, r, options, major, minor, val, gauge_range): 
       #range is a keyword in Python
       x = int(x) 
       y = int(y) 
       r = int(r) 
       options = int(options) 
       major = int(major) 
       minor = int(minor) 
       val = int(val) 
       gauge_range = int(gauge_range) 

       data = [0xffffff13, ((x << 16) | (y << 0)), ((r << 16) | (options << 0)), \
             ((major << 16) | (minor << 0)), ((value << 16) | (gauge_range))] 
       return data

    # 5.36 (200)
    def CMD_PROGRESS(self, x, y, w, h, options, val, progress_range):
       #range is a keyword in Python
       x = int(x) 
       y = int(y) 
       r = int(r) 
       options = int(options) 
       major = int(major) 
       minor = int(minor) 
       val = int(val) 
       progress_range = int(progress_range) 
    
       data = [0xffffff0f, ((x << 16) | (y << 0)), ((w << 16) | (h << 0)), \
              ((options << 16) | (val << 0)), progress_range]

       return data

    # 5.41 (213)
    def CMD_TEXT(self, x, y, font, options, s):
        x = int(x)
        y = int(y)
        font = int(font)
        s = bytearray(FT81x.string_pattern.match(s).group(1), 'utf8')
        s.append(0)     # NUL-terminate

        opts = self.get_options(options)

        data = [0xffffff0c, (y << 16) | (x << 0), (opts << 16) | (font << 0)]

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

    # 5.65 (248-249)
    def CMD_SETBITMAP(self, addr, fmt, width, height):
        addr = int(addr)
        fmt = FT81x.BITMAP_FORMATS[fmt]
        width = int(width)
        height = int(height)

        return [0xffffff43, addr, (width << 16) | (fmt << 0), height]

    VARIABLE_OFFSETS = {
            # Please keep alphabetized
            'CLEAR_COLOR_RGB': 0,
            'CMD_TEXT': 3,
            'COLOR_RGB': 0,
            'VERTEX2II': 0,
        }

    def get_variable_offset(self, command):
        try:
            return self.VARIABLE_OFFSETS[command]
        except KeyError:
            print(f"// Unsupported use of @variable with command {command}, omitted")
            return -1

ft81x = FT81x()

if len(sys.argv) < 2:
    print('Usage: %s <ESE project file>' % (sys.argv[0]), file=sys.stderr)
    sys.exit(1)

command_pattern = re.compile(r'(.+)\((.*)\)([ ]+@variable[ ]+([a-zA-Z0-9_]*))?')

variables = []

unimplemented_cmds = 0

with open(sys.argv[1], 'r') as ese_project_file:
    ese_project = json.load(ese_project_file)

    # Write display list start command.
    print("0xffffff00, // CMD_DLSTART()")

    lines_written = 1

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
            unimplemented_cmds += 1
            continue

        if matches.group(4) is not None:
            if ft81x.get_variable_offset(name) is not -1:
                variables.append(f"#define {matches.group(4)} {lines_written + ft81x.get_variable_offset(name)}")

        for i, word in enumerate(method(*args)):
            print('0x%s,%s' % (
                format(word, '08x'),
                (' // %s' % (command)) if i == 0 else ''
                ))
            lines_written = lines_written + 1

# Write display and swap commands.
print('0x00000000, // DISPLAY()')
print('0xffffff01, // CMD_DLSWAP()')

for variable in variables:
    print(variable)

if unimplemented_cmds != 0:
    print(str(unimplemented_cmds) + ' unimplemented commands!')

