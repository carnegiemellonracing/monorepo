#!/usr/bin/env python3
#
# ese_coproc.py
# Conversion from ESE project files to coprocessor command lists.
#
# Author: Carnegie Mellon Racing

import json
import re
import argparse

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
    # 4.8 (103)
    def BITMAP_LAYOUT(self,linestride, height):
        linestride = int(linestride)
        height = int(height)

        data = (0x28 << 24) | (linestride << 2) | (height << 0)
        return [data]
    # 4.9 (103)
    def BITMAP_SIZE(self, filter, wrapx, wrapy, width, height):
        filter = int(filter)
        wrapx = int(wrapx)
        wrapy = int(wrapy)
        width = int(width)
        height = int(height)

        data = (0x08 << 24) | (filter << 20) | (wrapx << 19) | (wrapy << 18) | (width << 9) | (height << 0)
        return [data]

    # 4.10 (105)
    def BITMAP_SIZE_H(self, width,height):
        width = int(width)
        height = int(height)
        data = (0x29 << 24) | (width << 2) | (height << 0)
        return [data]

    # 4.11 (106)
    def BITMAP_SOURCE(self, addr):
        addr = int(addr)
        data = (0x01 << 24) | (addr << 0)
        return [data]

    # 4.12 (108)
    def BITMAP_TRANSFORM_A(self, a):
        a = int(a)
        data = (0x15 << 24) | (a << 0)
        return [data]

    # 4.13 (109)
    def BITMAP_TRANSFORM_B(self, b):
        b = int(b)
        data = (0x16 << 24) | (b << 0)
        return [data]

    # 4.14 (110)
    def BITMAP_TRANSFORM_C(self, c):
        c = int(c)
        data = (0x17 << 24) | (c << 0)
        return [data]

    # 4.15 (111)
    def BITMAP_TRANSFORM_D(self, d):
        d = int(d)
        data = (0x18 << 24) | (d << 0)
        return [data]

    # 4.16 (112)
    def BITMAP_TRANSFORM_E(self, e):
        e = int(e)
        data = (0x19 << 24) | (e << 0)
        return [data]

    # 4.17 (113)
    def BITMAP_TRANSFORM_F(self, f):
        f = int(f)
        data = (0x1A << 24) | (f << 0)
        return [data]

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

    # 4.19 (116)
    def CALL(self, dest):
        dest = int(dest)
        data = (0x1d << 24) | (dest << 0)
        return [data]

    # 4.20 (117)
    def CELL(self, cell):
        cell = int(cell)
        data = (0x06 << 24) | (cell << 0)
        return [data]

    # 4.21 (118)
    def CLEAR(self, color, stencil, tag):
        color = int(color)
        stencil = int(stencil)
        tag = int(tag)

        data = (0x26 << 24) | (color << 2) | (stencil << 1) | (tag << 0)
        return [data]

    # 4.22 (120)
    def CLEAR_COLOR_A(self, alpha):
        alpha = int(alpha)
        data = (0x0f << 24) | (alpha << 0)
        return [data]

    # 4.23 (121)
    def CLEAR_COLOR_RGB(self, red, green, blue):
        red = int(red)
        green = int(green)
        blue = int(blue)

        data = (0x02 << 24) | (red << 16) | (blue << 8) | green
        return [data]

    #4.24 (122)
    def CLEAR_STENCIL(self, s):
        s = int(s)
        data = (0x11 << 24) | (s << 0)
        return [data]

    # 4.25 (123)
    def CLEAR_TAG(self, t):
        t = int(t)
        data = (0x12 << 24) | (t << 0)
        return [data]

    # 4.26 (124)
    def COLOR_A(self, a):
        a = int(a)
        data = (0x10 << 24) | (a << 0)
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

    #4.29 (127)
    def DISPLAY(self,_):
        data = (0x0 << 24)
        return [data]

    # 4.30 (128)
    def END(self, _):
        data = (0x21 << 24)
        return [data]

    # 4.31 (129)
    def JUMP(self, dest):
        dest = int(dest)
        data = (0x1e << 24) | (dest << 0)
        return [data]

    # 4.32 (130)
    def LINE_WIDTH(self, width):
        data = (0x0E << 24) | (int(width) << 0)
        return [data]

    # 4.33 (131)
    def MACRO(self, m):
        m = int(m)
        data = (0x25 << 24) | (m << 0)
        return [data]

    # 4.34 (131)
    def NOP(self,_):
        data = (0x2D << 24)
        return [data]

    # 4.35 (132)
    def PALETTE_SOURCE(self, addr):
        addr = int(addr)
        data = (0x2A << 24) | (addr << 0)
        return [data]

    # 4.36 (133)
    def POINT_SIZE(self, size):
        size = int (size)
        data = (0x0D << 24) | size
        return [data]

    # 4.37 (134)
    def RESTORE_CONTEXT(self, _):
        data = (0x23 << 24)
        return [data]
    # 4.38 (135)
    def RETURN(self, _):
        data = (0x24 << 24)
        return [data]

    # 4.39 (136)
    def SAVE_CONTEXT(self, _):
        data = (0x22 << 24)
        return [data]

    # 4.40 (137)
    def SCISSOR_SIZE(self, width, height):
        width = int(width)
        height = int(height)
        data = (0x1C << 24) | (width << 12) | (height << 0)
        return [data]

    # 4.41 (138)
    def SCISSOR_XY(self, x, y):
        x = int(x)
        y = int(y)
        data = (0x1B << 24) | (x << 11) | (y << 0)
        return [data]

    # 4.42 (139)
    def STENCIL_FUNC(self, func, ref, mask):
        func = int(func)
        ref = int(ref)
        mask = int(mask)
        data = (0x0A << 24) | (func << 16) | (ref << 8) | (mask << 0)
        return [data]

    # 4.43 (140)
    def STENCIL_MASK(self, mask):
        mask = int(mask)
        data = (0x13 << 24) | (mask << 0)
        return [data]

    # 4.44 (141)
    def STENCIL_OP(self, sfail, spass):
        sfail = int(sfail)
        spass = int(spass)
        data = (0x0C << 24) | (sfail << 3) | (spass << 0)
        return [data]

    # 4.45 (143)
    def TAG(self, s):
        s = int(s)
        data = (0x3 << 24) | (s << 0)
        return [data]

    # 4.46 (144)
    def TAG_MASK(self, mask):
        mask = int(mask)
        data = (0x14 << 24) | (mask << 0)
        return [data]

    # 4.47 (145)
    def VERTEX2F(self, x, y):
        x = int(x)
        y = int(y)

        data = (0x1 << 30) | (x << 15) | (y << 0)
        return [data]

    # 4.48 (146)
    def VERTEX2II(self, x, y, handle, cell):
        x = int(x)
        y = int(y)
        handle = int(handle)
        cell = int(cell)

        data = (0x2 << 30) | (x << 21) | (y << 12) | (handle << 7) | (cell << 0)
        return [data]

    # 4.49 (147)
    def VERTEX_FORMAT(self, frac):
        frac = int(frac)
        data = (0x27 << 24) | (frac << 0)
        return [data]

    # 4.50 (148)
    def VERTEX_TRANSLATE_X(self, x):
        x = int(x)
        data = (0x2B << 24) | (x << 0)
        return [data]

    # 4.51 (149)
    def VERTEX_TRANSLATE_Y(self, y):
        y = int(y)
        data = (0x2C << 24) | (y << 0)
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
            'OPT_NONE': 0,
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

    # 5.11 (162)
    def CMD_DLSTART(self, _):
        return [0xffffff00]

    # 5.12 (163)
    def CMD_SWAP(self, _):
        return [0xffffff01]

    # 5.13 (164)
    def CMD_COLDSTART(self, _):
        return [0xffffff32]

    # 5.14 (165)
    def CMD_INTERRUPT(self, ms):
        ms = int(ms)
        return [0xffffff02, ms]

    # 5.15 (166)
    def CMD_APPEND(self, ptr, num):
        ptr = int(ptr)
        num = int(num)
        return [0xffffff1e, ptr, num]

    # 5.16 (167)
    def CMD_REGREAD(self, ptr, result):
        ptr = int(ptr)
        result = int(result)
        return [0xffffff19, ptr, result]

    # 5.17 (168)
    def CMD_MEMWRITE(self, ptr, num):
        ptr = int(ptr)
        num = int(num)
        return [0xffffff1a, ptr, num]

    # 5.18 (168)
    def CMD_INFLATE(self, ptr):
        ptr = int(ptr)
        return [0xffffff22, ptr]

    # 5.19 (169)
    def CMD_LOADIMAGE(self, ptr, options):
        ptr = int(ptr)
        options = int(options)
        opts = self.get_options(options)

        return [0xffffff24, ptr, opts]

    # 5.20 (170)
    def CMD_MEDIAFIFO(self, ptr, size):
        ptr = int(ptr)
        size = int(size)
        return [0xffffff39, ptr, size]

    # 5.21 (171)
    def CMD_PLAYVIDEO(self, opts):
        opts = int(opts)
        return [0xffffff3a, opts]

    # 5.22 (172)
    def CMD_VIDEOSTART(self, _):
        return [0xffffff40]

    # 5.23 (173)
    def CMD_VIDEOFRAME(self, dst, ptr):
        dst = int(dst)
        ptr = int(ptr)
        return [0xffffff41, dst, ptr]

    
    # 5.30 (183)
    def CMD_FGCOLOR(self, c):
        c = int(c)
        return [0xffffff0a, c]

    # 5.31 (184)
    def CMD_BGCOLOR(self,c):
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
             ((major << 16) | (minor << 0)), ((val << 16) | (gauge_range))]
       return data

    # 5.36 (200)
    def CMD_PROGRESS(self, x, y, w, h, options, val, progress_range):
       #range is a keyword in Python
       x = int(x)
       y = int(y)
       w = int(w)
       h = int(h)
       options = int(options)
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

    # 5.61 (239)
    def CMD_ROMFONT(self, font, romslot):
        font = int(font)
        romslot = int(romslot)

        return [0xffffff3f, font, romslot]

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
            'VERTEX2F' : 0,
            'VERTEX2II': 0,
        }

    def get_variable_offset(self, command):
        if command in self.VARIABLE_OFFSETS:
            return self.VARIABLE_OFFSETS[command]
        else:
            print(f"Unsupported use of @variable with command {command}!")
            return -1

parser = argparse.ArgumentParser()
parser.add_argument("input_file",help = ".ese file")
args = parser.parse_args()

ft81x = FT81x()

command_pattern = re.compile(r'(.+)\((.*)\)( +@variable +(\w+))?')

variables = []

unimplemented_cmds = 0

with open(args.input_file, 'r') as ese_project_file:
    ese_project = json.load(ese_project_file)

    # Write display list start command.
    print("0xffffff00, // CMD_DLSTART()")

    data_offset = 1

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
            print(f"UNIMPLEMENTED: {command}")
            unimplemented_cmds += 1
            continue

        if matches.group(4) is not None:
            if ft81x.get_variable_offset(name) != -1:
                variables.append(f"#define ESE_{matches.group(4)} {data_offset + ft81x.get_variable_offset(name)}")

        for i, word in enumerate(method(*args)):
            print(f"0x{word:08x},{(' // ' + command) if i == 0 else ''}")
            data_offset += 1
# Write display and swap commands.
print('0x00000000, // DISPLAY()')
print('0xffffff01, // CMD_DLSWAP()')

for variable in variables:
    print(variable)

if unimplemented_cmds != 0:
    print(str(unimplemented_cmds) + ' unimplemented commands!')