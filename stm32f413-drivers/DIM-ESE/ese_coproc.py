#!/usr/bin/env python3
#
# ese_coproc.py
# Conversion from ESE project files to coprocessor command lists.
#
# Author: Carnegie Mellon Racing

import json
import re
import argparse
import os
import sys

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

    # 5.24 (173) - CMD_MEMCRC - compute a CRC-32 for memory
    def CMD_MEMCRC(self, ptr, num, result):
        ptr = int(ptr)
        num = int(num)
        result = int(result)
        return [0xffffff18, ptr, num, result]

    # 5.25 (174) - CMD_MEMZERO - write zero to a block of memory
    def CMD_MEMZERO(self, ptr, num):
        ptr = int(ptr)
        num = int(num)
        return [0xffffff1c, ptr, num]

    # 5.26 (175) - CMD_MEMSET - fill memory with a byte value
    def CMD_MEMSET(self, ptr, value, num):
        ptr = int(ptr)
        value = int(value)
        num = int(num)
        return [0xffffff1b, ptr, value, num]

    # 5.27 (176) - CMD_MEMCPY - copy a block of memory
    def CMD_MEMCPY(self, dest, src, num):
        dest = int(dest)
        src = int(src)
        num = int(num)
        return [0xffffff1d, dest, src, num]

    # 5.28 (176) - CMD_BUTTON - draw a button
    def CMD_BUTTON(self, x, y, w, h, font, options, s):
        x = int(x)
        y = int(y)
        w = int(w)
        h = int(h)
        font = int(font)
        s = bytearray(FT81x.string_pattern.match(s).group(1), 'utf8')
        s.append(0)  # NUL-terminate

        opts = self.get_options(options)

        data = [0xffffff0d, (y << 16) | (x << 0), (h << 16) | (w << 0),
                (opts << 16) | (font << 0)]

        # Encode string into 4-byte word groups
        for i in range(0, len(s), 4):
            word = 0
            for j in range(4):
                char = s[i + j] if i + j < len(s) else 0
                word |= (char << (j * 8))
            data.append(word)

        return data

    # 5.29 (179) - CMD_CLOCK - draw an analog clock
    def CMD_CLOCK(self, x, y, r, options, h, m, s, ms):
        x = int(x)
        y = int(y)
        r = int(r)
        h = int(h)
        m = int(m)
        s = int(s)
        ms = int(ms)

        opts = self.get_options(options)

        data = [0xffffff14, (y << 16) | (x << 0), (opts << 16) | (r << 0),
                (m << 16) | (h << 0), (ms << 16) | (s << 0)]

        return data

    # 5.30 (183)
    def CMD_FGCOLOR(self, c):
        c = int(c)
        return [0xffffff0a, c]

    # 5.31 (184)
    def CMD_BGCOLOR(self,c):
        c = int(c)
        return [0xffffff09, c]

    # 5.32 (185) - CMD_GRADCOLOR - set the 3D button highlight color
    def CMD_GRADCOLOR(self, c):
        c = int(c)
        return [0xffffff34, c]

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

    # 5.34 (193) - CMD_GRADIENT - draw a smooth color gradient
    def CMD_GRADIENT(self, x0, y0, rgb0, x1, y1, rgb1):
        x0 = int(x0)
        y0 = int(y0)
        rgb0 = int(rgb0)
        x1 = int(x1)
        y1 = int(y1)
        rgb1 = int(rgb1)

        return [0xffffff0b, (y0 << 16) | (x0 << 0), rgb0,
                (y1 << 16) | (x1 << 0), rgb1]

    # 5.35 (196) - CMD_KEYS - draw a row of keys
    def CMD_KEYS(self, x, y, w, h, font, options, s):
        x = int(x)
        y = int(y)
        w = int(w)
        h = int(h)
        font = int(font)
        s = bytearray(FT81x.string_pattern.match(s).group(1), 'utf8')
        s.append(0)  # NUL-terminate

        opts = self.get_options(options)

        data = [0xffffff0e, (y << 16) | (x << 0), (h << 16) | (w << 0),
                (opts << 16) | (font << 0)]

        # Encode string into 4-byte word groups
        for i in range(0, len(s), 4):
            word = 0
            for j in range(4):
                char = s[i + j] if i + j < len(s) else 0
                word |= (char << (j * 8))
            data.append(word)

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

    # 5.37 (203) - CMD_SCROLLBAR - draw a scroll bar
    def CMD_SCROLLBAR(self, x, y, w, h, options, val, size, range):
        x = int(x)
        y = int(y)
        w = int(w)
        h = int(h)
        val = int(val)
        size = int(size)
        range = int(range)

        opts = self.get_options(options)

        data = [0xffffff11, (y << 16) | (x << 0), (h << 16) | (w << 0),
                (val << 16) | (opts << 0), (range << 16) | (size << 0)]

        return data

    # 5.38 (205) - CMD_SLIDER - draw a slider
    def CMD_SLIDER(self, x, y, w, h, options, val, range):
        x = int(x)
        y = int(y)
        w = int(w)
        h = int(h)
        val = int(val)
        range = int(range)

        opts = self.get_options(options)

        data = [0xffffff10, (y << 16) | (x << 0), (h << 16) | (w << 0),
                (val << 16) | (opts << 0), range]

        return data

    # 5.39 (207) - CMD_DIAL - draw a rotary dial control
    def CMD_DIAL(self, x, y, r, options, val):
        x = int(x)
        y = int(y)
        r = int(r)
        val = int(val)

        opts = self.get_options(options)

        data = [0xffffff2d, (y << 16) | (x << 0), (opts << 16) | (r << 0), val]

        return data

    # 5.40 (210) - CMD_TOGGLE - draw a toggle switch
    def CMD_TOGGLE(self, x, y, w, font, options, state, s):
        x = int(x)
        y = int(y)
        w = int(w)
        font = int(font)
        state = int(state)
        s = bytearray(FT81x.string_pattern.match(s).group(1), 'utf8')
        s.append(0)  # NUL-terminate

        opts = self.get_options(options)

        data = [0xffffff12, (y << 16) | (x << 0), (font << 16) | (w << 0),
                (state << 16) | (opts << 0)]

        # Encode string into 4-byte word groups
        for i in range(0, len(s), 4):
            word = 0
            for j in range(4):
                char = s[i + j] if i + j < len(s) else 0
                word |= (char << (j * 8))
            data.append(word)

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

    # 5.42 (216) - CMD_SETBASE - Set the base for number output
    def CMD_SETBASE(self, b):
        b = int(b)
        return [0xffffff38, b]

    # 5.43 (217) - CMD_NUMBER - draw number
    def CMD_NUMBER(self, x, y, font, options, n):
        x = int(x)
        y = int(y)
        font = int(font)
        n = int(n)

        opts = self.get_options(options)

        data = [0xffffff2e, (y << 16) | (x << 0), (opts << 16) | (font << 0), n]

        return data

    # 5.44 (220) - CMD_LOADIDENTITY - Set the current matrix to the identity matrix
    def CMD_LOADIDENTITY(self, _):
        return [0xffffff26]

    # 5.45 (220) - CMD_SETMATRIX - write the current matrix to the display list
    def CMD_SETMATRIX(self, _):
        return [0xffffff2a]

    # 5.46 (221) - CMD_GETMATRIX - retrieves the current matrix coefficients
    def CMD_GETMATRIX(self, a, b, c, d, e, f):
        a = int(a)
        b = int(b)
        c = int(c)
        d = int(d)
        e = int(e)
        f = int(f)
        return [0xffffff33, a, b, c, d, e, f]

    # 5.47 (222) - CMD_GETPTR - get the end memory address of data inflated by CMD_INFLATE
    def CMD_GETPTR(self, result):
        result = int(result)
        return [0xffffff23, result]

    # 5.48 (223) - CMD_GETPROPS - get the image properties decompressed by CMD_LOADIMAGE
    def CMD_GETPROPS(self, ptr, width, height):
        ptr = int(ptr)
        width = int(width)
        height = int(height)
        return [0xffffff25, ptr, width, height]

    # 5.49 (223) - CMD_SCALE - apply a scale to the current matrix
    def CMD_SCALE(self, sx, sy):
        sx = int(sx)
        sy = int(sy)
        return [0xffffff28, sx, sy]

    # 5.50 (225) - CMD_ROTATE - apply a rotation to the current matrix
    def CMD_ROTATE(self, a):
        a = int(a)
        return [0xffffff29, a]

    # 5.51 (226) - CMD_TRANSLATE - apply a translation to the current matrix
    def CMD_TRANSLATE(self, tx, ty):
        tx = int(tx)
        ty = int(ty)
        return [0xffffff27, tx, ty]

    # 5.52 (227) - CMD_CALIBRATE - execute the touch screen calibration routine
    def CMD_CALIBRATE(self, result):
        result = int(result)
        return [0xffffff15, result]

    # 5.53 (228) - CMD_SETROTATE - Rotate the screen
    def CMD_SETROTATE(self, r):
        r = int(r)
        return [0xffffff36, r]

    # 5.54 (229) - CMD_SPINNER - start an animated spinner
    def CMD_SPINNER(self, x, y, style, scale):
        x = int(x)
        y = int(y)
        style = int(style)
        scale = int(scale)
        return [0xffffff16, (y << 16) | (x << 0), (scale << 16) | (style << 0)]

    # 5.55 (233) - CMD_SCREENSAVER - start an animated screensaver
    def CMD_SCREENSAVER(self, _):
        return [0xffffff2f]

    # 5.56 (234) - CMD_SKETCH - start a continuous sketch update
    def CMD_SKETCH(self, x, y, w, h, ptr, format):
        x = int(x)
        y = int(y)
        w = int(w)
        h = int(h)
        ptr = int(ptr)
        format = FT81x.BITMAP_FORMATS[format] if isinstance(format, str) else int(format)

        return [0xffffff30, (y << 16) | (x << 0), (h << 16) | (w << 0), ptr, format]

    # 5.57 (236) - CMD_STOP - stop any of spinner, screensaver or sketch
    def CMD_STOP(self, _):
        return [0xffffff17]

    # 5.58 (237) - CMD_SETFONT - set up a custom font
    def CMD_SETFONT(self, font, ptr):
        font = int(font)
        ptr = int(ptr)
        return [0xffffff2b, font, ptr]

    # 5.59 (238)
    def CMD_SETFONT2(self, font, ptr, firstchar):
        font = int(font)
        ptr = int(ptr)
        firstchar = int(firstchar)

        return [0xffffff3b, font, ptr, firstchar]

    # 5.60 (239) - CMD_SETSCRATCH - set the scratch bitmap for widget use
    def CMD_SETSCRATCH(self, handle):
        handle = int(handle)
        return [0xffffff3c, handle]

    # 5.61 (239)
    def CMD_ROMFONT(self, font, romslot):
        font = int(font)
        romslot = int(romslot)

        return [0xffffff3f, font, romslot]

    # 5.62 (240) - CMD_TRACK - track touches for a graphics object
    def CMD_TRACK(self, x, y, w, h, tag):
        x = int(x)
        y = int(y)
        w = int(w)
        h = int(h)
        tag = int(tag)

        return [0xffffff2c, (y << 16) | (x << 0), (h << 16) | (w << 0), tag]

    # 5.63 (245) - CMD_SNAPSHOT - take a snapshot of the current screen
    def CMD_SNAPSHOT(self, ptr):
        ptr = int(ptr)
        return [0xffffff1f, ptr]

    # 5.64 (246) - CMD_SNAPSHOT2 - take a snapshot of part of the current screen
    def CMD_SNAPSHOT2(self, fmt, ptr, x, y, w, h):
        if isinstance(fmt, str):
            fmt = FT81x.BITMAP_FORMATS[fmt]
        else:
            fmt = int(fmt)
        ptr = int(ptr)
        x = int(x)
        y = int(y)
        w = int(w)
        h = int(h)

        return [0xffffff37, fmt, ptr, (y << 16) | (x << 0), (h << 16) | (w << 0)]

    # 5.65 (248-249)
    def CMD_SETBITMAP(self, addr, fmt, width, height):
        addr = int(addr)
        fmt = FT81x.BITMAP_FORMATS[fmt]
        width = int(width)
        height = int(height)

        return [0xffffff43, addr, (width << 16) | (fmt << 0), height]

    # 5.66 (249) - CMD_LOGO - play FTDI logo animation
    def CMD_LOGO(self, _):
        return [0xffffff31]

    VARIABLE_OFFSETS = {
            # Please keep alphabetized
            'CLEAR_COLOR_RGB': 0,
            'CMD_BUTTON': 4,
            'CMD_KEYS': 4,
            'CMD_NUMBER': 3,
            'CMD_TEXT': 3,
            'CMD_TOGGLE': 4,
            'COLOR_RGB': 0,
            'VERTEX2F' : 0,
            'VERTEX2II': 0,
        }
    
    def get_variable_offset(self, command):
        if command in self.VARIABLE_OFFSETS:
            return self.VARIABLE_OFFSETS[command]
        else:
            print(f"Unsupported use of @variable with command {command}!", file=sys.stderr)
            return -1

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("input_file", help=".ese file")
    parser.add_argument("--output", "-o", help="Output file")
    args = parser.parse_args()

    ft81x = FT81x()
    command_pattern = re.compile(r'(.+)\((.*)\)( +@variable +(\w+))?')
    variables = []
    unimplemented_cmds = 0

    # Determine output file path
    if args.output:
        output_file_path = args.output
    else:
        # Get the basename of the input file without extension
        base_name = os.path.basename(args.input_file)
        base_name_without_ext = os.path.splitext(base_name)[0]

        # Create the output directory if it doesn't exist
        output_dir = os.path.join("stm32f413-drivers", "DIM-ESE", "include", "DIM-ESE")
        os.makedirs(output_dir, exist_ok=True)

        # Set the output file path with .rawh extension
        output_file_path = os.path.join(output_dir, base_name_without_ext + ".rawh")

    # Process the ESE project file
    with open(args.input_file, 'r') as ese_project_file:
        ese_project = json.load(ese_project_file)

        # Open the output file
        with open(output_file_path, 'w') as output_file:
            # Write display list start command
            output_file.write("0xffffff00, // CMD_DLSTART()\n")

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
                    print(f"UNIMPLEMENTED: {command}", file=sys.stderr)
                    unimplemented_cmds += 1
                    continue

                if matches.group(4) is not None:
                    if ft81x.get_variable_offset(name) != -1:
                        variables.append(f"#define ESE_{matches.group(4)} {data_offset + ft81x.get_variable_offset(name)}")

                for i, word in enumerate(method(*args)):
                    output_file.write(f"0x{word:08x},{(' // ' + command) if i == 0 else ''}\n")
                    data_offset += 1

            # Write display and swap commands
            output_file.write('0x00000000, // DISPLAY()\n')
            output_file.write('0xffffff01, // CMD_DLSWAP()\n')

            # Write variable definitions
            for variable in variables:
                output_file.write(variable + '\n')

            if unimplemented_cmds != 0:
                print(str(unimplemented_cmds) + ' unimplemented commands!', file=sys.stderr)

    print(f"Output written to {output_file_path}")

if __name__ == "__main__":
    main()