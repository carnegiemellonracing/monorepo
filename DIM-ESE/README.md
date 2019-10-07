# `DIM-ESE`
## Driver Interface Module EVE Screen Editor Project

This repository contains screens for the Driver Interface Module's (DIM) display.

## Screen Creation Workflow

1. Create or edit a screen using the [EVE Screen Editor (ESE)][ESE].
2. Use the script ese_coproc.py to convert the resulting .ese file. Copy the script output into an appropriately named .rawh file.
3. The .rawh file can now be included in DIM code. See DIM/tftDL.c to see how other .rawh files are included.

## Fonts workaround

It appears that ESE's support for importing fonts directly is broken. Instead,
they can be first converted into `raw` binary format using the
[EVE Asset Builder (EAB)][EAB].

1. Under "Generate Font", select the font file, font size, and output
   directory (the `fonts/` directory in this project).
2. Select the "Legacy format" tab.
3. Set "EVE Command Support" to `SETFONT2`.
4. Set the "EVE Address" to one that does not conflict with existing content
   already added in the ESE project.
   - You can observe the address and size of existing content by selecting the
     appropriate item in the ESE "Content" tab in the left sidebar, then looking
     at its "Address" and "Size" in its "Properties" view.
5. If only a subset of characters is needed, select the "User Defined Character
   Set" and an appropriate file. (See `fonts/numbers.txt` for an example.)
6. Press `Convert`.

The generated `*.raw` file (located under `fonts/<font>_<size>/L<bits>/` can
then added in ESE's "Content" tab in the left sidebar.

1. The `L<bits>` value indicates the number of bits used to represent each pixel.
    - Higher values look better, but use significantly more graphics memory.
2. Once the file has been selected, please change the path to be relative.
    - I.e., delete everything before `fonts/...` in the "Source file" property.
3. The converter type should be "Raw".
4. The "Start" should be 0; the "Length" should be the length of the file in
   bytes, rounded up to the closest multiple of 4 for alignment.
5. The "Address" should be the same address used when converting the font.
6. The other settings should remain at their defaults:
    - "Loaded: (yes)"
    - "Storage: Embedded"
    - "Compressed: (yes)"

[ESE]: https://brtchip.com/eve-toolchains/#EVEScreenEditor
[EAB]: https://brtchip.com/eve-toolchains/#EVEAssetBuilder

