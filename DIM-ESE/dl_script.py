# Paste RAM_DL text from EVE Screen Editor below.
# It can be found in the 'Inspector' tab at the bottom left
# Ensure it has the following format:
# Ensure that the last line is a DISPLAY() cmd

dl = """
0	0x26000007	CLEAR(1, 1, 1)
13	0x04ffffff	COLOR_RGB(255, 255, 255)
14	0x1f000009	BEGIN(RECTS)
15	0x8ae5b000	VERTEX2II(87, 91, 0, 0)
16	0x9d2eb000	VERTEX2II(233, 235, 0, 0)
17	0x21000000	END()
196	0x00000000	DISPLAY()
"""

def function(dl):
    dl = dl.split("\n")
    new_dl =[]
    for x in dl:
        i = x.find("0x",0,14)
        s = x[i:i+10]
        new_dl.append(s+",\n")
    new_dl = ''.join(new_dl)
    return new_dl
    
print("\nHere's your display list :)\n")

new_dl = function(dl)[2:-3]
new_dl = '\n'.join(["0xffffff00,",new_dl,"0xffffff01"])
print(new_dl)
print("\nCopy the list into your workspace's co processor display list declaration.\n")
print("It should look like the following:\n")
print("uint32_t list_name[] = {")
print("\tpaste,")
print("\tthe,")
print("\tlist,")
print("\there")
print("}")