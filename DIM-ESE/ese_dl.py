# Paste RAM_DL text from EVE Screen Editor after the prompt
# (It can be found in the 'Inspector' tab at the bottom left)
# Ensure the pasted text has line breaks after each command
# Ensure that the last line is a DISPLAY() cmd, perferably the first one
# Press CTRL-D to terminate Python shell to get output

import sys
print ('Paste stuff here:')

dl = sys.stdin.read()

def function(dl):
    dl = dl.split("\n")
    new_dl =[]
    for x in dl:
        i = x.find("0x",0,14)
        if (i == -1):
             pass
        else:
            s = x[i:i+10]
            new_dl.append(s+",\n")
    new_dl = ''.join(new_dl)
    return new_dl
    
print("\nHere's your display list :)\n")

new_dl = function(dl)
new_dl = '\n'.join(["0xffffff00,",new_dl])
print(new_dl + '0xffffff01')