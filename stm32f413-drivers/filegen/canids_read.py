import re 
file = "stm32f413-drivers/CMR/include/CMR/can_ids.h" #FIX 
atIDs = False 
num = 0; 


lines = [] 

with open(file, "r") as f:
    for line in f: 
        if "}" in line: #dumb way to look at only ids.... can't think of another rn
            atIDs = False 
        if bool(atIDs) and "CAN" and "ID" in line: #we should standardize can id names ... 
            findnum = re.search(r"0x[0-9A-Fa-f]+", line) 
            if findnum: #if a hex number is in line  
                num = int(findnum.group(), 16) 
            else: #not numbered, use prev then store 
                line = line.split(",")[0] 
                line += " = " + hex(num + 1) + ",\n" 
                num += 1 
            if "+ CMR" in line: #hard coded offset for now
                line = line.split("=")[0] 
                line += "= " + hex(num + 928) + ",\n" 
        if "enum" in line: #can also use { to parallel 
            atIDs = True 
        lines.append(line) 


with open("stm32f413-drivers/filegen/canids_post.h", "w") as f:
    f.write("".join(lines)) 