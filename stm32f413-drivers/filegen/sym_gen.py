import re
import json

output = "stm32f413-drivers/filegen/symv1.sym"
symlines = [] 
repeat_names = {} #canid, number of times repeated 

def id2hex(id):
    #uses canids_post.h to map id(from canid_type_map) to the hex number
    with open("stm32f413-drivers/filegen/canids_post.h", "r") as file:
        for line in file:
            if id in line:
                hex = re.search(r"0x[0-9A-Fa-f]+", line) 
                return hex.group().split("x")[1]

def add_mapper_data(canid, cycletime, timeout, structlines):
        name = re.findall(r'CMR_CANID_(\w+)',canid) 
        if canid in repeat_names:
            repeat_num = repeat_names[canid]
            if repeat_num > 0:
                symlines.append("["+name[0]+str(repeat_num)+"]")
            else:
                symlines.append("["+name[0]+"]")
        else:
            symlines.append("["+name[0]+"]")
        if id2hex(canid):
            structlines.append("ID:"+id2hex(canid)) 
        structlines.append("CycleTime="+str(cycletime))
        structlines.append("TimeOut="+str(timeout))

def get_cantypes_data(cantype, structs):
    for fields, name in structs:
        if re.search(name, cantype): 
            #find struct declaration with the right can type 
            return re.findall(r'\b((?:u)?int\d+_t|float)\s+(\w+)\b', fields) 
        
def format_bitpacking_1(vartype, name, size, packed_num, atbit, structlines):
#packed_num is the number of fields packed into var
    packed_size = int(size/packed_num)
    for i in range (0, packed_num):
        structlines.append("Var="+name+str(i)+" "+vartype+ " " +str(atbit)+","+str(packed_size))
        atbit+=packed_size
    return atbit

def format_bitpacking_2(vartype, atbit, packedinfo, structlines):
    for littlename in packedinfo:
        if littlename.startswith("empty"):
            atbit+=packedinfo[littlename]
            continue
        else:
            structlines.append("Var="+littlename+" "+vartype+" "+str(atbit)+","+str(packedinfo[littlename]))
            atbit+=packedinfo[littlename]
    return atbit

def format_fields(cantype, matches, structlines):
    atbit = 0
    size = None
    bitpacking = None
    bptype = 0 
    with open("stm32f413-drivers/filegen/bitpacking.json", "r") as file:
        bfile = json.load(file) 
        if cantype in bfile["type1"]:
            bitpacking = bfile["type1"][cantype]
            bptype=1
        elif cantype in bfile["type2"]:
            bitpacking = bfile["type2"][cantype]
            bptype=2
    for vartype, name in matches:
        findsize = re.search(r'\d+', vartype)
        if findsize:
            size = int(findsize.group())
            if vartype.startswith('u'):
                vartype = 'unsigned'
            else: 
                vartype = 'signed'
        else:
            if vartype == 'float':
                #technically unnecessary check, all others should be float
                size = 32 
        if bitpacking and name in bitpacking:
            #this field is bitpacked
            if(bptype==1):
                atbit = format_bitpacking_1(vartype, name, size, bitpacking[name], atbit, structlines)
                continue
            if(bptype==2):
                atbit = format_bitpacking_2(vartype, atbit, bitpacking[name], structlines)
                continue
        if size:
            structlines.append("Var="+name+" "+vartype+ " " +str(atbit)+","+str(size))
            atbit+=int(size)
        else:
            structlines.append("Issue with type of field")
    return str(int(atbit/8))

def check_repeat(canid):
    can_name = re.findall(r'CMR_CANID_(\w+)',canid)
    for line in symlines: 
        if can_name and can_name[0] in line:
            line = line.split("]")[0] + "0]" #??
            if canid not in repeat_names:
                repeat_names[canid]=1
                break
            else:
                repeat_names[canid]+=1 
                break


def main():
    with open("stm32f413-drivers/CMR/include/CMR/can_types.h", "r") as structf: 
        #find all struct declarations
        structs = re.findall(r'typedef\s+struct\s*\{([\s\S]*?)\}\s*(cmr_can\w+)\s*;', structf.read())
    with open("stm32f413-drivers/filegen/canid_type_map.json", "r") as file: 
        json_obj = json.load(file)
        canids = json_obj['canid_to_info']
        for canid in canids:
            #go through each can id dict 
            cantype = canids[canid]['type']
            cycletime = canids[canid]['cycleTime']
            timeout = canids[canid]['timeOut']
            if re.fullmatch(r'(cmr_[a-zA-Z0-9_]*_t)', cantype):
                check_repeat(canid)
                #valid can type in dict 
                structlines = []
                #look at mapper json for data
                add_mapper_data(canid, cycletime, timeout, structlines)
                #find and format the correct struct in cantypes.h 
                matches = get_cantypes_data(cantype, structs)
                if matches: 
                    dlc = format_fields(cantype, matches, structlines)
                    structlines.insert(1, "DLC="+dlc)
                else:
                    structlines.append("error with this struct in can_types.h")
                #add all formatted data to symbol file 
                for line in structlines:
                    symlines.append(line)
                symlines.append("\n") 
    #write into symbol file 
    with open("stm32f413-drivers/filegen/symv1.sym", "w") as file:
        file.write("\n".join(symlines))
    print(repeat_names)
    print("done") 

if __name__ == "__main__":
    main()
