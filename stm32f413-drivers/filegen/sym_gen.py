import re
import json

output = "stm32f413-drivers/filegen/symv1.sym"
symlines = [] 
repeat_names = {} #canid, number of times repeated 
units = ["mps"]


def numbercanids():
    #for can id file numbering 
    canidfile = "stm32f413-drivers/CMR/include/CMR/can_ids.h" 
    atIDs = False 
    num = 0; 
    lines = [] 
    with open(canidfile, "r") as f:
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

    with open("stm32f413-drivers/filegen/can_ids.h", "w") as f:
        f.write("".join(lines)) 

def id2hex(id):
    #uses can_ids.h to map id(from canid_type_map) to the hex number
    numbercanids() 
    with open("stm32f413-drivers/filegen/can_ids.h", "r") as file:
        for line in file:
            if id in line:
                hex = re.search(r"0x[0-9A-Fa-f]+", line) 
                return hex.group().split("x")[1]

def add_mapper_data(canid, cycletime, timeout, structlines):
        name = re.findall(r'CMR_CANID_(\w+)',canid) 
        if canid in repeat_names:
            repeat_num = repeat_names[canid]
            if repeat_num > 0:
                structlines.append("["+name[0]+str(repeat_num)+"]")
            else:
                structlines.append("["+name[0]+"]")
        else:
            structlines.append("["+name[0]+"]")
        if id2hex(name[0]):
            structlines.append("ID="+id2hex(name[0])) 
        structlines.append("CycleTime="+str(cycletime))
        structlines.append("TimeOut="+str(timeout))

def get_cantypes_data(cantype, structs):
    for fields, name in structs:
        if re.search(name, cantype): 
            #find struct declaration with the right can type 
            return re.findall(r'\b((?:u)?int\d+_t|float)\s+(\w+)\b', fields) 
        
def format_bitpacking_1(vartype, name, size, packed_num, atbit, structlines, field_params=None):
#packed_num is the number of fields packed into var
    packed_size = int(size/packed_num)
    for i in range (0, packed_num):
        appendstr="Var="+name+str(i)+" "+vartype+ " " +str(atbit)+","+str(packed_size)
        # Add field-specific parameters if available
        if field_params and name in field_params:
            appendstr += format_field_params(field_params[name])
        structlines.append(appendstr)
        atbit+=packed_size
    return atbit

def format_bitpacking_2(vartype, atbit, packedinfo, structlines, field_params=None):
    for littlename in packedinfo:
        if littlename.startswith("empty"):
            atbit+=packedinfo[littlename]
            continue
        else:
            appendstr = "Var="+littlename+" "+vartype+" "+str(atbit)+","+str(packedinfo[littlename])
            # Add field-specific parameters if available
            if field_params and littlename in field_params:
                appendstr += format_field_params(field_params[littlename])
            structlines.append(appendstr)
            atbit+=packedinfo[littlename]
    return atbit

def format_field_params(params):
    """Format field-specific parameters like u, f, p, e, etc."""
    param_str = ""
    if 'u' in params:
        param_str += f" /u:{params['u']}"
    if 'f' in params:
        param_str += f" /f:{params['f']}"
    if 'p' in params:
        param_str += f" /p:{params['p']}"
    if 'e' in params:
        param_str += f" /e:{params['e']}"
    if 'min' in params:
        param_str += f" /min:{params['min']}"
    if 'max' in params:
        param_str += f" /max:{params['max']}"
    return param_str

def format_fields(cantype, matches, structlines, field_params=None):
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
            bptype=2 #HVC heartbeat needs to be added 
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
                atbit = format_bitpacking_1(vartype, name, size, bitpacking[name], atbit, structlines, field_params)
                continue
            if(bptype==2):
                atbit = format_bitpacking_2(vartype, atbit, bitpacking[name], structlines, field_params)
                continue
        if size:
            appendstr = "Var="+name+" "+vartype+ " " +str(atbit)+","+str(size)
            # Add field-specific parameters if available
            if field_params and name in field_params:
                appendstr += format_field_params(field_params[name])
            structlines.append(appendstr)
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

def extract_field_params(canid_data):
    """Extract field-specific parameters from the canid data"""
    field_params = {}
    for key, value in canid_data.items():
        if key not in ['cycleTime', 'timeOut', 'type'] and isinstance(value, dict):
            field_params[key] = value
    return field_params if field_params else None

def main():
    with open("stm32f413-drivers/filegen/can_types_new.h", "r") as structf: 
        #find all struct declarations
        structs = re.findall(r'typedef\s+struct\s*\{([\s\S]*?)\}\s*(cmr_can\w+)\s*;', structf.read())
    with open("stm32f413-drivers/filegen/canid_type_map.json", "r") as file: 
        json_obj = json.load(file)
        canids = json_obj['canid_to_info']
        for canid in canids:
            #go through each can id dict 
            canid_data = canids[canid]
            cantype = canid_data['type']
            cycletime = canid_data['cycleTime']
            timeout = canid_data['timeOut']
            
            # Extract field-specific parameters
            field_params = extract_field_params(canid_data)
            
            if re.fullmatch(r'(cmr_[a-zA-Z0-9_]*_t)', cantype):
                found = True 
                check_repeat(canid)
                #valid can type in dict 
                structlines = []
                #look at mapper json for data
                add_mapper_data(canid, cycletime, timeout, structlines)
                #find and format the correct struct in cantypes.h 
                matches = get_cantypes_data(cantype, structs)
                if matches: 
                    dlc = format_fields(cantype, matches, structlines, field_params)
                    structlines.insert(1, "DLC="+dlc)
                else:
                    found = False
                    #structlines.append("error with this struct in can_types.h")
                #add all formatted data to symbol file 
                if found:
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