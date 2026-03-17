import re
import json
import math 
import sys

output = "stm32f413-drivers/PCAN/CMR 26x.sym"
symlines = [] 
used_varnames = [] 
used_canids = [] #delete once canids fixed, shouldn't need 
bitfields = False 

def numbercanids():
    canidfile = "stm32f413-drivers/CMR/include/CMR/can_ids.h"
    atIDs = False
    num = 0
    lines = []

    with open(canidfile, "r") as f:
        for line in f:
            # track enum scope
            if "enum" in line:
                atIDs = True
            if "}" in line:
                atIDs = False

            # only operate inside enum, and only on relevant lines
            if atIDs and ("CAN_ID" in line or "CANID" in line):
                # If line alr has explicit assignment, dont touch it.
                if "=" in line:
                    hexm = re.search(r"0x[0-9A-Fa-f]+", line)
                    if hexm:
                        num = int(hexm.group(), 16)
                    lines.append(line)
                    continue

                # No explicit assignment so auto-number it
                base = line.split(",")[0]
                base = base.rstrip()

                num += 1
                if "+ CMR" in line:
                    value = num + 928
                else:
                    value = num

                line = f"{base} = {hex(value)},\n"

            lines.append(line)

    with open(canidfile, "w") as f:
        f.write("".join(lines))

def id2hex(id):
    #uses can_ids.h to map id(from canid_type_map) to the hex number
    numbercanids() 
    with open("stm32f413-drivers/CMR/include/CMR/can_ids.h", "r") as file:
        for line in file:
            if id in line:
                hex = re.search(r"0x[0-9A-Fa-f]+", line) 
                return hex.group().split("x")[1]

def add_mapper_data(canid, cycletime, timeout, structlines):
        name = re.findall(r'CMR_CANID_(\w+)',canid) 
        if "HEARTBEAT" not in name[0]: 
            structlines.append("["+name[0]+"]")
        else:
            board = re.search(r'CMR_CANID_HEARTBEAT_(\w+)', canid); 
            boardname = board.group(1) 
            structlines.append("["+boardname+"_HEARTBEAT]") 
        if id2hex(name[0]):
            structlines.append("ID="+id2hex(name[0])+"h")  
        structlines.append("CycleTime="+str(cycletime))
        structlines.append("TimeOut="+str(timeout))

def get_cantypes_data(cantype, structs):
    for fields, name in structs:
        if re.search(name, cantype): 
            #find struct declaration with the right can type 
            return re.findall(r'\b((?:u)?int\d+_t|float|bool|(?:unsigned|signed)?\s*char)\s+([A-Za-z_]\w*[^;]*)', fields) 

def check_repeat_varname(name):
    repeat_num = 0
    for varname in used_varnames:
        if name == varname:
            repeat_num+=1
    used_varnames.append(name)
    if repeat_num!=0: 
        return name+str(repeat_num)
    return name 

def create_prefix(name, canid):
    can_name = re.findall(r'CMR_CANID_(\w+)',canid) 
    append_can_name = can_name[0].split("_")[1]+"_"+name 
    if "HEARTBEAT" in canid: 
        board = re.search(r'CMR_CANID_HEARTBEAT_(\w+)', canid); 
        boardname = board.group(1) 
        append_can_name = boardname+"_HEARTBEAT_"+name 
    if len(append_can_name) >= 30:
        #print("too long")
        return name
    return append_can_name 


def format_bitpacking(canid, structname, structlines, atbit, vartype, enums):
    #found = False
    for enumfields, cantype_name in enums:
        if cantype_name == structname: 
            #found = True
            packed_fields = re.findall(r'(?:CMR_CAN_)?(\w+)\s*=\s*\(?\s*(0x[\da-fA-F]+|\d+)\s*[a-zA-Z]*\s*\)?\s*(?:\(?\s*<<\s*(\d+)\s*\)?)?', enumfields) 
            for name, size, position in packed_fields: 
                if "0x" in size: 
                    size = int(size.split("0x")[1], 16) #convert to decimal value in int 
                if int(size) == 0: #why would we need this 
                    continue 
                #find number of bits (realsize) 
                realsize = 0 
                binary = bin(int(size))[2:] 
                binary = binary[::-1] 
                #position = atbit 
                for i, c in enumerate(binary):
                    if c == '1':
                        if not position: #also find position of least significant one if not given 
                            position = i 
                        realsize += 1 
                append_can_name = create_prefix(name, canid) 
                append_can_name = check_repeat_varname(append_can_name)
                if realsize == 1:
                    structlines.append("Var="+append_can_name+" bit "+str(atbit+int(position))+","+str(realsize)) 
                else: 
                    structlines.append("Var="+append_can_name+" "+vartype+" "+str(atbit+int(position))+","+str(realsize)) 
                #atbit+=realsize 
            return 

    #if not found:
    print(f"BUILD ERROR: Referencing a flag enum that doesn't exist for CAN ID {canid}!")
    sys.exit(1) 


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

def format_fields(canid, matches, structlines, enums, field_params=None):
    atbit = 0
    size = None
    bitfield_size = None 
    prev_size = 0
    size_change_start = 0 
    prev_bitfield_size = 0 

    for vartype, name in matches: 
        if ':' in name:
            bitfield_size = int(name.split(':')[-1]) 
            name = name.split(':')[0]
            bitfields = True 
        else: 
            bitfields = False 
        findsize = re.search(r'\d+', vartype)
        if findsize:
            size = int(findsize.group())
            if vartype.startswith('u'):
                vartype = 'unsigned'
            else: 
                vartype = 'signed' 
        else:
            if " " in vartype: #two word variable declaration 
                vartype, second_word = vartype.split(" ")
                if second_word == "char":
                    size = 8
                elif second_word == "short":
                    size = 16
                elif second_word == "int":
                    size = 32
                elif second_word == "long":
                    size = 64 
                else:
                    print(f"ERROR: can't find size for {vartype} {name}")
                    continue
            else:
                if vartype == 'float': 
                    size = 32 
                elif vartype == 'bool':
                    vartype = 'unsigned'
                    size = 8 
        if size != prev_size: #for bitfield overflow calculations 
            size_change_start = atbit 
        #check if field is bitpacked 
        if "[" in name:
            name = name.split("[")[0]
        if field_params and name in field_params:
            if 'enumstruct' in field_params[name]:
                flags = field_params[name]['enumstruct'].split()
                #not a heartbeat struct, normally bitpacked 
                if len(flags) == 1:
                    format_bitpacking(canid, flags[0], structlines, atbit, vartype, enums); 
                    atbit += int(size) 
                    continue 
                #heartbeat logic 
                else:
                    board = re.search(r'CMR_CANID_HEARTBEAT_(\w+)', canid); 
                    boardname = board.group(1) 
                    for flag in flags:
                        if boardname in flag:
                            format_bitpacking(canid, flag, structlines, atbit, vartype, enums); 
                            break 
                    atbit+=int(size)*2 #lowkey hardcoded but I think heartbeat is the only array 
                    continue 
        #add in field if not bitpacked 
        if size: 
            append_can_name = create_prefix(name, canid) 
            append_can_name = check_repeat_varname(append_can_name)
            if bitfields:
                new_atbit = atbit + int(bitfield_size) 
                if math.ceil((atbit-size_change_start)/size) < math.ceil((new_atbit-size_change_start)/size) and (atbit-size_change_start) % size != 0: #if bitfield overflows past var size 
                    print(f"WARNING: {name} is not aligned to variable size, check bit fields!")
                    temp_atbit = atbit - prev_bitfield_size + prev_size 
                    appendstr = "Var="+append_can_name+" "+vartype+ " " +str(temp_atbit)+","+str(bitfield_size) 
                    atbit = temp_atbit + int(bitfield_size) 
                else: #bit fields without overflow 
                    appendstr = "Var="+append_can_name+" "+vartype+ " " +str(atbit)+","+str(bitfield_size)
                    atbit = new_atbit
                prev_bitfield_size = bitfield_size 
                prev_size = size 
            else: #normal case 
                new_atbit = atbit + int(size) 
                appendstr = "Var="+append_can_name+" "+vartype+ " " +str(atbit)+","+str(size)
                atbit = new_atbit
            # Add field-specific parameters if available
            if field_params and name in field_params:
                appendstr += format_field_params(field_params[name])
            structlines.append(appendstr) 
        else: 
            structlines.append("Issue with type of field")
    return str(int(atbit/8))

def extract_field_params(canid_data):
    """Extract field-specific parameters from the canid data"""
    field_params = {}
    for key, value in canid_data.items():
        if key not in ['cycleTime', 'timeOut', 'type'] and isinstance(value, dict):
            field_params[key] = value
    return field_params if field_params else None

def extract_numeric_value(value):
    """Extract numeric value from string that might contain variable names or comments"""
    if isinstance(value, (int, float)):
        return int(value)
    
    # Convert to string if not already
    value_str = str(value)
    
    # Look for numeric values in the string
    numeric_match = re.search(r'\b(\d+)\b', value_str)
    if numeric_match:
        return int(numeric_match.group(1))
    
    # If no numeric value found, return 0 as default
    print(f"Warning: Could not extract numeric value from '{value_str}', using 0")
    return 0

def main():
    with open("stm32f413-drivers/CMR/include/CMR/can_types.h", "r") as structf: 
        #find all struct declarations
        cantypes = structf.read()
        structs = re.findall(r'typedef\s+struct\s*\{([\s\S]*?)\}\s*(cmr_can\w+)\s*;', cantypes)
        enums = re.findall(r'typedef\s+enum\s*\{([\s\S]*?)\}\s*(cmr_can\w+)\s*;', cantypes)
    with open("stm32f413-drivers/filegen/canid_type_map.json", "r") as file: 
        json_obj = json.load(file)
        canids = json_obj['canid_to_info']
        for canid in canids: 
            #go through each can id dict 
            canid_data = canids[canid]
            cantype = canid_data['type']
            cycletime = extract_numeric_value(canid_data['cycleTime'])  # Extract and convert to integer
            timeout = extract_numeric_value(canid_data['timeOut'])      # Extract and convert to integer
            
            # Extract field-specific parameters
            field_params = extract_field_params(canid_data)
            
            if re.fullmatch(r'(cmr_[a-zA-Z0-9_]*_t)', cantype):
                #check repeat, delete once canids fixed 
                id_num = id2hex(canid)
                if id_num in used_canids:
                    continue
                used_canids.append(id_num) 

                found = True 
                #check_repeat(canid)
                #valid can type in dict 
                structlines = []
                #look at mapper json for data
                add_mapper_data(canid, cycletime, timeout, structlines)
                #find and format the correct struct in cantypes.h 
                matches = get_cantypes_data(cantype, structs)
                if matches: 
                    dlc = format_fields(canid, matches, structlines, enums, field_params) 
                    structlines.insert(2, "DLC="+dlc)
                else:
                    found = False
                    #structlines.append("error with this struct in can_types.h")
                #add all formatted data to symbol file 
                if found:
                    for line in structlines:
                        symlines.append(line) 
                    symlines.append("\n") 
    #write into symbol file 
    with open("stm32f413-drivers/PCAN/CMR 26x.sym", "w") as file:
        file.write("\n".join(symlines)) 
    print("done writing symbols") 

if __name__ == "__main__":
    main()