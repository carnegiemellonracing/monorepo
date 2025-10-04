import re
import json
import math 

output = "stm32f413-drivers/filegen/symv1.sym"
symlines = [] 
used_varnames = [] 
used_canids = [] #delete once canids fixed, shouldn't need 

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
            if bool(atIDs) and "CAN_ID" in line or "CANID" in line: #we should standardize can id names ... 
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

    with open("stm32f413-drivers/CMR/include/CMR/can_ids.h", "w") as f:
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
            return re.findall(r'\b((?:u)?int\d+_t|float|bool)\s+(\w+)\b', fields) 

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
    append_can_name = can_name[0].split("_")[0]+"_"+can_name[0].split("_")[1]
    if "HEARTBEAT" in canid:
        print("hi") 
        board = re.search(r'CMR_CANID_HEARTBEAT_(\w+)', canid); 
        boardname = board.group(1) 
        append_can_name = boardname+"_HEARTBEAT" 
    return append_can_name 


def format_bitpacking(canid, structname, structlines, atbit, vartype, enums): 
    for enumfields, name in enums:
        if name == structname: 
            packed_fields = re.findall(r'(?:CMR_CAN_)?(\w+)\s*=\s*\(?\s*(0x[\da-fA-F]+|\d+)\s*[a-zA-Z]*\s*\)?\s*(?:\(?\s*<<\s*(\d+)\s*\)?)?', enumfields) 
            for name, size, position in packed_fields: 
                if "0x" in size: 
                    size = int(size.split("0x")[1], 16) 
                if int(size) == 0: 
                    continue 
                realsize = int(math.log(int(size), 2)) + 1
                if not position: 
                    binary = bin(size)[2:] 
                    binary = binary[::-1]
                    position = atbit 
                    for i, c in enumerate(binary):
                        if c == '1':
                            position = i 
                            break
                name = check_repeat_varname(name) 
                append_can_name = create_prefix(name, canid) 
                if realsize == 1:
                    structlines.append("Var="+append_can_name+"_"+name.lower()+" bit "+str(atbit+int(position))+","+str(realsize)) 
                else: 
                    structlines.append("Var="+append_can_name+"_"+name.lower()+" "+vartype+" "+str(atbit+int(position))+","+str(realsize)) 
                #atbit+=realsize 


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
            elif vartype == 'bool':
                vartype = 'unsigned'
                size = 8 
        #check if field is bitpacked 
        if field_params and name in field_params:
            if 'enumstruct' in field_params[name]:
                flags = field_params[name]['enumstruct'].split()
                #not a heartbeat struct, normally bitpacked 
                if len(flags) == 1:
                    format_bitpacking(canid, flags[0], structlines, atbit, vartype, enums); 
                    atbit+=int(size) 
                    continue 
                #heartbeat logic 
                else:
                    board = re.search(r'CMR_CANID_HEARTBEAT_(\w+)', canid); 
                    boardname = board.group(1) 
                    for flag in flags:
                        if boardname in flag:
                            format_bitpacking(canid, flag, structlines, atbit, vartype, enums); 
                            atbit+=int(size)*2 #lowkey hardcoded but I think heartbeat is the only array 
                            break 
                    continue 
        #add in field if not bitpacked 
        if size: 
            name = check_repeat_varname(name) 
            append_can_name = create_prefix(name, canid) 
            appendstr = "Var="+append_can_name+"_"+name+" "+vartype+ " " +str(atbit)+","+str(size)
            # Add field-specific parameters if available
            if field_params and name in field_params:
                appendstr += format_field_params(field_params[name])
            structlines.append(appendstr)
            atbit+=int(size)
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
    with open("stm32f413-drivers/filegen/can_types_new.h", "r") as structf: 
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
    with open("stm32f413-drivers/filegen/symv1.sym", "w") as file:
        file.write("\n".join(symlines)) 
    print("done") 

if __name__ == "__main__":
    main()