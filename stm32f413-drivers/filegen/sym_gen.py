#TODO: add factor and units and figure out weird structs :( 
#TODO: what to do with the repeated structs (no id) hvc bmb
import re
import json

output = "stm32f413-drivers/filegen/symv1.sym"
lines = [] 

def id2hex(id):
    #uses canids_post.h to map id(from canid_type_map) to the hex number
    with open("stm32f413-drivers/filegen/canids_post.h", "r") as file:
        for line in file:
            if id in line:
                hex = re.search(r"0x[0-9A-Fa-f]+", line) 
                return hex.group()
            
def add_mapper_data(canid, cantype, cycletime, timeout):
        name = re.findall(r'cmr_can(\w+)_t',cantype)
        lines.append("["+name[0]+"]")
        if id2hex(canid):
            lines.append("ID:"+id2hex(canid)) 
        lines.append("CycleTime="+str(cycletime))
        lines.append("TimeOut="+str(timeout))

def get_cantypes_data(cantype):
    print(cantype)
    with open("stm32f413-drivers/CMR/include/CMR/can_types.h", "r") as structf: 
        #find all struct declarations
        structs = re.findall(r'typedef\s+struct\s*\{([\s\S]*?)\}\s*(cmr_can\w+)\s*;', structf.read())
        for fields, name in structs:
            if re.search(name, cantype): 
                #find struct declaration with the right can type 
                return re.findall(r'\b((?:u)?int\d+_t|float)\s+(\w+)\b', fields) 


def format_fields(matches):
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
        if size:
            lines.append("Var="+name+" "+vartype+ " " +str(atbit)+","+str(size))
            atbit+=int(size)
        else:
            lines.append("Issue with type of field")


def main():
    #need to find: can_name, can ID, DLC, Cycle Time, Timeout, fields 
    with open("canid_type_map.json", "r") as file: 
        json_obj = json.load(file)
        canids = json_obj['canid_to_info']
        for canid in canids:
            #go through each can id dict 
            cantype = canids[canid]['type']
            cycletime = canids[canid]['cycleTime']
            timeout = canids[canid]['timeOut']
            if re.fullmatch(r'(cmr_[a-zA-Z0-9_]*_t)', cantype):
                #valid can type in dict 
                print("found valid can type "+cantype+" starting search\n")
                add_mapper_data(canid, cantype, cycletime, timeout)
                print("successfully parsed mapper data\n")
                matches = get_cantypes_data(cantype)
                print("finished parsing fields in can_types.h\n")
                if matches:
                    format_fields(matches)
                    print("formatted fields\n")
                else:
                    lines.append("error with this struct in can_types.h")
                lines.append("\n")
                
                        
    
    with open("stm32f413-drivers/filegen/symv1.sym", "w") as file:
        file.write("\n".join(lines))
    print("done") 

if __name__ == "__main__":
    main()
