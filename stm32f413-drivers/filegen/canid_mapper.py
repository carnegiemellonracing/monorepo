import os
import json
import re
from collections import defaultdict

#Can run file with "python canid_mapper.py" in terminal

ROOT_DIR = "."
CANID_PREFIXES = ["CMR_CANID_", "CAN_ID_"]
CANRX_PREFIXES = ["CANRX_"]
OUTPUT_FILE = "stm32f413-drivers/filegen/canid_type_map.json"

#Handle both 4 and 5 argument canTX calls
CANTX_PATTERN_5_ARGS = re.compile(
    r'canTX\s*\(\s*([^,]+)\s*,\s*([^,]+)\s*,\s*([^,]+)\s*,\s*sizeof\s*\(\s*([^)]+)\s*\)\s*(?:,\s*[^)]+)?\s*\)',
    re.MULTILINE | re.DOTALL
)

CANTX_PATTERN_4_ARGS = re.compile(
    r'canTX\s*\(\s*([^,]+)\s*,\s*([^,]+)\s*,\s*sizeof\s*\(\s*([^)]+)\s*\)\s*(?:,\s*[^)]+)?\s*\)',
    re.MULTILINE | re.DOTALL
)

#Pattern to find variable declarations of cmr_*_t types (including pointers)
VAR_DECLARATION_PATTERN = re.compile(
    r'(?:const\s+)?(cmr_[a-zA-Z0-9_]*_t)\s*\*?\s*([a-zA-Z0-9_]+)\s*(?:=|;)',
    re.MULTILINE
)

#Pattern to find struct member declarations like "cmr_type_t memberName;" inside structs
STRUCT_MEMBER_PATTERN = re.compile(
    r'typedef\s+struct\s*\{[^}]*?(cmr_[a-zA-Z0-9_]*_t)\s+([a-zA-Z0-9_]+)\s*;[^}]*\}\s*([a-zA-Z0-9_]+)\s*;',
    re.MULTILINE | re.DOTALL
)

#More specific pattern for struct members
STRUCT_CONTENT_PATTERN = re.compile(
    r'typedef\s+struct\s*\{([^}]+)\}\s*([a-zA-Z0-9_]+)\s*;',
    re.MULTILINE | re.DOTALL
)

#Pattern to find individual member declarations within struct content
MEMBER_DECLARATION_PATTERN = re.compile(
    r'(cmr_[a-zA-Z0-9_]*_t)\s+([a-zA-Z0-9_]+)\s*;',
    re.MULTILINE
)

#Pattern to find numerical constant definitions; includes TickType_t
CONST_DEFINITION_PATTERN = re.compile(
    r'(?:#define\s+([a-zA-Z0-9_]+)\s+([0-9]+))|(?:(?:static\s+)?(?:const\s+)?(?:int|uint32_t|uint16_t|uint8_t|unsigned|long|TickType_t)\s+([a-zA-Z0-9_]+)\s*=\s*([0-9]+))',
    re.MULTILINE
)

#Pattern to find CANRX array definitions and their entries
CANRX_ARRAY_PATTERN = re.compile(
    r'cmr_canRXMeta_t\s+([a-zA-Z0-9_]+)\s*\[[^\]]+\]\s*=\s*\{([^}]+)\}',
    re.MULTILINE | re.DOTALL
)

#Better pattern to find individual CANRX entries within arrays
CANRX_ENTRY_PATTERN = re.compile(
    r'\[\s*([A-Z_]+)\s*\]\s*=\s*\{[^}]*\.canID\s*=\s*([A-Z_]+)[^}]*(?:\.timeoutError_ms\s*=\s*([0-9]+))?[^}]*\}',
    re.MULTILINE | re.DOTALL
)

#Comprehensive getPayload patterns - covers multiple variations
GETPAYLOAD_PATTERNS = [
    #pattern 1: Direct assignment with type declaration
    re.compile(
        r'(?:volatile\s+)?(?:const\s+)?(cmr_\w+_t)\s*\*\s*\w+\s*=\s*(?:\w*[gG]et[pP]ayload|getPayload)\s*\(\s*(CANRX_\w+)\s*\)',
        re.IGNORECASE | re.MULTILINE
    ),
    #pattern 2: Direct assignment without volatile/const
    re.compile(
        r'(cmr_\w+_t)\s*\*\s*\w+\s*=\s*(?:\w*[gG]et[pP]ayload|getPayload)\s*\(\s*(CANRX_\w+)\s*\)',
        re.IGNORECASE | re.MULTILINE
    ),
    #pattern 3: Type casting with getPayload
    re.compile(
        r'\(\s*(?:volatile\s+)?(?:const\s+)?(cmr_\w+_t)\s*\*\s*\)\s*(?:\w*[gG]et[pP]ayload|getPayload)\s*\(\s*(CANRX_\w+)\s*\)',
        re.IGNORECASE | re.MULTILINE
    ),
    #pattern 4: Assignment with type casting
    re.compile(
        r'(?:volatile\s+)?(?:const\s+)?(cmr_\w+_t)\s*\*\s*\w+\s*=\s*\(\s*(?:volatile\s+)?(?:const\s+)?cmr_\w+_t\s*\*\s*\)\s*(?:\w*[gG]et[pP]ayload|getPayload)\s*\(\s*(CANRX_\w+)\s*\)',
        re.IGNORECASE | re.MULTILINE
    )
]

#Pattern for indirect payload access (for pattern 5 from original script)
INDIRECT_PAYLOAD_PATTERN = re.compile(
    r'(?:volatile\s+)?(?:const\s+)?(cmr_\w+_t)\s*\*\s*\w+\s*=\s*\([^)]+\)\s*\w+->payload',
    re.IGNORECASE | re.MULTILINE
)

META_PATTERN = re.compile(
    r'cmr_canRXMeta_t\s*\*\s*(\w+)\s*=\s*canRXMeta\s*\+\s*(CANRX_\w+)',
    re.IGNORECASE | re.MULTILINE
)

#Better CANRX array pattern for better matching
CANRX_DEFINITION_PATTERN = re.compile(
    r'\[(\w*CANRX_\w+)\]\s*=\s*\{[^}]*?\.canID\s*=\s*((?:CMR_CANID_|CAN_ID_)\w+)',
    re.IGNORECASE | re.MULTILINE | re.DOTALL
)

def is_canid(arg):
    """Check if an argument matches any of the CAN ID prefixes"""
    arg = arg.strip()
    return any(arg.startswith(prefix) for prefix in CANID_PREFIXES)

def is_canrx(arg):
    """Check if an argument matches any of the CANRX prefixes"""
    arg = arg.strip()
    return any(arg.startswith(prefix) for prefix in CANRX_PREFIXES)

def extract_variable_types(content):
    """Extract variable name to type mappings from file content"""
    var_to_type = {}
    
    #Find regular variable declarations (including const and pointer variants)
    matches = VAR_DECLARATION_PATTERN.findall(content)
    for type_name, var_name in matches:
        var_to_type[var_name] = type_name
    
    return var_to_type

def extract_struct_member_types(content):
    """Extract struct_name.member_name to type mappings from file content"""
    member_to_type = {}
    
    struct_matches = STRUCT_CONTENT_PATTERN.findall(content)
    for struct_content, struct_name in struct_matches:
        #Find all cmr_*_t members within this struct
        member_matches = MEMBER_DECLARATION_PATTERN.findall(struct_content)
        for member_type, member_name in member_matches:
            #Store as both struct_name.member_name and just member_name
            full_member_name = f"{struct_name}.{member_name}"
            member_to_type[full_member_name] = member_type
            #Also store just the member name for cases where struct name is variable
            member_to_type[member_name] = member_type
    
    return member_to_type

def extract_constant_definitions(content):
    """Extract constant definitions from file content"""
    constants = {}
    
    #Find #define and variable constant definitions
    matches = CONST_DEFINITION_PATTERN.findall(content)
    for define_name, define_value, var_name, var_value in matches:
        if define_name and define_value:
            try:
                constants[define_name] = int(define_value)
            except ValueError:
                constants[define_name] = define_value
        elif var_name and var_value:
            try:
                constants[var_name] = int(var_value)
            except ValueError:
                constants[var_name] = var_value
    
    return constants

def extract_canrx_mappings(content):
    """Extract CANRX to CAN ID mappings and timeout info from array definitions"""
    canrx_to_canid = {}
    canrx_to_timeout = {}
    
    if 'cmr_canRXMeta_t' not in content:
        return canrx_to_canid, canrx_to_timeout
    
    #Multi-line pattern matching for better coverage
    lines = content.split('\n')
    current_canrx = None
    
    for i, line in enumerate(lines):
        #Look for CANRX array entry start
        canrx_match = re.search(r'\[\s*(CANRX_\w+)\s*\]\s*=\s*\{', line, re.IGNORECASE)
        if canrx_match:
            current_canrx = canrx_match.group(1)
            
            #Look ahead up to 20 lines for the closing brace
            for j in range(i, min(i + 20, len(lines))):
                #Look for .canID assignment
                canid_match = re.search(r'\.canID\s*=\s*((?:CMR_CANID_|CAN_ID_)\w+)', lines[j], re.IGNORECASE)
                if canid_match:
                    can_id = canid_match.group(1)
                    if is_canrx(current_canrx) and is_canid(can_id):
                        canrx_to_canid[current_canrx] = can_id
                
                timeout_match = re.search(r'\.timeoutError_ms\s*=\s*([0-9]+)', lines[j], re.IGNORECASE)
                if timeout_match:
                    try:
                        timeout_val = int(timeout_match.group(1))
                        if current_canrx and is_canrx(current_canrx):
                            canrx_to_timeout[current_canrx] = timeout_val
                    except ValueError:
                        pass
                        
                if '}' in lines[j] and j > i:
                    break
    
    #try the original patterns as backup
    array_matches = CANRX_ARRAY_PATTERN.findall(content)
    for array_name, array_content in array_matches:
        #find individual entries within the array
        entry_matches = CANRX_ENTRY_PATTERN.findall(array_content)
        for match in entry_matches:
            if len(match) == 3:
                canrx_key, canid, timeout_error_ms = match
                canrx_key = canrx_key.strip()
                canid = canid.strip()
                timeout_error_ms = timeout_error_ms.strip()
                
                if is_canrx(canrx_key) and is_canid(canid):
                    if canrx_key not in canrx_to_canid:
                        canrx_to_canid[canrx_key] = canid
                    if timeout_error_ms and canrx_key not in canrx_to_timeout:
                        try:
                            timeout_val = int(timeout_error_ms)
                            canrx_to_timeout[canrx_key] = timeout_val
                        except ValueError:
                            canrx_to_timeout[canrx_key] = timeout_error_ms
    
    #better pattern for broader matching
    enhanced_matches = CANRX_DEFINITION_PATTERN.findall(content)
    for canrx_name, can_id in enhanced_matches:
        canrx_name = canrx_name.strip()
        can_id = can_id.strip()
        if is_canrx(canrx_name) and is_canid(can_id):
            if canrx_name not in canrx_to_canid:
                canrx_to_canid[canrx_name] = can_id
    
    return canrx_to_canid, canrx_to_timeout

def extract_canrx_type_mappings(content):
    """Extract CANRX to type mappings from *getPayload* calls using comprehensive patterns"""
    canrx_to_type = {}
    
    if 'getPayload' not in content and 'GetPayload' not in content:
        return canrx_to_type
    
    #Apply all getPayload patterns
    for pattern in GETPAYLOAD_PATTERNS:
        matches = pattern.findall(content)
        for match in matches:
            if len(match) == 2:
                type_name, canrx_key = match
                canrx_key = canrx_key.strip()
                if is_canrx(canrx_key):
                    canrx_to_type[canrx_key] = type_name
    
    #Handle indirect payload access
    indirect_matches = INDIRECT_PAYLOAD_PATTERN.findall(content)
    if indirect_matches:
        meta_matches = META_PATTERN.findall(content)
        meta_to_canrx = {}
        for meta_var, canrx_name in meta_matches:
            meta_to_canrx[meta_var] = canrx_name
        
        #Match indirect payload access with meta variables
        for type_name in indirect_matches:
            pass
    
    return canrx_to_type

def parse_sizeof_argument(sizeof_arg):
    """Parse sizeof argument to extract the key for type lookup"""
    sizeof_arg = sizeof_arg.strip()
    
    #Handle different sizeof patterns:
    # sizeof(*variable) -> variable
    # sizeof(variable) -> variable  
    # sizeof(struct->member) -> member
    # sizeof(variable->member) -> member
    
    #Remove leading * if present
    if sizeof_arg.startswith('*'):
        sizeof_arg = sizeof_arg[1:].strip()
    
    #Handle struct->member or variable->member
    if '->' in sizeof_arg:
        parts = sizeof_arg.split('->')
        if len(parts) >= 2:
            return parts[-1].strip()
    
    #Handle struct.member
    if '.' in sizeof_arg:
        parts = sizeof_arg.split('.')
        if len(parts) >= 2:
            return parts[-1].strip()
    
    return sizeof_arg

def resolve_constant_value(constant_name, local_constants, global_constants):
    """Resolve a constant name to its numerical value"""
    constant_name = constant_name.strip()
    
    #Try to parse as int first (in case it's alr a number)
    try:
        return int(constant_name)
    except ValueError:
        pass
    
    #Check local constants first
    if constant_name in local_constants:
        value = local_constants[constant_name]
        if isinstance(value, int):
            return value
        try:
            return int(value)
        except (ValueError, TypeError):
            return value
    
    #check global constants
    if constant_name in global_constants:
        value = global_constants[constant_name]
        if isinstance(value, int):
            return value
        try:
            return int(value)
        except (ValueError, TypeError):
            return value
    
    return constant_name

def resolve_sizeof_type(sizeof_arg, local_var_types, global_var_types, local_struct_members, global_struct_members):
    """Resolve the type from sizeof argument, prioritizing local then global mappings"""
    lookup_key = parse_sizeof_argument(sizeof_arg)
    
    if lookup_key.startswith('cmr_') and lookup_key.endswith('_t'):
        return lookup_key, True
    
    if lookup_key in local_var_types:
        return local_var_types[lookup_key], True
    
    if lookup_key in local_struct_members:
        return local_struct_members[lookup_key], True
    
    if lookup_key in global_var_types:
        return global_var_types[lookup_key], True
    
    if lookup_key in global_struct_members:
        return global_struct_members[lookup_key], True
    
    return sizeof_arg, False

def calculate_cycle_time_and_timeout(cycle_time_value):
    """Calculate cycle time and timeout (timeout = 5 * cycle_time)"""
    if isinstance(cycle_time_value, int):
        time_out = 5 * cycle_time_value
    else:
        time_out = f"5*{cycle_time_value}"
    
    return cycle_time_value, time_out

def clean_canid(raw_canid):
    """Remove bit-shifts, additions, and offsets from CAN ID expressions."""
    #Keep only the first token that looks like a CAN ID
    return re.split(r'\s|\+|\-', raw_canid.strip())[0]

def extract_canids_from_file(filepath, global_var_types, global_struct_members, global_constants, global_canrx_mappings, global_canrx_timeouts, global_canrx_types):
    """Extract CAN IDs, types, cycleTime, and timeout from a single file"""
    try:
        with open(filepath, 'r', encoding='utf-8', errors='ignore') as f:
            content = f.read()
        
        local_var_types = extract_variable_types(content)
        local_struct_members = extract_struct_member_types(content)
        local_constants = extract_constant_definitions(content)
        local_canrx_mappings, local_canrx_timeouts = extract_canrx_mappings(content)
        local_canrx_types = extract_canrx_type_mappings(content)
        
        result = {}
        
        #Find all canTX calls with 5 arguments
        CAN_CALL_5_ARGS_FULL = re.compile(
            r'canTX\s*\(\s*([^,]+)\s*,\s*([^,]+)\s*,\s*([^,]+)\s*,\s*sizeof\s*\(\s*([^)]+)\s*\)\s*,\s*([^)]+)\s*\)',
            re.MULTILINE | re.DOTALL
        )
        
        for match in CAN_CALL_5_ARGS_FULL.finditer(content):
            arg1, arg2, arg3, sizeof_arg, period = match.groups()
            arg1 = arg1.strip()
            arg2 = arg2.strip()
            arg3 = arg3.strip()
            sizeof_arg = sizeof_arg.strip()
            period = period.strip()

            canid = None
            if is_canid(arg1):
                canid = clean_canid(arg1)
            elif is_canid(arg2):
                canid = clean_canid(arg2)
            elif is_canid(arg3):
                canid = clean_canid(arg3)

            if canid:
                resolved_type, _ = resolve_sizeof_type(
                    sizeof_arg, local_var_types, global_var_types, local_struct_members, global_struct_members
                )
                
                cycle_time = resolve_constant_value(period, local_constants, global_constants)
                cycle_time, time_out = calculate_cycle_time_and_timeout(cycle_time)
                
                result[canid] = {
                    "type": resolved_type,
                    "cycleTime": cycle_time,
                    "timeOut": time_out,
                    "source": "canTX"
                }

        #Find all canTX calls with 4 arguments
        CAN_CALL_4_ARGS_FULL = re.compile(
            r'canTX\s*\(\s*([^,]+)\s*,\s*([^,]+)\s*,\s*sizeof\s*\(\s*([^)]+)\s*\)\s*,\s*([^)]+)\s*\)',
            re.MULTILINE | re.DOTALL
        )
        
        for match in CAN_CALL_4_ARGS_FULL.finditer(content):
            arg1, arg2, sizeof_arg, period = match.groups()
            arg1 = arg1.strip()
            arg2 = arg2.strip()
            sizeof_arg = sizeof_arg.strip()
            period = period.strip()

            canid = None
            if is_canid(arg1):
                canid = clean_canid(arg1)
            elif is_canid(arg2):
                canid = clean_canid(arg2)
            
            if canid and canid not in result:
                resolved_type, _ = resolve_sizeof_type(
                    sizeof_arg, local_var_types, global_var_types, local_struct_members, global_struct_members
                )
                
                cycle_time = resolve_constant_value(period, local_constants, global_constants)
                cycle_time, time_out = calculate_cycle_time_and_timeout(cycle_time)
                
                result[canid] = {
                    "type": resolved_type,
                    "cycleTime": cycle_time,
                    "timeOut": time_out,
                    "source": "canTX"
                }

        #Handle CANRX mappings - merge local and global mappings
        all_canrx_mappings = {**global_canrx_mappings, **local_canrx_mappings}
        all_canrx_timeouts = {**global_canrx_timeouts, **local_canrx_timeouts}
        all_canrx_types = {**global_canrx_types, **local_canrx_types}
        
        #For each CANRX w/ type mapping, find its CAN ID and map type to ID
        for canrx_key, canrx_type in all_canrx_types.items():
            if canrx_key in all_canrx_mappings:
                canid = all_canrx_mappings[canrx_key]
                
                #Check if we already have this CAN ID from canTX
                if canid in result:
                    continue
                else:
                    #No canTX info, use CANRX info
                    timeout_val = all_canrx_timeouts.get(canrx_key, "N/A")
                    if isinstance(timeout_val, int) and timeout_val > 0:
                        cycle_time = timeout_val / 5
                    else:
                        cycle_time = "N/A"
                    
                    result[canid] = {
                        "type": canrx_type,
                        "cycleTime": cycle_time,
                        "timeOut": timeout_val,
                        "source": "canRX",
                        "canrx_key": canrx_key
                    }

        return result
    except Exception as e:
        return {}

def find_all_c_files(root_dir):
    """Find all C/C++ files in the directory tree"""
    c_files = []
    for root, dirs, files in os.walk(root_dir):
        for filename in files:
            if filename.endswith(('.c', '.cpp', '.cc', '.cxx', '.h', '.hpp', '.hh', '.hxx')):
                c_files.append(os.path.join(root, filename))
    return c_files

def build_global_mappings(c_files):
    """Build global type mappings from all files"""
    global_var_types = {}
    global_struct_members = {}
    global_constants = {}
    global_canrx_mappings = {}
    global_canrx_timeouts = {}
    global_canrx_types = {}
    
    for filepath in c_files:
        try:
            with open(filepath, 'r', encoding='utf-8', errors='ignore') as f:
                content = f.read()
            
            #Skip files without relevant content
            if not any(prefix in content for prefix in CANID_PREFIXES + CANRX_PREFIXES):
                continue
                
            var_types = extract_variable_types(content)
            global_var_types.update(var_types)
            
            struct_members = extract_struct_member_types(content)
            global_struct_members.update(struct_members)
            
            constants = extract_constant_definitions(content)
            global_constants.update(constants)
            
            canrx_mappings, canrx_timeouts = extract_canrx_mappings(content)
            global_canrx_mappings.update(canrx_mappings)
            global_canrx_timeouts.update(canrx_timeouts)
            
            canrx_types = extract_canrx_type_mappings(content)
            global_canrx_types.update(canrx_types)
                
        except Exception as e:
            continue
    
    return global_var_types, global_struct_members, global_constants, global_canrx_mappings, global_canrx_timeouts, global_canrx_types

def main():
    c_files = find_all_c_files(ROOT_DIR)
    
    if not c_files:
        print("No C/C++ files found!")
        return
    
    #Build global mappings
    global_var_types, global_struct_members, global_constants, global_canrx_mappings, global_canrx_timeouts, global_canrx_types = build_global_mappings(c_files)
    #Extract CAN IDs from all files
    canid_to_info = {}
    files_with_matches = 0
    
    for filepath in c_files:
        result = extract_canids_from_file(filepath, global_var_types, global_struct_members, global_constants, global_canrx_mappings, global_canrx_timeouts, global_canrx_types)
        
        if result:
            files_with_matches += 1
        
        #Update result, but keep existing canTX info if present
        for canid, info in result.items():
            if canid not in canid_to_info:
                canid_to_info[canid] = info
            elif info.get("source") == "canTX" and canid_to_info[canid].get("source") == "canRX":
                #Replace canRX info with canTX info (canTX has priority)
                canid_to_info[canid] = info
    
    #Clean up the output - remove source and canrx_key fields for final output
    cleaned_canid_to_info = {}
    for canid, info in canid_to_info.items():
        cleaned_info = {
            "type": info["type"],
            "cycleTime": info["cycleTime"],
            "timeOut": info["timeOut"]
        }
        cleaned_canid_to_info[canid] = cleaned_info
    
    #Create output directory if it doesn't exist
    output_dir = os.path.dirname(OUTPUT_FILE)
    if output_dir and not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    output_data = {
        "canid_to_info": cleaned_canid_to_info,
        "summary": {
            "files_processed": len(c_files),
            "files_with_matches": files_with_matches,
            "total_canids": len(cleaned_canid_to_info),
            "prefixes_used": CANID_PREFIXES,
            "canrx_prefixes_used": CANRX_PREFIXES,
            "global_mappings": {
                "variable_types": len(global_var_types),
                "struct_members": len(global_struct_members),
                "constants": len(global_constants),
                "canrx_mappings": len(global_canrx_mappings),
                "canrx_timeouts": len(global_canrx_timeouts),
                "canrx_types": len(global_canrx_types)
            }
        }
    }
    
    with open(OUTPUT_FILE, 'w') as f:
        json.dump(output_data, f, indent=4, sort_keys=True)
    
    #Count canTX vs canRX mappings for summary
    cantx_count = 0
    canrx_count = 0
    
    for canid, info in cleaned_canid_to_info.items():
        cycle_time = info.get('cycleTime', 'N/A')
        timeout = info.get('timeOut', 'N/A')
        
        #If we have numeric cycleTime and calculated timeout (5*cycleTime), it's likely from canTX
        if (isinstance(cycle_time, (int, float)) and 
            isinstance(timeout, (int, float)) and 
            abs(timeout - 5 * cycle_time) < 0.01):  #Allow for floating point precision
            cantx_count += 1
        else:
            canrx_count += 1
    
    #Summary output
    print(f"Processed {len(c_files)} files, found {len(cleaned_canid_to_info)} CAN IDs ({cantx_count} TX, {canrx_count} RX) -> {OUTPUT_FILE}")

if __name__ == "__main__":
    main()