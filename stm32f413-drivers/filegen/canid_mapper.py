import os
import json
import re
from collections import defaultdict

#Can run file with "python canid_mapper.py" in terminal

ROOT_DIR = "."
CANID_PREFIXES = ["CMR_CANID_", "CAN_ID_"]
CANRX_PREFIXES = ["CANRX_"]
OUTPUT_FILE = "stm32f413-drivers/filegen/canid_type_map.json"

CANTX_PATTERN_5_ARGS = re.compile(
    r'canTX\s*\(\s*([^,]+)\s*,\s*([^,]+)\s*,\s*([^,]+)\s*,\s*sizeof\s*\(\s*([^)]+)\s*\)\s*(?:,\s*[^)]+)?\s*\)',
    re.MULTILINE | re.DOTALL
)

CANTX_PATTERN_4_ARGS = re.compile(
    r'canTX\s*\(\s*([^,]+)\s*,\s*([^,]+)\s*,\s*sizeof\s*\(\s*([^)]+)\s*\)\s*(?:,\s*[^)]+)?\s*\)',
    re.MULTILINE | re.DOTALL
)

VAR_DECLARATION_PATTERN = re.compile(
    r'(?:const\s+)?(cmr_[a-zA-Z0-9_]*_t)\s*\*?\s*([a-zA-Z0-9_]+)\s*(?:=|;)',
    re.MULTILINE
)

STRUCT_MEMBER_PATTERN = re.compile(
    r'typedef\s+struct\s*\{[^}]*?(cmr_[a-zA-Z0-9_]*_t)\s+([a-zA-Z0-9_]+)\s*;[^}]*\}\s*([a-zA-Z0-9_]+)\s*;',
    re.MULTILINE | re.DOTALL
)

STRUCT_CONTENT_PATTERN = re.compile(
    r'typedef\s+struct\s*\{([^}]+)\}\s*([a-zA-Z0-9_]+)\s*;',
    re.MULTILINE | re.DOTALL
)

MEMBER_DECLARATION_PATTERN = re.compile(
    r'(cmr_[a-zA-Z0-9_]*_t)\s+([a-zA-Z0-9_]+)\s*;',
    re.MULTILINE
)

CONST_DEFINITION_PATTERN = re.compile(
    r'(?:#define\s+([a-zA-Z0-9_]+)\s+([0-9]+))|(?:(?:static\s+)?(?:const\s+)?(?:int|uint32_t|uint16_t|uint8_t|unsigned|long|TickType_t)\s+([a-zA-Z0-9_]+)\s*=\s*([0-9]+))',
    re.MULTILINE
)

CANRX_ARRAY_PATTERN = re.compile(
    r'cmr_canRXMeta_t\s+([a-zA-Z0-9_]+)\s*\[[^\]]+\]\s*=\s*\{([^}]+)\}',
    re.MULTILINE | re.DOTALL
)

CANRX_ENTRY_PATTERN = re.compile(
    r'\[\s*([A-Z_]+)\s*\]\s*=\s*\{[^}]*\.canID\s*=\s*([A-Z_]+)[^}]*(?:\.timeoutError_ms\s*=\s*([0-9]+))?[^}]*\}',
    re.MULTILINE | re.DOTALL
)

GETPAYLOAD_PATTERNS = [
    re.compile(
        r'(?:volatile\s+)?(?:const\s+)?(cmr_\w+_t)\s*\*\s*\w+\s*=\s*(?:\w*[gG]et[pP]ayload|getPayload)\s*\(\s*(CANRX_\w+)\s*\)',
        re.IGNORECASE | re.MULTILINE
    ),
    re.compile(
        r'(cmr_\w+_t)\s*\*\s*\w+\s*=\s*(?:\w*[gG]et[pP]ayload|getPayload)\s*\(\s*(CANRX_\w+)\s*\)',
        re.IGNORECASE | re.MULTILINE
    ),
    re.compile(
        r'\(\s*(?:volatile\s+)?(?:const\s+)?(cmr_\w+_t)\s*\*\s*\)\s*(?:\w*[gG]et[pP]ayload|getPayload)\s*\(\s*(CANRX_\w+)\s*\)',
        re.IGNORECASE | re.MULTILINE
    ),
    re.compile(
        r'(?:volatile\s+)?(?:const\s+)?(cmr_\w+_t)\s*\*\s*\w+\s*=\s*\(\s*(?:volatile\s+)?(?:const\s+)?cmr_\w+_t\s*\*\s*\)\s*(?:\w*[gG]et[pP]ayload|getPayload)\s*\(\s*(CANRX_\w+)\s*\)',
        re.IGNORECASE | re.MULTILINE
    ),
    re.compile(
        r'(?:\(\s*)+(?:volatile\s+)?(?:const\s+)?(cmr_\w+_t)\s*\*\s*\)\s*(?:\w*[gG]et[pP]ayload|getPayload)\s*\(\s*(CANRX_\w+)\s*\)',
        re.IGNORECASE | re.MULTILINE
    ),
]

INDIRECT_PAYLOAD_PATTERN = re.compile(
    r'(?:volatile\s+)?(?:const\s+)?(cmr_\w+_t)\s*\*\s*\w+\s*=\s*\([^)]+\)\s*\w+->payload',
    re.IGNORECASE | re.MULTILINE
)

META_PATTERN = re.compile(
    r'cmr_canRXMeta_t\s*\*\s*(\w+)\s*=\s*canRXMeta\s*\+\s*(CANRX_\w+)',
    re.IGNORECASE | re.MULTILINE
)

CANRX_DEFINITION_PATTERN = re.compile(
    r'\[(\w*CANRX_\w+)\]\s*=\s*\{[^}]*?\.canID\s*=\s*((?:CMR_CANID_|CAN_ID_)\w+)',
    re.IGNORECASE | re.MULTILINE | re.DOTALL
)

def is_canid(arg):
    arg = arg.strip()
    return any(arg.startswith(prefix) for prefix in CANID_PREFIXES)

def is_canrx(arg):
    arg = arg.strip()
    return any(arg.startswith(prefix) for prefix in CANRX_PREFIXES)

def extract_variable_types(content):
    var_to_type = {}
    
    matches = VAR_DECLARATION_PATTERN.findall(content)
    for type_name, var_name in matches:
        var_to_type[var_name] = type_name
    
    return var_to_type

def extract_variable_declarations_ordered(content):
    decls = []
    for m in VAR_DECLARATION_PATTERN.finditer(content):
        type_name, var_name = m.group(1), m.group(2)
        decls.append((m.start(), type_name, var_name))
    return decls

def type_for_var_at_position(var_name, position, ordered_declarations):
    current = None
    for pos, type_name, vname in ordered_declarations:
        if vname == var_name and pos < position:
            current = type_name
    return current

def extract_struct_member_types(content):
    member_to_type = {}
    
    struct_matches = STRUCT_CONTENT_PATTERN.findall(content)
    for struct_content, struct_name in struct_matches:
        member_matches = MEMBER_DECLARATION_PATTERN.findall(struct_content)
        for member_type, member_name in member_matches:
            full_member_name = f"{struct_name}.{member_name}"
            member_to_type[full_member_name] = member_type
            member_to_type[member_name] = member_type
    
    return member_to_type

def extract_constant_definitions(content):
    constants = {}
    
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
    canrx_to_canid = {}
    canrx_to_timeout = {}
    
    if 'cmr_canRXMeta_t' not in content:
        return canrx_to_canid, canrx_to_timeout
    
    COMPLETE_ENTRY_PATTERN = re.compile(
        r'\[\s*(CANRX_\w+)\s*\]\s*=\s*\{([^}]+)\}',
        re.IGNORECASE | re.MULTILINE
    )
    
    matches = COMPLETE_ENTRY_PATTERN.findall(content)
    
    for canrx_key, entry_content in matches:
        canrx_key = canrx_key.strip()
        
        if not is_canrx(canrx_key):
            continue
        if canrx_key.endswith("_LEN"):
            continue

        canid_match = re.search(r'\.canID\s*=\s*((?:CMR_CANID_|CAN_ID_)\w+)', entry_content, re.IGNORECASE)
        if canid_match:
            can_id = canid_match.group(1).strip()
            if is_canid(can_id):
                canrx_to_canid[canrx_key] = can_id
        
        timeout_match = re.search(r'\.timeoutError_ms\s*=\s*([0-9]+)', entry_content, re.IGNORECASE)
        if timeout_match:
            try:
                timeout_val = int(timeout_match.group(1))
                canrx_to_timeout[canrx_key] = timeout_val
            except ValueError:
                pass
    
    return canrx_to_canid, canrx_to_timeout

def extract_canrx_type_mappings(content):
    """Extract CANRX to type mappings from *getPayload* calls using comprehensive patterns"""
    canrx_to_type = {}
    
    if not re.search(r"[gG]et[pP]ayload", content):
        return canrx_to_type
    
    for pattern in GETPAYLOAD_PATTERNS:
        matches = pattern.findall(content)
        for match in matches:
            if len(match) == 2:
                type_name, canrx_key = match
                canrx_key = canrx_key.strip()
                if is_canrx(canrx_key):
                    canrx_to_type[canrx_key] = type_name
    
    indirect_matches = INDIRECT_PAYLOAD_PATTERN.findall(content)
    if indirect_matches:
        meta_matches = META_PATTERN.findall(content)
        meta_to_canrx = {}
        for meta_var, canrx_name in meta_matches:
            meta_to_canrx[meta_var] = canrx_name
        
        for type_name in indirect_matches:
            pass
    
    return canrx_to_type

def parse_sizeof_argument(sizeof_arg):
    """Parse sizeof argument to extract the key for type lookup"""
    sizeof_arg = sizeof_arg.strip()
    
    if sizeof_arg.startswith('*'):
        sizeof_arg = sizeof_arg[1:].strip()
    
    #Handle struct->member or variable->member
    if '->' in sizeof_arg:
        parts = sizeof_arg.split('->')
        if len(parts) >= 2:
            member_name = parts[-1].strip()
            if member_name.startswith('can') or member_name.startswith('cmr_'):
                return member_name
            return member_name
    
    if '.' in sizeof_arg:
        parts = sizeof_arg.split('.')
        if len(parts) >= 2:
            member_name = parts[-1].strip()
            # If the member name looks like a type (starts with 'can'), return it directly
            if member_name.startswith('can') or member_name.startswith('cmr_'):
                return member_name
            return member_name
    
    return sizeof_arg

def resolve_constant_value(constant_name, local_constants, global_constants):
    constant_name = constant_name.strip()
    
    try:
        return int(constant_name)
    except ValueError:
        pass
    
    if constant_name in local_constants:
        value = local_constants[constant_name]
        if isinstance(value, int):
            return value
        try:
            return int(value)
        except (ValueError, TypeError):
            return value
    
    if constant_name in global_constants:
        value = global_constants[constant_name]
        if isinstance(value, int):
            return value
        try:
            return int(value)
        except (ValueError, TypeError):
            return value
    
    return constant_name

def resolve_sizeof_type(sizeof_arg, local_var_types, global_var_types, local_struct_members, global_struct_members, position=None, ordered_var_declarations=None):
    lookup_key = parse_sizeof_argument(sizeof_arg)
    
    if lookup_key.startswith('cmr_') and lookup_key.endswith('_t'):
        return lookup_key, True
    
    if lookup_key.startswith('can') and not lookup_key.startswith('can_'):
        proper_type = f"cmr_{lookup_key}_t"
        return proper_type, True
    
    if ordered_var_declarations is not None and position is not None and re.match(r'^[a-zA-Z_][a-zA-Z0-9_]*$', lookup_key):
        scoped = type_for_var_at_position(lookup_key, position, ordered_var_declarations)
        if scoped is not None:
            return scoped, True

    if lookup_key in local_var_types:
        return local_var_types[lookup_key], True
    
    if lookup_key in local_struct_members:
        return local_struct_members[lookup_key], True
    
    if lookup_key in global_var_types:
        return global_var_types[lookup_key], True
    
    if lookup_key in global_struct_members:
        return global_struct_members[lookup_key], True
    
    return lookup_key, False

def calculate_cycle_time_and_timeout(cycle_time_value):
    if isinstance(cycle_time_value, int):
        time_out = 5 * cycle_time_value
    else:
        time_out = f"5*{cycle_time_value}"
    
    return cycle_time_value, time_out

def clean_canid(raw_canid):
    return re.split(r'\s|\+|\-', raw_canid.strip())[0]

def extract_canids_from_file(filepath, global_var_types, global_struct_members, global_constants, global_canrx_mappings, global_canrx_timeouts, global_canrx_types):
    try:
        with open(filepath, 'r', encoding='utf-8', errors='ignore') as f:
            content = f.read()
        
        local_var_types = extract_variable_types(content)
        ordered_var_declarations = extract_variable_declarations_ordered(content)
        local_struct_members = extract_struct_member_types(content)
        local_constants = extract_constant_definitions(content)
        local_canrx_mappings, local_canrx_timeouts = extract_canrx_mappings(content)
        local_canrx_types = extract_canrx_type_mappings(content)
        
        result = {}
        
        CAN_CALL_5_ARGS_FULL = re.compile(
            r'\bcanTX\s*\(\s*([^,]+)\s*,\s*([^,]+)\s*,\s*([^,]+)\s*,\s*sizeof\s*\(\s*([^)]+)\s*\)\s*,\s*([^)]+)\s*\)',
            re.MULTILINE | re.DOTALL
        )
        
        for match in CAN_CALL_5_ARGS_FULL.finditer(content):
            arg1, arg2, arg3, sizeof_arg, period = match.groups()
            arg1 = arg1.strip()
            arg2 = arg2.strip()
            arg3 = arg3.strip()
            sizeof_arg = sizeof_arg.strip()
            period = period.strip()
            call_pos = match.start()

            canid = None
            if is_canid(arg1):
                canid = clean_canid(arg1)
            elif is_canid(arg2):
                canid = clean_canid(arg2)
            elif is_canid(arg3):
                canid = clean_canid(arg3)

            if canid:
                resolved_type, _ = resolve_sizeof_type(
                    sizeof_arg, local_var_types, global_var_types, local_struct_members, global_struct_members,
                    position=call_pos, ordered_var_declarations=ordered_var_declarations
                )
                
                cycle_time = resolve_constant_value(period, local_constants, global_constants)
                cycle_time, time_out = calculate_cycle_time_and_timeout(cycle_time)
                
                result[canid] = {
                    "type": resolved_type,
                    "cycleTime": cycle_time,
                    "timeOut": time_out,
                    "source": "canTX"
                }

        CAN_CALL_4_ARGS_FULL = re.compile(
            r'\bcanTX\s*\(\s*([^,]+)\s*,\s*([^,]+)\s*,\s*sizeof\s*\(\s*([^)]+)\s*\)\s*,\s*([^)]+)\s*\)',
            re.MULTILINE | re.DOTALL
        )
        
        for match in CAN_CALL_4_ARGS_FULL.finditer(content):
            arg1, arg2, sizeof_arg, period = match.groups()
            arg1 = arg1.strip()
            arg2 = arg2.strip()
            sizeof_arg = sizeof_arg.strip()
            period = period.strip()
            call_pos = match.start()

            canid = None
            if is_canid(arg1):
                canid = clean_canid(arg1)
            elif is_canid(arg2):
                canid = clean_canid(arg2)
            
            if canid and canid not in result:
                resolved_type, _ = resolve_sizeof_type(
                    sizeof_arg, local_var_types, global_var_types, local_struct_members, global_struct_members,
                    position=call_pos, ordered_var_declarations=ordered_var_declarations
                )
                
                cycle_time = resolve_constant_value(period, local_constants, global_constants)
                cycle_time, time_out = calculate_cycle_time_and_timeout(cycle_time)
                
                result[canid] = {
                    "type": resolved_type,
                    "cycleTime": cycle_time,
                    "timeOut": time_out,
                    "source": "canTX"
                }

        all_canrx_mappings = {**global_canrx_mappings, **local_canrx_mappings}
        all_canrx_timeouts = {**global_canrx_timeouts, **local_canrx_timeouts}
        all_canrx_types = {**global_canrx_types, **local_canrx_types}
        
        for canrx_key, canrx_type in all_canrx_types.items():
            if canrx_key in all_canrx_mappings:
                canid = all_canrx_mappings[canrx_key]
                
                if canid in result:
                    continue
                else:
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
    c_files = []
    for root, dirs, files in os.walk(root_dir):
        for filename in files:
            if filename.endswith(('.c', '.cpp', '.cc', '.cxx', '.h', '.hpp', '.hh', '.hxx')):
                c_files.append(os.path.join(root, filename))
    return c_files

def build_global_mappings(c_files):
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

def merge_canid_extractions(c_files):
    global_var_types, global_struct_members, global_constants, global_canrx_mappings, global_canrx_timeouts, global_canrx_types = build_global_mappings(c_files)
    canid_to_info = {}
    files_with_matches = 0
    for filepath in c_files:
        result = extract_canids_from_file(filepath, global_var_types, global_struct_members, global_constants, global_canrx_mappings, global_canrx_timeouts, global_canrx_types)
        if result:
            files_with_matches += 1
        for canid, info in result.items():
            if canid not in canid_to_info:
                canid_to_info[canid] = info
            elif info.get("source") == "canTX" and canid_to_info[canid].get("source") == "canRX":
                canid_to_info[canid] = info
    return (
        canid_to_info,
        files_with_matches,
        global_var_types,
        global_struct_members,
        global_constants,
        global_canrx_mappings,
        global_canrx_timeouts,
        global_canrx_types,
    )

def main():
    c_files = find_all_c_files(ROOT_DIR)
    
    if not c_files:
        print("No C/C++ files found!")
        return
    
    (
        canid_to_info,
        files_with_matches,
        global_var_types,
        global_struct_members,
        global_constants,
        global_canrx_mappings,
        global_canrx_timeouts,
        global_canrx_types,
    ) = merge_canid_extractions(c_files)
    
    cantx_count = 0
    canrx_count = 0
    
    for canid, info in canid_to_info.items():
        if info.get("source") == "canTX":
            cantx_count += 1
        elif info.get("source") == "canRX":
            canrx_count += 1
    
    cleaned_canid_to_info = {}
    for canid, info in canid_to_info.items():
        cleaned_info = {
            "type": info["type"],
            "cycleTime": info["cycleTime"],
            "timeOut": info["timeOut"]
        }
        cleaned_canid_to_info[canid] = cleaned_info
    
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
    
    print(f"Processed {len(c_files)} files, found {len(cleaned_canid_to_info)} CAN IDs ({cantx_count} TX, {canrx_count} RX) -> {OUTPUT_FILE}")

if __name__ == "__main__":
    main()