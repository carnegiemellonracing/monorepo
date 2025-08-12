import os
import json
import re
from collections import defaultdict

# Run file with "python canid_mapper.py" in terminal

ROOT_DIR = "."
CANID_PREFIXES = ["CMR_CANID_", "CAN_ID_"]
CANRX_PREFIXES = ["CANRX_"]
OUTPUT_FILE = "stm32f413-drivers/filegen/canid_type_map.json"

# Handle both 4 and 5 argument canTX calls
CANTX_PATTERN_5_ARGS = re.compile(
    r'canTX\s*\(\s*([^,]+)\s*,\s*([^,]+)\s*,\s*([^,]+)\s*,\s*sizeof\s*\(\s*([^)]+)\s*\)\s*(?:,\s*[^)]+)?\s*\)',
    re.MULTILINE | re.DOTALL
)

CANTX_PATTERN_4_ARGS = re.compile(
    r'canTX\s*\(\s*([^,]+)\s*,\s*([^,]+)\s*,\s*sizeof\s*\(\s*([^)]+)\s*\)\s*(?:,\s*[^)]+)?\s*\)',
    re.MULTILINE | re.DOTALL
)

# Pattern to find variable declarations of cmr_*_t types (including pointers)
VAR_DECLARATION_PATTERN = re.compile(
    r'(?:const\s+)?(cmr_[a-zA-Z0-9_]*_t)\s*\*?\s*([a-zA-Z0-9_]+)\s*(?:=|;)',
    re.MULTILINE
)

# Pattern to find struct member declarations like "cmr_type_t memberName;" inside structs
STRUCT_MEMBER_PATTERN = re.compile(
    r'typedef\s+struct\s*\{[^}]*?(cmr_[a-zA-Z0-9_]*_t)\s+([a-zA-Z0-9_]+)\s*;[^}]*\}\s*([a-zA-Z0-9_]+)\s*;',
    re.MULTILINE | re.DOTALL
)

# More specific pattern for struct members
STRUCT_CONTENT_PATTERN = re.compile(
    r'typedef\s+struct\s*\{([^}]+)\}\s*([a-zA-Z0-9_]+)\s*;',
    re.MULTILINE | re.DOTALL
)

# Pattern to find individual member declarations within struct content
MEMBER_DECLARATION_PATTERN = re.compile(
    r'(cmr_[a-zA-Z0-9_]*_t)\s+([a-zA-Z0-9_]+)\s*;',
    re.MULTILINE
)

# Pattern to find numerical constant definitions; includes TickType_t
CONST_DEFINITION_PATTERN = re.compile(
    r'(?:#define\s+([a-zA-Z0-9_]+)\s+([0-9]+))|(?:(?:static\s+)?(?:const\s+)?(?:int|uint32_t|uint16_t|uint8_t|unsigned|long|TickType_t)\s+([a-zA-Z0-9_]+)\s*=\s*([0-9]+))',
    re.MULTILINE
)

# Pattern to find CANRX array definitions and their entries
CANRX_ARRAY_PATTERN = re.compile(
    r'cmr_canRXMeta_t\s+([a-zA-Z0-9_]+)\s*\[[^\]]+\]\s*=\s*\{([^}]+)\}',
    re.MULTILINE | re.DOTALL
)

# Pattern to find individual CANRX entries within the array
CANRX_ENTRY_PATTERN = re.compile(
    r'\[\s*([A-Z_]+)\s*\]\s*=\s*\{[^}]*\.canID\s*=\s*([A-Z_]+)[^}]*\.timeoutError_ms\s*=\s*([0-9]+)[^}]*\}',
    re.MULTILINE | re.DOTALL
)

# Updated patterns for *getPayload* calls - more specific to match "getPayload" or "___GetPayload"
CANRX_GETPAYLOAD_PATTERN = re.compile(
    r'(?:volatile\s+)?(cmr_[a-zA-Z0-9_]*_t)\s*\*\s*[a-zA-Z0-9_]+\s*=\s*(?:\([^)]*\))?\s*(?:[a-zA-Z0-9_]*GetPayload|getPayload)\s*\(\s*([A-Z_]+)\s*\)',
    re.MULTILINE | re.DOTALL
)

# Alternative pattern for explicit casting - updated to match specific naming
CANRX_GETPAYLOAD_CAST_PATTERN = re.compile(
    r'(?:volatile\s+)?[a-zA-Z0-9_*\s]+\s*=\s*\(\s*(cmr_[a-zA-Z0-9_]*_t)\s*\*\s*\)\s*(?:[a-zA-Z0-9_]*GetPayload|getPayload)\s*\(\s*([A-Z_]+)\s*\)',
    re.MULTILINE | re.DOTALL
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
    
    # Find regular variable declarations (including const and pointer variants)
    matches = VAR_DECLARATION_PATTERN.findall(content)
    for type_name, var_name in matches:
        var_to_type[var_name] = type_name
    
    return var_to_type

def extract_struct_member_types(content):
    """Extract struct_name.member_name to type mappings from file content"""
    member_to_type = {}
    
    struct_matches = STRUCT_CONTENT_PATTERN.findall(content)
    for struct_content, struct_name in struct_matches:
        # Find all cmr_*_t members within this struct
        member_matches = MEMBER_DECLARATION_PATTERN.findall(struct_content)
        for member_type, member_name in member_matches:
            # Store as both struct_name.member_name and just member_name
            full_member_name = f"{struct_name}.{member_name}"
            member_to_type[full_member_name] = member_type
            # Also store just the member name for cases where struct name is variable
            member_to_type[member_name] = member_type
    
    return member_to_type

def extract_constant_definitions(content):
    """Extract constant definitions from file content"""
    constants = {}
    
    # Find #define and variable constant definitions
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
    
    # Skip if no CANRX arrays in content
    if 'cmr_canRXMeta_t' not in content:
        return canrx_to_canid, canrx_to_timeout
    
    # Find CANRX array definitions
    array_matches = CANRX_ARRAY_PATTERN.findall(content)
    for array_name, array_content in array_matches:
        # Find individual entries within the array
        entry_matches = CANRX_ENTRY_PATTERN.findall(array_content)
        for canrx_key, canid, timeout_error_ms in entry_matches:
            canrx_key = canrx_key.strip()
            canid = canid.strip()
            timeout_error_ms = timeout_error_ms.strip()
            
            if is_canrx(canrx_key) and is_canid(canid):
                canrx_to_canid[canrx_key] = canid
                try:
                    timeout_val = int(timeout_error_ms)
                    canrx_to_timeout[canrx_key] = timeout_val
                except ValueError:
                    canrx_to_timeout[canrx_key] = timeout_error_ms
    
    return canrx_to_canid, canrx_to_timeout

def extract_canrx_type_mappings(content):
    """Extract CANRX to type mappings from *getPayload* calls"""
    canrx_to_type = {}
    
    # Skip if no getPayload calls in content
    if 'getPayload' not in content and 'GetPayload' not in content:
        return canrx_to_type
    
    # Pattern 1: Direct assignment with type declaration
    matches = CANRX_GETPAYLOAD_PATTERN.findall(content)
    for type_name, canrx_key in matches:
        canrx_key = canrx_key.strip()
        if is_canrx(canrx_key):
            canrx_to_type[canrx_key] = type_name
    
    # Pattern 2: Explicit casting
    cast_matches = CANRX_GETPAYLOAD_CAST_PATTERN.findall(content)
    for type_name, canrx_key in cast_matches:
        canrx_key = canrx_key.strip()
        if is_canrx(canrx_key):
            canrx_to_type[canrx_key] = type_name
    
    return canrx_to_type

def parse_sizeof_argument(sizeof_arg):
    """Parse sizeof argument to extract the key for type lookup"""
    sizeof_arg = sizeof_arg.strip()
    
    # Handle different sizeof patterns:
    # sizeof(*variable) -> variable
    # sizeof(variable) -> variable  
    # sizeof(struct->member) -> member
    # sizeof(variable->member) -> member
    
    # Remove leading * if present
    if sizeof_arg.startswith('*'):
        sizeof_arg = sizeof_arg[1:].strip()
    
    # Handle struct->member or variable->member
    if '->' in sizeof_arg:
        parts = sizeof_arg.split('->')
        if len(parts) >= 2:
            return parts[-1].strip()
    
    # Handle struct.member
    if '.' in sizeof_arg:
        parts = sizeof_arg.split('.')
        if len(parts) >= 2:
            return parts[-1].strip()
    
    return sizeof_arg

def resolve_constant_value(constant_name, local_constants, global_constants):
    """Resolve a constant name to its numerical value"""
    constant_name = constant_name.strip()
    
    # Try to parse as integer first (in case it's already a number)
    try:
        return int(constant_name)
    except ValueError:
        pass
    
    # Check local constants first
    if constant_name in local_constants:
        value = local_constants[constant_name]
        # If it's already an integer, return it
        if isinstance(value, int):
            return value
        # If it's a string that can be converted to int, convert it
        try:
            return int(value)
        except (ValueError, TypeError):
            return value
    
    # Then check global constants
    if constant_name in global_constants:
        value = global_constants[constant_name]
        # If it's already an integer, return it
        if isinstance(value, int):
            return value
        # If it's a string that can be converted to int, convert it
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
    # Keep only the first token that looks like a CAN ID
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
        
        # Find all canTX calls with 5 arguments
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

            # Determine CAN ID
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
                    "timeOut": time_out
                }

        # Find all canTX calls with 4 arguments
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
                    "timeOut": time_out
                }

        # Handle CANRX mappings
        all_canrx_mappings = {**global_canrx_mappings, **local_canrx_mappings}
        all_canrx_timeouts = {**global_canrx_timeouts, **local_canrx_timeouts}
        all_canrx_types = {**global_canrx_types, **local_canrx_types}
        
        # For each CANRX that has a type mapping, find its CAN ID and map type to ID
        for canrx_key, canrx_type in all_canrx_types.items():
            if canrx_key in all_canrx_mappings:
                canid = all_canrx_mappings[canrx_key]
                # Only add if not already found via canTX
                if canid not in result:
                    # Get timeout and calculate cycle time (timeout / 5)
                    timeout_val = all_canrx_timeouts.get(canrx_key, "N/A")
                    if isinstance(timeout_val, int):
                        cycle_time = timeout_val / 5
                    else:
                        cycle_time = "N/A"
                    
                    result[canid] = {
                        "type": canrx_type,
                        "cycleTime": cycle_time,
                        "timeOut": timeout_val
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
            
            # Skip files that don't contain relevant content
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
    
    # Build global mappings
    global_var_types, global_struct_members, global_constants, global_canrx_mappings, global_canrx_timeouts, global_canrx_types = build_global_mappings(c_files)
    
    # Extract CAN IDs from all files
    canid_to_info = {}
    files_with_matches = 0
    
    for filepath in c_files:
        result = extract_canids_from_file(filepath, global_var_types, global_struct_members, global_constants, global_canrx_mappings, global_canrx_timeouts, global_canrx_types)
        
        if result:
            files_with_matches += 1
        
        canid_to_info.update(result)
    
    # Save results
    output_data = {
        "canid_to_info": canid_to_info,
        "summary": {
            "files_processed": len(c_files),
            "files_with_matches": files_with_matches,
            "total_canids": len(canid_to_info),
            "prefixes_used": CANID_PREFIXES,
            "canrx_prefixes_used": CANRX_PREFIXES
        }
    }
    
    with open(OUTPUT_FILE, 'w') as f:
        json.dump(output_data, f, indent=4, sort_keys=True)
    
    # Final summary
    print(f"Results:")
    print(f"  Files processed: {len(c_files)}")
    print(f"  Files with CAN IDs: {files_with_matches}")
    print(f"  Total CAN IDs found: {len(canid_to_info)}")
    
    if not(canid_to_info):
        print("\nNo CAN IDs found.")
    
    print(f"\nSaved to {OUTPUT_FILE}")

if __name__ == "__main__":
    main()