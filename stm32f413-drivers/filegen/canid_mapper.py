import os
import json
import re
from collections import defaultdict

# Run file with "python canid_mapper.py" in terminal

ROOT_DIR = "."
CANID_PREFIXES = ["CMR_CANID_", "CAN_ID_"]
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

def extract_base_canid(arg):
    """Extract base CAN ID from expressions that might contain arithmetic"""
    arg = arg.strip()
    
    # Look for CAN ID at the start of the expression
    for prefix in CANID_PREFIXES:
        if arg.startswith(prefix):
            # Find the CAN ID part (alphanumeric and underscores only)
            match = re.match(r'([A-Z_][A-Z0-9_]*)', arg)
            if match:
                return match.group(1)
    
    return None

def is_canid(arg):
    """Check if an argument contains a CAN ID and extract the base ID"""
    return extract_base_canid(arg) is not None

def get_canid_from_arg(arg):
    """Get the actual CAN ID from an argument (handling arithmetic expressions)"""
    return extract_base_canid(arg)

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

def extract_canids_from_file(filepath, global_var_types, global_struct_members, global_constants):
    """Extract CAN IDs, types, cycleTime, and timeout from a single file"""
    try:
        with open(filepath, 'r', encoding='utf-8', errors='ignore') as f:
            content = f.read()
        
        local_var_types = extract_variable_types(content)
        local_struct_members = extract_struct_member_types(content)
        local_constants = extract_constant_definitions(content)
        
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

            # Determine CAN ID - extract base CAN ID from expressions
            canid = None
            if is_canid(arg1):
                canid = get_canid_from_arg(arg1)
            elif is_canid(arg2):
                canid = get_canid_from_arg(arg2)
            elif is_canid(arg3):
                canid = get_canid_from_arg(arg3)

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
                canid = get_canid_from_arg(arg1)
            elif is_canid(arg2):
                canid = get_canid_from_arg(arg2)
            
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
    
    for filepath in c_files:
        try:
            with open(filepath, 'r', encoding='utf-8', errors='ignore') as f:
                content = f.read()
            
            # Skip files that don't contain relevant content
            if not any(prefix in content for prefix in CANID_PREFIXES):
                continue
                
            var_types = extract_variable_types(content)
            global_var_types.update(var_types)
            
            struct_members = extract_struct_member_types(content)
            global_struct_members.update(struct_members)
            
            constants = extract_constant_definitions(content)
            global_constants.update(constants)
                
        except Exception as e:
            continue
    
    return global_var_types, global_struct_members, global_constants

def main():
    c_files = find_all_c_files(ROOT_DIR)
    
    if not c_files:
        print("No C/C++ files found!")
        return
    
    # Build global mappings
    global_var_types, global_struct_members, global_constants = build_global_mappings(c_files)
    
    # Extract CAN IDs from all files
    canid_to_info = {}
    files_with_matches = 0
    
    for filepath in c_files:
        result = extract_canids_from_file(filepath, global_var_types, global_struct_members, global_constants)
        
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
            "prefixes_used": CANID_PREFIXES
        }
    }
    
    with open(OUTPUT_FILE, 'w') as f:
        json.dump(output_data, f, indent=4, sort_keys=True)
    
    # Final summary
    print(f"Results:")
    print(f"  Files processed: {len(c_files)}")
    print(f"  Files with CAN IDs: {files_with_matches}")
    print(f"  Total CAN IDs found: {len(canid_to_info)}")
    
    print(f"\nSaved to {OUTPUT_FILE}")

if __name__ == "__main__":
    main()