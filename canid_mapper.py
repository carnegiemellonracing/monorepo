import os
import json
import re
from collections import defaultdict

# Configuration
ROOT_DIR = "."
CANID_PREFIX = "CMR_CANID_"
OUTPUT_FILE = "canid_type_map.json"

# Regex patterns - Updated to handle both 4 and 5 argument canTX calls
# Also handles sizeof(*variable), sizeof(variable), and sizeof(struct->member)
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
    
    # Find struct definitions and their members
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

def parse_sizeof_argument(sizeof_arg):
    """Parse sizeof argument to extract the key for type lookup"""
    # Clean up the argument
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
            return parts[-1].strip()  # Return the member name
    
    # Handle struct.member
    if '.' in sizeof_arg:
        parts = sizeof_arg.split('.')
        if len(parts) >= 2:
            return parts[-1].strip()  # Return the member name
    
    # Just a variable name
    return sizeof_arg

def resolve_sizeof_type(sizeof_arg, local_var_types, global_var_types, local_struct_members, global_struct_members):
    """Resolve the type from sizeof argument, prioritizing local then global mappings"""
    # Parse the sizeof argument to get the lookup key
    lookup_key = parse_sizeof_argument(sizeof_arg)
    
    # If it's already a type (starts with cmr_ and ends with _t)
    if lookup_key.startswith('cmr_') and lookup_key.endswith('_t'):
        return lookup_key, True
    
    # First check local file variable mapping
    if lookup_key in local_var_types:
        return local_var_types[lookup_key], True
    
    # Then check local struct member mapping
    if lookup_key in local_struct_members:
        return local_struct_members[lookup_key], True
    
    # Then check global variable mapping
    if lookup_key in global_var_types:
        return global_var_types[lookup_key], True
    
    # Finally check global struct member mapping
    if lookup_key in global_struct_members:
        return global_struct_members[lookup_key], True
    
    # If we can't resolve it, return the original argument
    return sizeof_arg, False

def extract_canids_from_file(filepath, global_var_types, global_struct_members):
    """Extract CAN IDs and their types from a single file"""
    try:
        with open(filepath, 'r', encoding='utf-8', errors='ignore') as f:
            content = f.read()
        
        # Extract local mappings from this file
        local_var_types = extract_variable_types(content)
        local_struct_members = extract_struct_member_types(content)
        
        result = {}
        
        # First try 5-argument pattern: canTX(bus, canid, &var, sizeof(type), period)
        matches_5_args = CANTX_PATTERN_5_ARGS.findall(content)
        for arg1, arg2, arg3, sizeof_arg in matches_5_args:
            # Clean up arguments
            arg1 = arg1.strip()
            arg2 = arg2.strip()
            arg3 = arg3.strip()
            sizeof_arg = sizeof_arg.strip()
            
            # In 5-arg pattern, CAN ID is typically in second position
            canid = None
            if arg2.startswith(CANID_PREFIX):
                canid = arg2
            elif arg1.startswith(CANID_PREFIX):
                canid = arg1
            elif arg3.startswith(CANID_PREFIX):
                canid = arg3
            
            if canid:
                resolved_type, was_resolved = resolve_sizeof_type(
                    sizeof_arg, local_var_types, global_var_types, local_struct_members, global_struct_members
                )
                result[canid] = resolved_type
        
        # Then try 4-argument pattern: canTX(canid, &var, sizeof(type), period)
        matches_4_args = CANTX_PATTERN_4_ARGS.findall(content)
        for arg1, arg2, sizeof_arg in matches_4_args:
            # Clean up arguments
            arg1 = arg1.strip()
            arg2 = arg2.strip()
            sizeof_arg = sizeof_arg.strip()
            
            # Check if CAN ID is in first or second argument
            canid = None
            if arg1.startswith(CANID_PREFIX):
                canid = arg1
            elif arg2.startswith(CANID_PREFIX):
                canid = arg2
            
            if canid and canid not in result:  # Don't overwrite 5-arg matches
                resolved_type, was_resolved = resolve_sizeof_type(
                    sizeof_arg, local_var_types, global_var_types, local_struct_members, global_struct_members
                )
                result[canid] = resolved_type
        
        return result
    except Exception as e:
        print(f"Error processing {filepath}: {e}")
        return {}

def find_all_c_files(root_dir):
    """Find all C/C++ files in the directory tree"""
    c_files = []
    for root, dirs, files in os.walk(root_dir):
        for filename in files:
            if filename.endswith(('.c', '.cpp', '.cc', '.cxx', '.h', '.hpp', '.hh', '.hxx')):
                c_files.append(os.path.join(root, filename))
    return c_files

def main():
    print("CAN ID to Type Mapper...")
    print(f"Root directory: {os.path.abspath(ROOT_DIR)}")
    
    # Find all C files
    c_files = find_all_c_files(ROOT_DIR)
    print(f"Found {len(c_files)} C/C++ files to process")
    
    if not c_files:
        print(" No C/C++ files found!")
        return
    
    # PHASE 1: Build global mappings from all files
    print("\n Phase 1: Building global type mappings...")
    global_var_types = {}
    global_struct_members = {}
    
    for filepath in c_files:
        try:
            with open(filepath, 'r', encoding='utf-8', errors='ignore') as f:
                content = f.read()
            
            # Extract variable types
            var_types = extract_variable_types(content)
            for var_name, type_name in var_types.items():
                global_var_types[var_name] = type_name
            
            # Extract struct member types
            struct_members = extract_struct_member_types(content)
            for member_name, type_name in struct_members.items():
                global_struct_members[member_name] = type_name
                
        except Exception as e:
            continue
    
    print(f" Built global mapping:")
    print(f"   Variables: {len(global_var_types)}")
    print(f"   Struct members: {len(global_struct_members)}")
    
    # PHASE 2: Extract CAN IDs using local-first then global mappings
    print("\n🔍 Phase 2: Extracting CAN IDs...")
    canid_to_type = {}
    files_processed = 0
    files_with_matches = 0
    
    # Process each file for canTX calls
    for filepath in c_files:
        result = extract_canids_from_file(filepath, global_var_types, global_struct_members)
        
        if result:
            files_with_matches += 1
            print(f"📁 {os.path.basename(filepath)}: {len(result)} CAN IDs")
        
        # Merge results
        canid_to_type.update(result)
        files_processed += 1
        
        if files_processed % 100 == 0:
            print(f"Processed {files_processed}/{len(c_files)} files...")
    
    # Save results
    output_data = {
        "canid_to_type": canid_to_type,
        "summary": {
            "files_processed": files_processed,
            "files_with_matches": files_with_matches,
            "total_canids": len(canid_to_type)
        }
    }
    
    with open(OUTPUT_FILE, 'w') as f:
        json.dump(output_data, f, indent=4, sort_keys=True)
    
    print(f"\n Results:")
    print(f"   Files processed: {files_processed}")
    print(f"   Files with CAN IDs: {files_with_matches}")
    print(f"   Total CAN IDs found: {len(canid_to_type)}")
    
    # Show all results
    if canid_to_type:
        print(f"\n CAN ID Mappings:")
        for canid, type_name in sorted(canid_to_type.items()):
            print(f"   {canid} -> {type_name}")
    else:
        print("\n No CAN IDs found.")
    
    print(f"\n Saved to {OUTPUT_FILE}")

if __name__ == "__main__":
    main()