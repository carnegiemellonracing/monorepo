#!/usr/bin/env python3
"""
Script to update CAN IDs JSON with additional type information from C header file.
Parses C struct definitions and extracts f, p, e, min, max, state values from comments.
"""

import json
import re
from typing import Dict, Any, Optional

def parse_c_header(header_content: str) -> Dict[str, Dict[str, Any]]:
    """
    Parse C header file and extract struct information with field metadata.
    
    Args:
        header_content: Content of the C header file
        
    Returns:
        Dictionary mapping struct names to their field information
    """
    structs = {}
    
    # Pattern to match typedef struct definitions
    struct_pattern = r'typedef\s+struct\s*{([^}]+)}\s*(\w+);'
    
    for match in re.finditer(struct_pattern, header_content, re.DOTALL):
        struct_body = match.group(1)
        struct_name = match.group(2)
        
        fields = {}
        
        # Pattern to match field definitions with optional comments
        field_pattern = r'(\w+)\s+(\w+)(?:\[.*?\])?;\s*(?://([^/\n]*?))?(?:\*\*<[^>]*>)?'
        
        for field_match in re.finditer(field_pattern, struct_body):
            field_type = field_match.group(1)
            field_name = field_match.group(2)
            comment = field_match.group(3) if field_match.group(3) else ""
            
            field_info = {
                'type': field_type,
                'name': field_name
            }
            
            # Parse comment for metadata (f:, p:, e:, min:, max:, state:)
            if comment:
                comment = comment.strip()
                
                # Extract factor (f:)
                f_match = re.search(r'f:([\d.-]+)', comment)
                if f_match:
                    field_info['f'] = float(f_match.group(1))
                
                # Extract precision (p:)
                p_match = re.search(r'p:(\d+)', comment)
                if p_match:
                    field_info['p'] = int(p_match.group(1))
                
                # Extract error (e:)
                e_match = re.search(r'e:([\d.-]+)', comment)
                if e_match:
                    field_info['e'] = float(e_match.group(1))
                
                # Extract min value
                min_match = re.search(r'min:([\d.-]+)', comment)
                if min_match:
                    field_info['min'] = float(min_match.group(1))
                
                # Extract max value
                max_match = re.search(r'max:([\d.-]+)', comment)
                if max_match:
                    field_info['max'] = float(max_match.group(1))
                
                # Extract state information
                state_match = re.search(r'state:(\w+)', comment)
                if state_match:
                    field_info['state'] = state_match.group(1)
            
            fields[field_name] = field_info
        
        structs[struct_name] = fields
    
    return structs

def update_json_with_types(json_data: Dict[str, Any], struct_info: Dict[str, Dict[str, Any]]) -> Dict[str, Any]:
    """
    Update the JSON data with additional type information from parsed structs.
    
    Args:
        json_data: Original JSON data
        struct_info: Parsed struct information
        
    Returns:
        Updated JSON data
    """
    updated_data = json_data.copy()
    
    for can_id, can_info in updated_data["canid_to_info"].items():
        struct_type = can_info.get("type")
        
        if struct_type and struct_type in struct_info:
            # Add struct field information to the CAN ID entry
            can_info["fields"] = struct_info[struct_type]
            
            # Count fields with different metadata types
            metadata_counts = {
                'f_fields': 0,
                'p_fields': 0, 
                'e_fields': 0,
                'min_fields': 0,
                'max_fields': 0,
                'state_fields': 0
            }
            
            for field_name, field_info in struct_info[struct_type].items():
                if 'f' in field_info:
                    metadata_counts['f_fields'] += 1
                if 'p' in field_info:
                    metadata_counts['p_fields'] += 1
                if 'e' in field_info:
                    metadata_counts['e_fields'] += 1
                if 'min' in field_info:
                    metadata_counts['min_fields'] += 1
                if 'max' in field_info:
                    metadata_counts['max_fields'] += 1
                if 'state' in field_info:
                    metadata_counts['state_fields'] += 1
            
            # Add metadata counts to the CAN ID info
            can_info.update(metadata_counts)
    
    return updated_data

def main():
    """Main function to process files and update JSON."""
    
    # Read the original JSON file
    try:
        with open('stm32f412-drivers/filegen/canid_type_map.json', 'r') as f:
            json_data = json.load(f)
    except FileNotFoundError:
        print("Error: canid_type_map.json not found. Please ensure the JSON file exists.")
        return
    except json.JSONDecodeError as e:
        print(f"Error parsing JSON: {e}")
        return
    
    # Read the C header file (you'll need to save your C code to a file)
    import os
    print(f"Current directory: {os.getcwd()}")
    print(f"Files in directory: {os.listdir('.')}")
    
    header_file_path = os.path.abspath('stm32f412-drivers/filegen/can_types_new.h')
    print(f"Looking for header file at: {header_file_path}")
    print(f"File exists: {os.path.exists('stm32f412-drivers/filegen/can_types_new.h')}")
    
    try:
        with open('stm32f412-drivers/filegen/can_types_new.h', 'r') as f:
            header_content = f.read()
        print(f"Successfully read can_types_new.h ({len(header_content)} characters)")
    except FileNotFoundError:
        print("Error: can_types_new.h not found. Please save your C header content to this file.")
        return
    except Exception as e:
        print(f"Error reading file: {e}")
        return
    
    # Parse the C header file
    print("Parsing C header file...")
    struct_info = parse_c_header(header_content)
    print(f"Found {len(struct_info)} struct definitions")
    
    # Update JSON with parsed information
    print("Updating JSON with type information...")
    updated_json = update_json_with_types(json_data, struct_info)
    
    # Write updated JSON to new file
    output_filename = 'stm32f412-drivers/filegen/updated_can_types.json'
    with open(output_filename, 'w') as f:
        json.dump(updated_json, f, indent=4)
    
    print(f"Updated JSON saved to {output_filename}")
    
    # Print summary
    print("\nSummary:")
    print(f"- Total CAN IDs: {len(updated_json['canid_to_info'])}")
    
    matched_types = 0
    for can_info in updated_json["canid_to_info"].values():
        if "fields" in can_info:
            matched_types += 1
    
    print(f"- CAN IDs with matched struct types: {matched_types}")
    print(f"- CAN IDs without matches: {len(updated_json['canid_to_info']) - matched_types}")

if __name__ == "__main__":
    main()