#!/usr/bin/env python3
"""
Script to update CAN ID JSON file with additional fields (f, p, e, min, max, state)
by parsing can_types_new file and matching struct types.
"""

import json
import re
import os
import sys

# Configuration
INPUT_JSON = "stm32f413-drivers/filegen/canid_type_map.json"  # Your existing JSON file
CAN_TYPES_FILE = "stm32f413-drivers/filegen/can_types_new.h"  # File containing the struct definitions
OUTPUT_FILE = "stm32f413-drivers/filegen/canid_type_map.json"

def parse_can_types_file(file_path):
    """Parse the can_types file and extract struct definitions with field attributes."""
    structs = {}
    
    try:
        with open(file_path, 'r') as f:
            content = f.read()
    except FileNotFoundError:
        print(f"Error: Could not find can_types file: {file_path}")
        return structs
    
    # Find all typedef struct definitions
    struct_pattern = r'typedef\s+struct\s*\{(.*?)\}\s*(\w+);'
    matches = re.findall(struct_pattern, content, re.DOTALL)
    
    for struct_body, struct_name in matches:
        fields = {}
        
        # Find all field definitions with potential attributes
        field_pattern = r'(\w+(?:\s+\w+)*)\s+(\w+)(?:\[.*?\])?;\s*(?://(.*))?'
        field_matches = re.findall(field_pattern, struct_body)
        
        for field_type, field_name, comment in field_matches:
            field_info = {}
            
            if comment:
                # Parse comment for attributes like f:, p:, e:, etc.
                # Look for patterns like f:0.001, e:State, p:100, min:0, max:255
                attr_patterns = {
                    'f': r'f:([\d.-]+)',
                    'p': r'p:([\d.-]+)', 
                    'e': r'e:(\w+)',
                    'min': r'min:([\d.-]+)',
                    'max': r'max:([\d.-]+)',
                    'state': r'state:(\w+)'
                }
                
                for attr, pattern in attr_patterns.items():
                    match = re.search(pattern, comment)
                    if match:
                        value = match.group(1)
                        # Try to convert to number if possible
                        try:
                            if '.' in value:
                                field_info[attr] = float(value)
                            else:
                                field_info[attr] = int(value)
                        except ValueError:
                            field_info[attr] = value
            
            if field_info:  # Only add if we found attributes
                fields[field_name] = field_info
        
        if fields:  # Only add struct if it has fields with attributes
            structs[struct_name] = fields
    
    return structs

def update_json_file(json_file_path, can_types_data, output_file_path):
    """Update the JSON file with additional fields from can_types data."""
    
    # Read existing JSON
    try:
        with open(json_file_path, 'r') as f:
            data = json.load(f)
    except FileNotFoundError:
        print(f"Error: Could not find JSON file: {json_file_path}")
        return False
    except json.JSONDecodeError as e:
        print(f"Error: Invalid JSON in file {json_file_path}: {e}")
        return False
    
    if 'canid_to_info' not in data:
        print("Error: JSON file doesn't contain 'canid_to_info' section")
        return False
    
    # Update each CAN ID entry
    updated_count = 0
    for canid, canid_info in data['canid_to_info'].items():
        if 'type' in canid_info:
            struct_type = canid_info['type']
            
            # Look for matching struct in can_types_data
            if struct_type in can_types_data:
                # Add the field attributes to the CAN ID info
                canid_info.update(can_types_data[struct_type])
                updated_count += 1
                print(f"Updated {canid} (type: {struct_type})")
    
    # Create output directory if it doesn't exist
    os.makedirs(os.path.dirname(output_file_path), exist_ok=True)
    
    # Write updated JSON to output file
    try:
        with open(output_file_path, 'w') as f:
            json.dump(data, f, indent=4, sort_keys=True)
        print(f"\nSuccessfully updated JSON file: {output_file_path}")
        print(f"Total CAN IDs updated: {updated_count}")
        return True
    except Exception as e:
        print(f"Error writing output file: {e}")
        return False

def main():
    """Main function."""
    print("CAN ID JSON Updater")
    print("=" * 50)
    
    # Check if files exist
    if not os.path.exists(INPUT_JSON):
        print(f"Error: Input JSON file not found: {INPUT_JSON}")
        sys.exit(1)
    
    if not os.path.exists(CAN_TYPES_FILE):
        print(f"Error: CAN types file not found: {CAN_TYPES_FILE}")
        print("Please make sure the can_types_new.h file is in the current directory")
        sys.exit(1)
    
    # Parse can_types file
    print(f"Parsing CAN types from: {CAN_TYPES_FILE}")
    can_types_data = parse_can_types_file(CAN_TYPES_FILE)
    print(f"Found {len(can_types_data)} struct definitions with attributes")
    
    if not can_types_data:
        print("No struct definitions with attributes found!")
        sys.exit(1)
    
    # Update JSON file
    print(f"\nReading JSON from: {INPUT_JSON}")
    success = update_json_file(INPUT_JSON, can_types_data, OUTPUT_FILE)
    
    if not success:
        sys.exit(1)
    
    print("\nDone!")

if __name__ == "__main__":
    main()