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
    
    # First, remove all /* */ style comments but preserve their content for later parsing
    # We'll extract them and associate them with nearby fields
    comment_blocks = {}
    block_comment_pattern = r'/\*\*(.*?)\*/'
    block_matches = list(re.finditer(block_comment_pattern, content, re.DOTALL))
    
    # Store block comments with their positions
    for match in block_matches:
        comment_blocks[match.start()] = match.group(1).strip()
    
    # Find all typedef struct definitions
    struct_pattern = r'typedef\s+struct\s*\{(.*?)\}\s*(\w+);'
    matches = re.findall(struct_pattern, content, re.DOTALL)
    
    for struct_body, struct_name in matches:
        fields = {}
        
        # Find struct start position in original content
        struct_start = content.find(f'typedef struct')
        if struct_start == -1:
            continue
            
        # Find all field definitions with potential attributes
        field_pattern = r'(\w+(?:\s+\w+)*)\s+(\w+)(?:\[.*?\])?;\s*(?://(.*))?'
        field_matches = re.finditer(field_pattern, struct_body)
        
        for field_match in field_matches:
            field_type, field_name, line_comment = field_match.groups()
            field_info = {}
            
            # Get the position of this field in the original content
            field_start = struct_start + field_match.start()
            
            # Look for the closest /** */ comment before this field
            block_comment = None
            closest_comment_pos = -1
            for comment_pos, comment_content in comment_blocks.items():
                # Check if this comment is before the field and within reasonable distance
                if comment_pos < field_start and comment_pos > closest_comment_pos:
                    # Check if the comment is reasonably close (within a few hundred characters)
                    if field_start - comment_pos < 500:  # Adjust this threshold as needed
                        closest_comment_pos = comment_pos
                        block_comment = comment_content
            
            # Combine comments from both sources
            all_comments = []
            if line_comment:
                all_comments.append(line_comment.strip())
            if block_comment:
                all_comments.append(block_comment)
            
            # Parse all comments for attributes
            for comment in all_comments:
                if comment:
                    # Parse comment for attributes like f:, p:, e:, etc.
                    # Look for patterns like f:0.001, e:State, p:100, min:0, max:255
                    attr_patterns = {
                        'u': r'u:\s*([^\s,/\*]+)',
                        'f': r'f:\s*([\d.-]+)',
                        'p': r'p:\s*([\d.-]+)',
                        'e': r'e:\s*(\w+)',
                        'min': r'min:\s*([\d.-]+)',
                        'max': r'max:\s*([\d.-]+)',
                        'state': r'state:\s*(\w+)',
                        'enumstruct': r'(?i)Flag:\s*(cmr_\w+_t)'
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

def parse_can_types_file_improved(file_path):
    """
    Improved parser that better handles both // and /** */ comments.
    This version processes the file more systematically.
    """
    structs = {}
    
    try:
        with open(file_path, 'r') as f:
            content = f.read()
    except FileNotFoundError:
        print(f"Error: Could not find can_types file: {file_path}")
        return structs
    
    # Find all typedef struct definitions with their content
    struct_pattern = r'typedef\s+struct\s*\{(.*?)\}\s*(\w+);'
    struct_matches = re.finditer(struct_pattern, content, re.DOTALL)
    
    for struct_match in struct_matches:
        struct_body = struct_match.group(1)
        struct_name = struct_match.group(2)
        struct_start = struct_match.start()
        struct_end = struct_match.end()
        
        fields = {}
        
        # Get the section of content that includes potential comments before the struct
        extended_start = max(0, struct_start - 1000)  # Look 1000 chars before struct
        extended_content = content[extended_start:struct_end]
        
        # Find /** */ comments in this extended section
        block_comments = []
        block_comment_pattern = r'/\*\*(.*?)\*/'
        for comment_match in re.finditer(block_comment_pattern, extended_content, re.DOTALL):
            comment_content = comment_match.group(1).strip()
            comment_abs_pos = extended_start + comment_match.start()
            block_comments.append((comment_abs_pos, comment_content))
        
        # Process each field in the struct
        field_pattern = r'(\w+(?:\s+\w+)*)\s+(\w+)(?:\[.*?\])?;\s*(?://(.*))?'
        field_matches = list(re.finditer(field_pattern, struct_body))
        
        for i, field_match in enumerate(field_matches):
            field_type, field_name, line_comment = field_match.groups()
            field_info = {}
            
            # Calculate absolute position of this field
            field_abs_pos = struct_start + struct_body.find('typedef struct {') + len('typedef struct {') + field_match.start()
            
            # Collect all relevant comments for this field
            comments_to_parse = []
            
            # Add line comment if present
            if line_comment:
                comments_to_parse.append(line_comment.strip())
            
            # Find the most relevant block comment
            # Look for block comments that are:
            # 1. Before this field
            # 2. After the previous field (if any)
            # 3. Not too far away
            
            prev_field_pos = -1
            if i > 0:
                prev_field_match = field_matches[i-1]
                prev_field_pos = struct_start + struct_body.find('typedef struct {') + len('typedef struct {') + prev_field_match.end()
            
            best_block_comment = None
            for comment_pos, comment_content in block_comments:
                # Comment should be before current field
                if comment_pos >= field_abs_pos:
                    continue
                
                # Comment should be after previous field (if any)
                if prev_field_pos > 0 and comment_pos <= prev_field_pos:
                    continue
                
                # Comment should be reasonably close
                if field_abs_pos - comment_pos > 800:
                    continue
                
                # This looks like a good candidate
                if best_block_comment is None or comment_pos > best_block_comment[0]:
                    best_block_comment = (comment_pos, comment_content)
            
            if best_block_comment:
                comments_to_parse.append(best_block_comment[1])
            
            # Parse all collected comments for attributes
            for comment in comments_to_parse:
                if comment:
                    attr_patterns = {
                        'u': r'u:\s*([^\s,/\*\n]+)',
                        'f': r'f:\s*([\d.-]+)',
                        'p': r'p:\s*([\d.-]+)',
                        'e': r'e:\s*(\w+)',
                        'min': r'min:\s*([\d.-]+)',
                        'max': r'max:\s*([\d.-]+)',
                        'state': r'state:\s*(\w+)',
                        'enumstruct': r'Flag:\s*(cmr_\w+_t)' 
                    }
                    
                    for attr, pattern in attr_patterns.items():
                        match = re.search(pattern, comment, re.MULTILINE)
                        if match:
                            value = match.group(1).strip()
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
    
    # Parse can_types file using the improved parser
    print(f"Parsing CAN types from: {CAN_TYPES_FILE}")
    can_types_data = parse_can_types_file_improved(CAN_TYPES_FILE)
    print(f"Found {len(can_types_data)} struct definitions with attributes")
    
    if not can_types_data:
        print("No struct definitions with attributes found!")
        sys.exit(1)
    
    # Show what was found for debugging
    for struct_name, fields in can_types_data.items():
        print(f"\nStruct: {struct_name}")
        for field_name, attrs in fields.items():
            print(f"  {field_name}: {attrs}")
    
    # Update JSON file
    print(f"\nReading JSON from: {INPUT_JSON}")
    success = update_json_file(INPUT_JSON, can_types_data, OUTPUT_FILE)
    
    if not success:
        sys.exit(1)
    
if __name__ == "__main__":
    main()