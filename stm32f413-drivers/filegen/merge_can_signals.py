#!/usr/bin/env python3
"""
CAN Signal Merger Script
Merges signals from CMR 25e.sym into symv1.sym, adding only new signals that don't already exist.
"""

import re
import os
from typing import Dict, Set, List, Tuple

def parse_can_message(message_block: str) -> Tuple[str, Dict]:
    """
    Parse a single CAN message block and return message name and details.
    
    Args:
        message_block: String containing a complete message definition
        
    Returns:
        Tuple of (message_name, message_data_dict)
    """
    lines = message_block.strip().split('\n')
    if not lines:
        return None, None
    
    # Extract message name from first line (e.g., "[AMK_FL_ACT_2]")
    first_line = lines[0].strip()
    if not first_line.startswith('[') or not first_line.endswith(']'):
        return None, None
    
    message_name = first_line[1:-1]  # Remove brackets
    
    # Parse message properties
    message_data = {
        'header': first_line,
        'properties': [],
        'variables': [],
        'raw_block': message_block,
        'can_id': None
    }
    
    for line in lines[1:]:
        line = line.strip()
        if not line:
            continue
            
        if line.startswith('Var='):
            # Extract variable name for comparison
            var_match = re.match(r'Var=(\w+)', line)
            if var_match:
                var_name = var_match.group(1)
                message_data['variables'].append((var_name, line))
        elif line.startswith('ID='):
            # Extract CAN ID
            id_match = re.match(r'ID=([0-9A-Fa-f]+)h?', line)
            if id_match:
                message_data['can_id'] = id_match.group(1).upper()
            message_data['properties'].append(line)
        else:
            # Other properties like DLC, CycleTime, etc.
            message_data['properties'].append(line)
    
    return message_name, message_data

def parse_sym_file(file_path: str) -> Dict[str, Dict]:
    """
    Parse a .sym file and return a dictionary of messages.
    
    Args:
        file_path: Path to the .sym file
        
    Returns:
        Dictionary mapping message names to their data
    """
    print(f"Attempting to read: {file_path}")
    print(f"File exists: {os.path.exists(file_path)}")
    
    try:
        # Normalize the path to handle any path separator issues
        normalized_path = os.path.normpath(file_path)
        print(f"Normalized path: {normalized_path}")
        
        with open(normalized_path, 'r', encoding='utf-8') as f:
            content = f.read()
    except FileNotFoundError:
        print(f"Error: File {normalized_path} not found.")
        return {}
    except Exception as e:
        print(f"Error reading {normalized_path}: {e}")
        return {}
    
    # Split content into message blocks
    # Messages start with [MessageName] and continue until the next [MessageName] or end of file
    message_blocks = re.split(r'\n(?=\[)', content)
    
    messages = {}
    for block in message_blocks:
        if not block.strip():
            continue
            
        message_name, message_data = parse_can_message(block)
        if message_name and message_data:
            messages[message_name] = message_data
    
    return messages

def get_message_variables(message_data: Dict) -> Set[str]:
    """
    Get all variable names from a message.
    
    Args:
        message_data: Message data dictionary
        
    Returns:
        Set of variable names
    """
    return {var_name for var_name, _ in message_data.get('variables', [])}

def merge_signals(symv1_path: str, cmr25e_path: str, output_path: str = None) -> None:
    """
    Merge signals from CMR 25e.sym into symv1.sym based on CAN ID matching.
    
    Args:
        symv1_path: Path to symv1.sym file
        cmr25e_path: Path to CMR 25e.sym file  
        output_path: Output file path (defaults to symv1_merged.sym)
    """
    if output_path is None:
        output_path = "symv1_merged.sym"
    
    print("Parsing symv1.sym...")
    symv1_messages = parse_sym_file(symv1_path)
    
    print("Parsing CMR 25e.sym...")
    cmr25e_messages = parse_sym_file(cmr25e_path)
    
    if not symv1_messages:
        print("Error: No messages found in symv1.sym")
        return
    
    if not cmr25e_messages:
        print("Error: No messages found in CMR 25e.sym")
        return
    
    print(f"Found {len(symv1_messages)} messages in symv1.sym")
    print(f"Found {len(cmr25e_messages)} messages in CMR 25e.sym")
    
    # Create CAN ID to message mapping for symv1
    symv1_by_id = {}
    for msg_name, msg_data in symv1_messages.items():
        can_id = msg_data.get('can_id')
        if can_id:
            symv1_by_id[can_id] = (msg_name, msg_data)
        else:
            print(f"Warning: No CAN ID found for message {msg_name}")
    
    # Track what we're adding
    new_messages = []
    updated_messages = []
    skipped_messages = []
    new_signals_count = 0
    
    # Check each message from CMR 25e
    for cmr_msg_name, cmr_msg_data in cmr25e_messages.items():
        cmr_can_id = cmr_msg_data.get('can_id')
        
        if not cmr_can_id:
            print(f"Warning: No CAN ID found for CMR message {cmr_msg_name}, skipping")
            continue
        
        if cmr_can_id in symv1_by_id:
            # Message with same CAN ID exists in symv1
            symv1_msg_name, symv1_msg_data = symv1_by_id[cmr_can_id]
            
            symv1_vars = get_message_variables(symv1_msg_data)
            cmr_vars = get_message_variables(cmr_msg_data)
            
            # Find new variables
            new_vars = cmr_vars - symv1_vars
            
            if new_vars:
                updated_messages.append((f"{symv1_msg_name} (ID: {cmr_can_id})", new_vars))
                # Add new variables to the existing message
                for var_name, var_line in cmr_msg_data['variables']:
                    if var_name in new_vars:
                        symv1_messages[symv1_msg_name]['variables'].append((var_name, var_line))
                        new_signals_count += 1
            else:
                skipped_messages.append(f"{cmr_msg_name} (ID: {cmr_can_id}) -> {symv1_msg_name}")
        else:
            # Completely new message (new CAN ID)
            new_messages.append(f"{cmr_msg_name} (ID: {cmr_can_id})")
            symv1_messages[cmr_msg_name] = cmr_msg_data
            new_signals_count += len(cmr_msg_data.get('variables', []))
    
    # Write merged file
    print(f"\nWriting merged file to {output_path}...")
    
    try:
        with open(output_path, 'w', encoding='utf-8') as f:
            for msg_name, msg_data in symv1_messages.items():
                # Write message header
                f.write(f"{msg_data['header']}\n")
                
                # Write properties
                for prop in msg_data['properties']:
                    f.write(f"{prop}\n")
                
                # Write variables
                for var_name, var_line in msg_data['variables']:
                    f.write(f"{var_line}\n")
                
                # Add blank line between messages
                f.write("\n")
    
    except Exception as e:
        print(f"Error writing output file: {e}")
        return
    
    # Print summary
    print("\n" + "="*60)
    print("MERGE SUMMARY")
    print("="*60)
    
    if new_messages:
        print(f"\nNew messages added ({len(new_messages)}):")
        for msg in new_messages:
            print(f"  - {msg}")
    
    if updated_messages:
        print(f"\nExisting messages with new signals ({len(updated_messages)}):")
        for msg_name, new_vars in updated_messages:
            print(f"  - {msg_name}: added {len(new_vars)} signals")
            for var in sorted(new_vars):
                print(f"    • {var}")
    
    if skipped_messages:
        print(f"\nSkipped messages (same CAN ID, no new signals) ({len(skipped_messages)}):")
        for msg in skipped_messages[:10]:  # Show first 10 to avoid spam
            print(f"  - {msg}")
        if len(skipped_messages) > 10:
            print(f"  ... and {len(skipped_messages) - 10} more")
    
    print(f"\nTotal new signals added: {new_signals_count}")
    print(f"Output written to: {output_path}")
    
    if new_signals_count == 0:
        print("\nNo new signals found - all signals from CMR 25e.sym already exist in symv1.sym")

def main():
    """Main function to run the merger."""
    print("CAN Signal Merger")
    print("="*50)
    
    # Fixed file paths - no command line arguments
    symv1_path = os.path.join("stm32f413-drivers", "filegen", "symv1.sym")
    cmr25e_path = os.path.join("stm32f413-drivers", "PCAN", "CMR 25e.sym")
    output_path = os.path.join("stm32f413-drivers", "filegen", "symv1.sym")
    
    print(f"Source files:")
    print(f"  symv1.sym: {symv1_path}")
    print(f"  CMR 25e.sym: {cmr25e_path}")
    print(f"Output file: {output_path}")
    
    # Run the merge
    merge_signals(symv1_path, cmr25e_path, output_path)

if __name__ == "__main__":
    main()