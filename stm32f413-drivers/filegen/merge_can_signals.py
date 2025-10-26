#!/usr/bin/env python3
"""
CAN Signal Merger Script
Merges messages from CMR 25e.sym into CMR 26x.sym based on CAN ID.
Only adds messages with CAN IDs that don't already exist in 26x sym.
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
    
    first_line = lines[0].strip()
    if not first_line.startswith('[') or not first_line.endswith(']'):
        return None, None
    
    message_name = first_line[1:-1]

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
            var_match = re.match(r'Var=(\w+)', line)
            if var_match:
                var_name = var_match.group(1)
                message_data['variables'].append((var_name, line))
        elif line.startswith('ID='):
            id_match = re.match(r'ID=([0-9A-Fa-f]+)h?', line)
            if id_match:
                message_data['can_id'] = id_match.group(1).upper()
            message_data['properties'].append(line)
        else:
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
    
    message_blocks = re.split(r'\n(?=\[)', content)
    
    messages = {}
    for block in message_blocks:
        if not block.strip():
            continue
            
        message_name, message_data = parse_can_message(block)
        if message_name and message_data:
            messages[message_name] = message_data
    
    return messages

def merge_signals(symv1_path: str, cmr25e_path: str, output_path: str = None) -> None:
    """
    Merge messages from CMR 25e.sym into CMR 26x.sym based on CAN ID.
    Only adds messages whose CAN ID doesn't already exist in 26x sym.
    
    Args:
        symv1_path: Path to CMR 26x.sym file
        cmr25e_path: Path to CMR 25e.sym file  
        output_path: Output file path
    """
    
    print("Parsing CMR 26x.sym...")
    symv1_messages = parse_sym_file(symv1_path)
    
    print("Parsing CMR 25e.sym...")
    cmr25e_messages = parse_sym_file(cmr25e_path)
    
    if not symv1_messages:
        print("Error: No messages found in CMR 26x.sym")
        return
    
    if not cmr25e_messages:
        print("Error: No messages found in CMR 25e.sym")
        return
    
    print(f"Found {len(symv1_messages)} messages in CMR 26x.sym")
    print(f"Found {len(cmr25e_messages)} messages in CMR 25e.sym")
    
    existing_can_ids = set()
    for msg_name, msg_data in symv1_messages.items():
        can_id = msg_data.get('can_id')
        if can_id:
            existing_can_ids.add(can_id)
        else:
            print(f"Warning: No CAN ID found for message {msg_name}")
    
    print(f"Found {len(existing_can_ids)} unique CAN IDs in CMR 26x.sym")
    
    new_messages = []
    skipped_messages = []
    new_signals_count = 0
    
    for cmr_msg_name, cmr_msg_data in cmr25e_messages.items():
        cmr_can_id = cmr_msg_data.get('can_id')
        
        if not cmr_can_id:
            print(f"Warning: No CAN ID found for CMR message {cmr_msg_name}, skipping")
            continue
        
        if cmr_can_id in existing_can_ids:
            skipped_messages.append(f"{cmr_msg_name} (ID: {cmr_can_id})")
        else:
            new_messages.append(f"{cmr_msg_name} (ID: {cmr_can_id})")
            symv1_messages[cmr_msg_name] = cmr_msg_data
            existing_can_ids.add(cmr_can_id)  # Track that we've added this ID
            new_signals_count += len(cmr_msg_data.get('variables', []))
    
    print(f"\nWriting merged file to {output_path}...")
    
    try:
        with open(output_path, 'w', encoding='utf-8') as f:
            for msg_name, msg_data in symv1_messages.items():
                f.write(f"{msg_data['header']}\n")
                
                for prop in msg_data['properties']:
                    f.write(f"{prop}\n")
                
                for var_name, var_line in msg_data['variables']:
                    f.write(f"{var_line}\n")
                
                f.write("\n")
    
    except Exception as e:
        print(f"Error writing output file: {e}")
        return
    
    print("\n" + "="*60)
    print("MERGE SUMMARY")
    print("="*60)
    
    if new_messages:
        print(f"\nNew messages added ({len(new_messages)}):")
        for msg in new_messages:
            print(f"  + {msg}")
    
    if skipped_messages:
        print(f"\nSkipped messages (CAN ID already exists in symv1) ({len(skipped_messages)}):")
        for msg in skipped_messages[:20]:  # Show first 20
            print(f"  - {msg}")
        if len(skipped_messages) > 20:
            print(f"  ... and {len(skipped_messages) - 20} more")
    
    print(f"\nTotal new messages added: {len(new_messages)}")
    print(f"Total new signals added: {new_signals_count}")
    print(f"Output written to: {output_path}")
    
    if len(new_messages) == 0:
        print("\nNo new messages found")

def main():
    print("CAN Signal Merger")
    print("="*50)
    
    symv1_path = os.path.join("stm32f413-drivers", "PCAN", "CMR 26x.sym")
    cmr25e_path = os.path.join("stm32f413-drivers", "PCAN", "CMR 25e.sym")
    output_path = os.path.join("stm32f413-drivers", "PCAN", "CMR 26x.sym")
    
    print(f"Source files:")
    print(f"  CMR 26x.sym: {symv1_path}")
    print(f"  CMR 25e.sym: {cmr25e_path}")
    print(f"Output file: {output_path}")
    
    merge_signals(symv1_path, cmr25e_path, output_path)

if __name__ == "__main__":
    main()