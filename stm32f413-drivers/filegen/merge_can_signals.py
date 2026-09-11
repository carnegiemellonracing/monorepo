#!/usr/bin/env python3
"""
Merges messages from CMR 25e.sym into CMR 27x.sym based on can id
Only adds messages with CAN IDs that don't already exist in 26x sym
"""

import re
import os
from typing import Dict, Tuple

def parse_can_message(message_block: str) -> Tuple[str, Dict]:
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

def parse_sym_file(file_path: str) -> Tuple[str, Dict[str, Dict]]:
    print(f"Attempting to read: {file_path}")
    print(f"File exists: {os.path.exists(file_path)}")
    
    try:
        normalized_path = os.path.normpath(file_path)
        print(f"Normalized path: {normalized_path}")
        
        with open(normalized_path, 'r', encoding='utf-8') as f:
            content = f.read()
    except FileNotFoundError:
        print(f"Error: File {normalized_path} not found.")
        return "", {}
    except Exception as e:
        print(f"Error reading {normalized_path}: {e}")
        return "", {}
    
    first_message_match = re.search(r'(?m)^\[', content)
    if not first_message_match:
        return content, {}

    preamble = content[:first_message_match.start()]
    message_blocks = re.split(r'\n(?=\[)', content[first_message_match.start():])
    
    messages = {}
    for block in message_blocks:
        if not block.strip():
            continue
            
        message_name, message_data = parse_can_message(block)
        if message_name and message_data:
            messages[message_name] = message_data
    
    return preamble, messages

def unique_extra_name(message_name: str, existing_names: set) -> str:
    new_name = f"{message_name}_extra"
    suffix = 2

    while new_name.casefold() in existing_names:
        new_name = f"{message_name}_extra_{suffix}"
        suffix += 1

    return new_name

def merge_signals(symv1_path: str, cmr25e_path: str, output_path: str = None) -> None:
    
    print("Parsing CMR 27x.sym...")
    symv1_preamble, symv1_messages = parse_sym_file(symv1_path)
    
    print("Parsing CMR 25e.sym...")
    _, cmr25e_messages = parse_sym_file(cmr25e_path)
    
    if not symv1_messages:
        print("Error: No messages found in CMR 27x.sym")
        return
    
    if not cmr25e_messages:
        print("Error: No messages found in CMR 25e.sym")
        return
    
    print(f"Found {len(symv1_messages)} messages in CMR 27x.sym")
    print(f"Found {len(cmr25e_messages)} messages in CMR 25e.sym")
    
    existing_can_ids = set()
    existing_names = {msg_name.casefold() for msg_name in symv1_messages}
    for msg_name, msg_data in symv1_messages.items():
        can_id = msg_data.get('can_id')
        if can_id:
            existing_can_ids.add(can_id)
        else:
            print(f"Warning: No CAN ID found for message {msg_name}")
    
    print(f"Found {len(existing_can_ids)} unique CAN IDs in CMR 27x.sym")
    
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
            if cmr_msg_name.casefold() in existing_names:
                renamed_msg_name = unique_extra_name(cmr_msg_name, existing_names)
                print(
                    f"Renaming duplicate symbol {cmr_msg_name} to "
                    f"{renamed_msg_name} (ID: {cmr_can_id})"
                )
                cmr_msg_data['header'] = f"[{renamed_msg_name}]"
                cmr_msg_name = renamed_msg_name

            new_messages.append(f"{cmr_msg_name} (ID: {cmr_can_id})")
            symv1_messages[cmr_msg_name] = cmr_msg_data
            existing_can_ids.add(cmr_can_id)  # Track that we've added this ID
            existing_names.add(cmr_msg_name.casefold())
            new_signals_count += len(cmr_msg_data.get('variables', []))
    
    print(f"\nWriting merged file to {output_path}...")
    
    try:
        with open(output_path, 'w', encoding='utf-8') as f:
            if symv1_preamble:
                f.write(symv1_preamble)
                if not symv1_preamble.endswith('\n'):
                    f.write('\n')

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
    
    # print("\n" + "="*60)
    # print("MERGE SUMMARY")
    # print("="*60)
    
    # if new_messages:
    #     print(f"\nNew messages added ({len(new_messages)}):")
    #     for msg in new_messages:
    #         print(f"  + {msg}")
    
    # if skipped_messages:
    #     print(f"\nSkipped messages (CAN ID already exists in symv1) ({len(skipped_messages)}):")
    #     for msg in skipped_messages[:20]:  # Show first 20
    #         print(f"  - {msg}")
    #     if len(skipped_messages) > 20:
    #         print(f"  ... and {len(skipped_messages) - 20} more")
    
    print(f"\nTotal new messages added: {len(new_messages)}")
    print(f"Total new signals added: {new_signals_count}")
    print(f"Output written to: {output_path}")
    
    if len(new_messages) == 0:
        print("\nNo new messages found")

def main():
    print("CAN Signal Merger")
    print("="*50)
    
    symv1_path = os.path.join("stm32f413-drivers", "PCAN", "CMR 27x.sym")
    cmr25e_path = os.path.join("stm32f413-drivers", "PCAN", "CMR 25e.sym")
    output_path = os.path.join("stm32f413-drivers", "PCAN", "CMR 27x.sym")
    
    print(f"Source files:")
    print(f"  CMR 27x.sym: {symv1_path}")
    print(f"  CMR 25e.sym: {cmr25e_path}")
    print(f"Output file: {output_path}")
    
    merge_signals(symv1_path, cmr25e_path, output_path)

if __name__ == "__main__":
    main()
