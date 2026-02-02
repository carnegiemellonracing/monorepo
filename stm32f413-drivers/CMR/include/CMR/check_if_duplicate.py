import re
import os

def get_enum_ids(file_path):
    """
    Parses only enum-style definitions (NAME = VALUE).
    Automatically ignores #define macros (since #define usually lacks an equals sign).
    """
    if not os.path.exists(file_path):
        print(f"Error: File not found {file_path}")
        return {}

    existing_ids = {}
    
    # Regex logic:
    # ^\s* -> Match leading whitespace
    # (\w+)  -> Capture variable name (Group 1)
    # \s*=\s* -> Force match on equals sign (Crucial for distinguishing enum from define)
    # (0x[0-9a-fA-F]+|\d+) -> Capture Hex or Decimal value (Group 2)
    pattern = re.compile(r'^\s*(\w+)\s*=\s*(0x[0-9a-fA-F]+|\d+)')

    with open(file_path, 'r', encoding='utf-8', errors='ignore') as f:
        for line in f:
            clean_line = line.split('//')[0].split('/*')[0].strip()
            
            if clean_line.startswith('#'):
                continue

            match = pattern.search(clean_line)
            if match:
                name = match.group(1)
                val_str = match.group(2)
                
                try:
                    val = int(val_str, 0)
                    
                    if val not in existing_ids:
                        existing_ids[val] = []
                    existing_ids[val].append(name)
                except ValueError:
                    continue

    return existing_ids

def main():
    file_name = "can_ids.h"
    print(f"Reading Enum definitions from {file_name}...")
    
    enum_db = get_enum_ids(file_name)
    
    if not enum_db:
        print("No IDs found.")
        return

    print(f"Loaded {len(enum_db)} unique CAN IDs from Enum.")
    print("-" * 50)

    found_duplicate = False

    for can_id, names_list in enum_db.items():
        if len(names_list) > 1:
            print(f"⚠️  Duplicated CAN ID found: {hex(can_id)}")
            print(f"    Shared by: {', '.join(names_list)}")
            found_duplicate = True

    print("-" * 50)
    if found_duplicate:
        print("❌ Test FAILED: Duplicates found in the existing file.")
    else:
        print("✅ Test PASS: No duplicated CAN IDs found.")

if __name__ == "__main__":
    main()
