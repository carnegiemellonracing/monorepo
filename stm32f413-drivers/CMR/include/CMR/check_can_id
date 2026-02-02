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
        return

    print(f"Loaded {len(enum_db)} unique CAN IDs from Enum.")
    print("-" * 50)
    print("Enter a new CAN ID to check (e.g., 0x123 or 291):")
    print("(Enter 'q' to quit)")
    print("-" * 50)

    while True:
        user_input = input("\nCheck ID: ").strip()
        
        if user_input.lower() == 'q':
            break
        if not user_input:
            continue

        try:
            # Parse user input
            new_id = int(user_input, 0)
            hex_display = f"0x{new_id:X}"
            
            if new_id in enum_db:
                print(f"❌ [CONFLICT] ID {hex_display} (Decimal: {new_id}) already exists in Enum!")
                print(f"   Used by: {', '.join(enum_db[new_id])}")
            else:
                print(f"✅ [AVAILABLE] ID {hex_display} is free to use (not found in Enum).")
                
        except ValueError:
            print("⚠️ Invalid format. Please enter a number (e.g., 0x500 or 1280).")

if __name__ == "__main__":
    main()
