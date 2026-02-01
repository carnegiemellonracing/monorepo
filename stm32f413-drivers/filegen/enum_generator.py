import os
import re

def extract_enums_from_file(filepath):
    """Extract enum definitions from C header files"""
    try:
        with open(filepath, 'r', encoding='utf-8', errors='ignore') as f:
            content = f.read()
    except Exception as e:
        print(f"Error reading {filepath}: {e}")
        return {}
    
    enums = {}
    
    #Pattern to match typedef enum blocks
    enum_pattern = re.compile(
        r'typedef\s+enum\s*\{([^}]+)\}\s*(cmr_can\w+_t)\s*;',
        re.MULTILINE | re.DOTALL
    )
    
    matches = enum_pattern.findall(content)
    
    for enum_content, enum_name in matches:
        #Extract the middle part (remove cmr_can prefix and _t suffix)
        if enum_name.startswith('cmr_can') and enum_name.endswith('_t'):
            #Extract middle part: cmr_canState_t -> State
            middle_part = enum_name[7:-2]
            
            enum_values = {}
            
            #Split by lines and process each line
            lines = enum_content.split('\n')
            current_value = 0
            
            for line in lines:
                line = line.strip()
                if not line or line.startswith('/*') or line.startswith('//'):
                    continue
                
                #Remove comments from end of line
                line = re.sub(r'//.*$', '', line)
                line = re.sub(r'/\*.*?\*/', '', line)
                line = line.strip()
                
                if not line:
                    continue
                
                line = line.rstrip(',')
                
                #Check for explicit value assignment
                if '=' in line:
                    parts = line.split('=', 1)
                    enum_member = parts[0].strip()
                    try:
                        value_part = parts[1].strip()
                        #Handle hex values, bit shifts, etc.
                        if value_part.startswith('(') and value_part.endswith(')'):
                            value_part = value_part[1:-1]
                        
                        if value_part.startswith('0x') or value_part.startswith('0X'):
                            current_value = int(value_part, 16)
                        elif '<<' in value_part:
                            #Handle bit shifts like (1 << 0) or (0xF << 12)
                            shift_match = re.match(r'(0x[0-9a-fA-F]+|0X[0-9a-fA-F]+|\d+)U?\s*<<\s*(\d+)', value_part)
                            if shift_match:
                                base_value = shift_match.group(1)
                                shift_amount = int(shift_match.group(2))
                                # Parse base value (could be hex or decimal)
                                if base_value.startswith('0x') or base_value.startswith('0X'):
                                    base = int(base_value, 16)
                                else:
                                    base = int(base_value.rstrip('U'))
                                current_value = base << shift_amount
                            else:
                                # Try to evaluate it directly
                                try:
                                    current_value = eval(value_part.replace('U', ''))
                                except:
                                    current_value = int(value_part)
                        else:
                            # Handle plain integers with optional U suffix
                            try:
                                current_value = int(value_part.rstrip('U'))
                            except ValueError:
                                current_value = int(value_part)
                    except ValueError:
                        continue
                else:
                    enum_member = line.strip()
                
                if enum_member:
                    #Convert enum member name to display string
                    if enum_member.startswith('CMR_CAN_'):
                        #Remove CMR_CAN_ prefix and convert to title case
                        display_name = enum_member[8:]
                        
                        #Remove the enum type prefix if it exists
                        upper_middle = middle_part.upper()
                        if display_name.startswith(upper_middle + '_'):
                            display_name = display_name[len(upper_middle) + 1:]
                        
                        enum_values[current_value] = display_name
                        current_value += 1
                    elif '_LEN' not in enum_member and '_UNKNOWN' not in enum_member:
                        #For other enum members, use as is but clean up
                        enum_values[current_value] = enum_member
                        current_value += 1
            
            if enum_values:
                enums[middle_part] = enum_values
    
    return enums

def format_enum_for_symbol_file(enum_name, enum_values, max_line_len=70):
    """Format an enum for the symbol file with line wrapping if too long"""
    sorted_items = sorted(enum_values.items())
    
    formatted_values = []
    for value, name in sorted_items:
        formatted_values.append(f'{value}="{name}"')
    
    # Start with "enum X("
    prefix = f'enum {enum_name}('
    lines = [prefix]
    
    for i, val in enumerate(formatted_values):
        # Add comma before item except the first
        if i > 0:
            candidate = lines[-1] + ', ' + val
        else:
            candidate = lines[-1] + val
        
        # If candidate line exceeds max length, start new line
        if len(candidate) > max_line_len and i > 0:
            lines[-1] = lines[-1] + ','  # close previous line with comma
            lines.append('    ' + val)   # indent wrapped line
        else:
            lines[-1] = candidate
    
    # Close with ")"
    lines[-1] = lines[-1] + ')'
    
    return '\n'.join(lines)

def find_header_files(root_dir):
    """Find all header files in the directory tree, filtering for can_types.h only"""
    header_files = []
    for root, dirs, files in os.walk(root_dir):
        for filename in files:
            # Only process files named can_types.h
            if filename == 'can_types.h':
                header_files.append(os.path.join(root, filename))
    return header_files

def generate_symbol_enums(root_dir=".", output_file="stm32f413-drivers/PCAN/CMR 26x.sym"):
    """Generate enum definitions and prepend to existing symbol file"""
    
    #Find all can_types.h files
    header_files = find_header_files(root_dir)
    
    if not header_files:
        print("No can_types.h files found!")
        return
    
    print(f"Found {len(header_files)} can_types.h file(s):")
    for f in header_files:
        print(f"  {f}")
    
    all_enums = {}
    
    #Process each header file
    for filepath in header_files:
        file_enums = extract_enums_from_file(filepath)
        all_enums.update(file_enums)
    
    if not all_enums:
        print("No enums found!")
        return
    
    #Read existing file content if it exists
    existing_content = ""
    if os.path.exists(output_file):
        try:
            with open(output_file, 'r', encoding='utf-8') as f:
                existing_content = f.read()
            print(f"Found existing file: {output_file}")
        except Exception as e:
            print(f"Error reading existing file {output_file}: {e}")
            return
    
    #Always start with header content
    final_content_parts = []
    
    #Add file header first
    header_content = [
        'FormatVersion=5.0 // Do not edit this line!',
        'UniqueVariables=True',
        'Title="CMR 26x Generated"',
        ''
    ]
    final_content_parts.extend(header_content)
    
    final_content_parts.extend([
        '// ========== AUTO-GENERATED ENUMS ==========',
        '',
        '{ENUMS}'
    ])
    
    #Sort enums by name for consistent output
    for enum_name in sorted(all_enums.keys()):
        enum_values = all_enums[enum_name]
        enum_line = format_enum_for_symbol_file(enum_name, enum_values)
        final_content_parts.append(enum_line)
    
    final_content_parts.extend([
        '',
        '{SENDRECEIVE}',
        '',
        '// ========== END AUTO-GENERATED ENUMS ==========',
        ''
    ])
    
    #Add existing content (with auto-generated section removed)
    if existing_content:
        lines = existing_content.split('\n')
        filtered_lines = []
        in_auto_section = False
        skip_header = True  #Skip the header from existing content since we're adding our own
        
        for line in lines:
            if skip_header:
                if (line.startswith('FormatVersion=') or 
                    line.startswith('UniqueVariables=') or 
                    line.startswith('Title=') or
                    line.strip() == ''):
                    continue
                else:
                    skip_header = False
            
            if '// ========== AUTO-GENERATED ENUMS ==========' in line:
                in_auto_section = True
                continue
            elif '// ========== END AUTO-GENERATED ENUMS ==========' in line:
                in_auto_section = False
                continue
            elif not in_auto_section:
                filtered_lines.append(line)
        
        #Add the cleaned existing content
        cleaned_existing_content = '\n'.join(filtered_lines).strip()
        if cleaned_existing_content:
            final_content_parts.append(cleaned_existing_content)
    
    #Join all parts
    final_content = '\n'.join(final_content_parts)
    
    #Write the combined content to file
    try:
        #Create directory if it doesn't exist
        os.makedirs(os.path.dirname(output_file), exist_ok=True)
        
        with open(output_file, 'w', encoding='utf-8') as f:
            f.write(final_content)
        
        print(f"Successfully wrote {len(all_enums)} enums to {output_file}")
        
        for enum_name, enum_values in sorted(all_enums.items()):
            print(f"  {enum_name}: {len(enum_values)} values")
            
    except Exception as e:
        print(f"Error writing to {output_file}: {e}")

if __name__ == "__main__":
    generate_symbol_enums()