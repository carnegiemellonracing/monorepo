#!/usr/bin/env python3
import os
import re
import sys
import argparse
from typing import List, Set, Dict, Tuple


def extract_types(file_path: str) -> Tuple[Set[str], Set[str], Dict[str, Set[str]]]:
    """
    Extract enum and struct type definitions from a header file.

    Args:
        file_path: Path to the header file

    Returns:
        Tuple of sets containing enum and struct type names, and a dictionary mapping enum types to their values
    """
    with open(file_path, 'r') as f:
        content = f.read()

    # Extract typedef enum declarations
    enum_pattern = r'typedef\s+enum\s+(?:\w+\s+)?\{([^}]*)\}\s*(\w+)\s*;'
    enums = set()
    enum_values = {}

    for enum_content, enum_name in re.findall(enum_pattern, content):
        enums.add(enum_name)

        # Extract individual enum values
        enum_values[enum_name] = set()
        value_pattern = r'(\w+)\s*='
        for value in re.findall(value_pattern, enum_content):
            enum_values[enum_name].add(value)

    # Extract typedef struct declarations
    struct_pattern = r'typedef\s+struct(?:\s+\w+|\s+__attribute__\([^)]*\))?\s+\{[^}]*\}\s*(\w+)\s*;'
    structs = set(re.findall(struct_pattern, content))

    # Also capture structs defined with __attribute__
    attr_struct_pattern = r'typedef\s+struct\s+__attribute__\(\([^)]*\)\)\s+(\w+)\s*\{[^}]*\}\s*(\w+)\s*;'
    for _, struct_name in re.findall(attr_struct_pattern, content):
        structs.add(struct_name)

    return enums, structs, enum_values


def find_usage_in_file(file_path: str, types: Set[str]) -> Set[str]:
    """
    Find which types are used in a file.

    Args:
        file_path: Path to the file to search in
        types: Set of type names to search for

    Returns:
        Set of type names found in the file
    """
    try:
        with open(file_path, 'r', errors='ignore') as f:
            content = f.read()
    except Exception as e:
        print(f"Warning: Could not read {file_path}: {e}", file=sys.stderr)
        return set()

    found_types = set()
    for type_name in types:
        # Look for the type name with word boundaries
        pattern = r'\b' + re.escape(type_name) + r'\b'
        if re.search(pattern, content):
            found_types.add(type_name)

    return found_types


def find_unused_types(dir_path: str, header_files: List[str], exclude_dirs: List[str] = None, file_extensions: List[str] = None) -> Dict[str, Set[str]]:
    """
    Find unused enum and struct types defined in header_files but not used elsewhere in dir_path.

    Args:
        dir_path: Root directory to search in
        header_files: List of header files containing type definitions
        exclude_dirs: List of directories to exclude from the search
        file_extensions: List of file extensions to include in the search

    Returns:
        Dict with 'enums' and 'structs' keys mapping to sets of unused type names
    """
    if exclude_dirs is None:
        exclude_dirs = []

    if file_extensions is None:
        file_extensions = ['.c', '.h']

    # Convert exclude_dirs to absolute paths for easier comparison
    exclude_dirs = [os.path.abspath(os.path.join(dir_path, d)) for d in exclude_dirs]

    # Extract all enum and struct types from the header files
    all_enums = set()
    all_structs = set()
    all_enum_values = {}

    for header_file in header_files:
        enums, structs, enum_values = extract_types(header_file)
        all_enums.update(enums)
        all_structs.update(structs)
        all_enum_values.update(enum_values)

    # Find all types that are used in any file in the directory
    used_types = set()
    used_enum_values = set()

    for root, dirs, files in os.walk(dir_path):
        # Skip excluded directories
        dirs[:] = [d for d in dirs if os.path.abspath(os.path.join(root, d)) not in exclude_dirs]

        for file in files:
            # Only process files with specified extensions
            if not any(file.endswith(ext) for ext in file_extensions):
                continue

            file_path = os.path.join(root, file)

            # Skip the header files themselves
            if file_path in [os.path.abspath(hf) for hf in header_files]:
                continue

            # Find which types are used in this file
            used_in_file = find_usage_in_file(file_path, all_enums | all_structs)
            used_types.update(used_in_file)

            # Find which enum values are used in this file
            for enum_type, values in all_enum_values.items():
                for value in values:
                    pattern = r'\b' + re.escape(value) + r'\b'
                    try:
                        with open(file_path, 'r', errors='ignore') as f:
                            content = f.read()
                            if re.search(pattern, content):
                                used_enum_values.add(value)
                                # Mark the parent enum type as used if any of its values are used
                                used_types.add(enum_type)
                    except Exception as e:
                        print(f"Warning: Could not check for enum values in {file_path}: {e}", file=sys.stderr)

    # Determine which types are unused
    unused_enums = all_enums - used_types
    unused_structs = all_structs - used_types

    return {
        'enums': unused_enums,
        'structs': unused_structs
    }


def main():
    parser = argparse.ArgumentParser(description='Find unused enum and struct types in a C/C++ project')
    parser.add_argument('directory', nargs='?', default='.',
                      help='Root directory to search in (default: current directory)')
    parser.add_argument('--exclude-dirs', nargs='*', default=[], help='Directories to exclude from the search')
    parser.add_argument('--output', default='unused_types.txt',
                      help='Output filename (will be saved in script directory, default: unused_types.txt)')
    parser.add_argument('--stdout', action='store_true',
                      help='Print results to stdout instead of writing to a file')

    args = parser.parse_args()

    # Convert to absolute path for consistency
    root_dir = os.path.abspath(args.directory)

    # Adjusted paths since script is now in scripts/ directory
    header_files = [
        os.path.join(root_dir, 'stm32f413-drivers', 'CMR', 'include', 'CMR', 'can_ids.h'),
        os.path.join(root_dir, 'stm32f413-drivers', 'CMR', 'include', 'CMR', 'can_types.h')
    ]

    # Check if the header files exist
    for header_file in header_files:
        if not os.path.exists(header_file):
            print(f"Error: Header file not found: {header_file}", file=sys.stderr)
            sys.exit(1)

    # Always exclude the stm32f413-drivers directory and build directory from the search
    exclude_dirs = args.exclude_dirs
    if 'stm32f413-drivers' not in exclude_dirs:
        exclude_dirs.append('stm32f413-drivers')
    if 'build' not in exclude_dirs:
        exclude_dirs.append('build')

    # Only scan .c and .h files
    file_extensions = ['.c', '.h']

    unused_types = find_unused_types(root_dir, header_files, exclude_dirs, file_extensions)

    # Set up output stream (file or stdout)
    if args.stdout:
        output_stream = sys.stdout
    else:
        # Adjust output path since script is in scripts/ directory
        output_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), args.output)
        print(f"Writing results to {output_path}")
        output_stream = open(output_path, 'w')

    try:
        # Print results
        print("Unused Enum Types:", file=output_stream)
        for enum_type in sorted(unused_types['enums']):
            print(f"  {enum_type}", file=output_stream)

        print("\nUnused Struct Types:", file=output_stream)
        for struct_type in sorted(unused_types['structs']):
            print(f"  {struct_type}", file=output_stream)

        print(f"\nTotal: {len(unused_types['enums'])} unused enums, {len(unused_types['structs'])} unused structs",
              file=output_stream)
    finally:
        if args.output:
            output_stream.close()


if __name__ == '__main__':
    main()