import csv
import os
from pathlib import Path
import tempfile

BOARDS = ["DCM", "DIM", "HVC", "HVI", "LV-BMS", "RAM"]
TARGET_HEADER_FILE = "stm32f413-drivers/CMR/include/CMR/logging.h"
ERROR_DATABASE_FILE = 'scripts/errorLogging/errorDatabase.csv'

def main():
    enumerateErrors()
    perfectHashKey = createPerfectHash()
    writeToFile(perfectHashKey)


def writeToFile(perfectHashKey: int):
    with open(ERROR_DATABASE_FILE, 'r', newline='') as errorDataBase:
        
        with open(TARGET_HEADER_FILE, "r") as f:
            lines = f.readlines()

        #finds start text
        startLine = None
        for i, line in enumerate(lines):
            if "AUTOMATICALLY GENERATED" in line:
                startLine = i
            
        if startLine is None:
            raise Exception("Start Text Not found")

        #modifies the perfect hash key
        lines = lines[0:startLine+1]
        lines.append("\n")
        lines.append(f"static uint32_t hash_key = {perfectHashKey};\n")
        lines.append("\n")
        lines.append("typedef enum {\n")
        
        #modifies the enum
        reader = csv.DictReader(errorDataBase)
        for row in reader:
            errorName = row["Error Name"]
            enumName = f"CMR_LOG_{errorName.upper()}"
            lines.append(f"  {enumName},\n")
        lines.append("} cmr_log_errors_t;\n")

        #writes back to file
        with open(TARGET_HEADER_FILE, "w") as f:
            f.writelines(lines)
        

def createPerfectHash() -> int:
    candidateKey = 1
    perfectHash = True
    while(True):
        for board in BOARDS:
            dir = Path(f"{board}/inc")
            if not isPerfectHashForDirectory(dir, candidateKey):
                perfectHash = False
                break
            
        if perfectHash:
            return candidateKey
        
        candidateKey += 1
        perfectHash = True

        
def isPerfectHashForDirectory(directory: Path, candidateKey: int):
    seenHashes = set()
    fileNames = [file.name for file in directory.iterdir() if file.is_file()]
    for fileName in fileNames:
        if fileName in seenHashes: 
            return False
        seenHashes.add(hashString(fileName, candidateKey))

    return True
    
#same hash as in logging.c
def hashString(s: str, hashKey: int) -> int:
    hash = hashKey
    for c in s:
        hash = hash * 33 + ord(c) 
    return hash & 0xFF  # Return lower 8 bits

def enumerateErrors():
    with open(ERROR_DATABASE_FILE, 'r', newline='') as infile, \
         tempfile.NamedTemporaryFile('w', delete= False, newline='') as tmpfile:

        reader = csv.DictReader(infile)
        writer = csv.DictWriter(tmpfile, fieldnames=reader.fieldnames)
        writer.writeheader()
        
        #creates error codes
        for rowNumber, row in enumerate(reader):
            row['Error Code (do not edit)'] = rowNumber
            writer.writerow(row)

    # Replace original with updated file
    os.replace(tmpfile.name, ERROR_DATABASE_FILE)    
    
    
main()