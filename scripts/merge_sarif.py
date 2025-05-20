import subprocess
from pathlib import Path

# Root is one level up from this script's directory
ROOT = Path(__file__).resolve().parents[1]
BUILD_STATIC_DIR = ROOT / "build" / "static"
TARGET_DIRS_FILE = BUILD_STATIC_DIR / "CMakeFiles" / "TargetDirectories.txt"
SARIF_OUT_DIR = ROOT / "build" / "sarif"

def get_external_project_names():
    if not TARGET_DIRS_FILE.exists():
        raise FileNotFoundError(f"{TARGET_DIRS_FILE} not found. Has CMake run yet?")

    project_names = []
    with open(TARGET_DIRS_FILE, 'r') as f:
        for line in f:
            line = line.strip()
            if not line.endswith(".dir"):
                continue
            name = Path(line).stem
            if name not in ("edit_cache", "rebuild_cache"):  # Skip CMake internal targets
                project_names.append(name)
    return project_names

def run_sarif_copy(sarif_dir: Path, output_path: Path):
    try:
        subprocess.run([
            "sarif", "copy",
            str(sarif_dir),
            "--output", str(output_path)
        ], check=True)
        print(f"[+] Wrote merged SARIF → {output_path}")
    except subprocess.CalledProcessError as e:
        print(f"[!] sarif copy failed for {sarif_dir}: {e}")

def merge_sarif_files_for_project(project):
    sarif_input_dir = BUILD_STATIC_DIR / project / "src" / f"{project}-build"
    if not sarif_input_dir.exists():
        print(f"[!] {sarif_input_dir} not found, skipping {project}")
        return

    sarif_files = list(sarif_input_dir.glob("*.sarif"))
    if not sarif_files:
        print(f"[!] No SARIF files found in {sarif_input_dir}")
        return

    SARIF_OUT_DIR.mkdir(exist_ok=True)
    output_path = SARIF_OUT_DIR / f"{project}.sarif"
    run_sarif_copy(sarif_input_dir, output_path)

def main():
    projects = get_external_project_names()
    for project in projects:
        merge_sarif_files_for_project(project)

if __name__ == "__main__":
    main()
