import json
from pathlib import Path

# Adjust these paths as needed
ROOT = Path(__file__).resolve().parents[1]
BUILD_STATIC_DIR = ROOT / "build" / "static"
TARGET_DIRS_FILE = BUILD_STATIC_DIR / "CMakeFiles" / "TargetDirectories.txt"
SARIF_OUT_DIR = ROOT / "build" / "sarif"


def get_external_project_names():
    if not TARGET_DIRS_FILE.exists():
        raise FileNotFoundError(
            f"{TARGET_DIRS_FILE} not found. Has CMake run yet?"
        )

    project_names = []
    with open(TARGET_DIRS_FILE, 'r') as f:
        for line in f:
            line = line.strip()
            if not line.endswith(".dir"):
                continue
            name = Path(line).stem
            if name not in ("edit_cache", "rebuild_cache"):
                project_names.append(name)
    return project_names


def merge_sarif_in_dir(input_dir: Path, output_file: Path):
    """
    Programmatically merges all .sarif files in `input_dir` by flattening their `runs` arrays
    into a single SARIF log at `output_file`.
    """
    sarif_files = sorted(input_dir.glob("*.sarif"))
    if not sarif_files:
        print(f"[!] No SARIF files found in {input_dir}")
        return

    all_runs = []
    schema = None
    version = None

    for sarif_path in sarif_files:
        # print(f"[+] Loading {sarif_path.name}")
        data = json.loads(sarif_path.read_text())
        schema = schema or data.get("$schema")
        version = version or data.get("version")
        all_runs.extend(data.get("runs", []))

    merged = {
        "$schema": schema,
        "version": version,
        "runs": all_runs
    }

    output_file.parent.mkdir(parents=True, exist_ok=True)
    output_file.write_text(json.dumps(merged, indent=2))
    print(f"[+] Wrote merged SARIF → {output_file}")


def main():
    # Ensure output directory
    SARIF_OUT_DIR.mkdir(parents=True, exist_ok=True)

    # 1) Merge per-project SARIF files
    for project in get_external_project_names():
        src_dir = BUILD_STATIC_DIR / project / "src" / f"{project}-build"
        out_file = SARIF_OUT_DIR / f"{project}.sarif"
        if src_dir.exists():
            merge_sarif_in_dir(src_dir, out_file)
        else:
            print(f"[!] Skipping {project}, src directory not found: {src_dir}")

    # 2) Merge all per-project outputs into one final SARIF
    final_out = SARIF_OUT_DIR / "merged_results.sarif"
    merge_sarif_in_dir(SARIF_OUT_DIR, final_out)


if __name__ == "__main__":
    main()
