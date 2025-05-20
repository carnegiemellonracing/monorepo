import json
from pathlib import Path
import copy  # Added for deepcopy

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
    Programmatically merges all .sarif files in `input_dir`.
    Runs from the same tool (identified by tool.driver info) are merged together.
    Results, artifacts, invocations are concatenated. Tool rules are merged uniquely by ID.
    """
    sarif_files = sorted(input_dir.glob("*.sarif"))
    if not sarif_files:
        print(f"[!] No SARIF files found in {input_dir}")
        return

    merged_runs_map = {}  # Stores merged run data, keyed by tool identity
    final_schema = None
    final_version = None

    for sarif_path in sarif_files:
        # Skip reading the output file itself if it's in the input directory
        if output_file.exists() and sarif_path.resolve() == output_file.resolve():
            # print(f"[+] Skipping {sarif_path.name} as it is the target output file for this operation.")
            continue

        # print(f"[+] Loading {sarif_path.name} for merging runs by tool")
        try:
            data = json.loads(sarif_path.read_text(encoding='utf-8'))
        except json.JSONDecodeError as e:
            print(f"[!] Error decoding JSON from {sarif_path.name}: {e}")
            continue
        except Exception as e:  # Catch other file reading errors
            print(f"[!] Error reading file {sarif_path.name}: {e}")
            continue

        if not final_schema:
            final_schema = data.get("$schema")
        if not final_version:
            final_version = data.get("version")

        for current_run_item in data.get("runs", []):
            tool_definition = current_run_item.get("tool", {})

            # Create a key from the 'driver' part, excluding rules/notifications for identity
            driver_for_key = copy.deepcopy(tool_definition.get("driver", {}))
            driver_for_key.pop("rules", None)
            driver_for_key.pop("notifications", None)
            # Add other volatile parts to pop if necessary for stable identity

            if not driver_for_key:  # Fallback if driver is empty/unspecified
                tool_for_key = copy.deepcopy(tool_definition)
                tool_for_key.pop("extensions", None)
                tool_for_key.pop("driver", None)  # Already handled if it was empty
                if not tool_for_key:
                    tool_identity_key = "__EMPTY_OR_GENERIC_TOOL__"
                else:
                    tool_identity_key = json.dumps(tool_for_key, sort_keys=True)
            else:
                tool_identity_key = json.dumps(driver_for_key, sort_keys=True)

            if tool_identity_key not in merged_runs_map:
                merged_runs_map[tool_identity_key] = copy.deepcopy(current_run_item)
                # Ensure essential lists/dicts are present
                merged_runs_map[tool_identity_key].setdefault("results", [])
                merged_runs_map[tool_identity_key].setdefault("artifacts", []) # Ensure artifacts list exists
                merged_runs_map[tool_identity_key].setdefault("invocations", [])
                merged_runs_map[tool_identity_key].setdefault("originalUriBaseIds", {})
                # Ensure tool.driver.rules list exists for aggregation
                merged_runs_map[tool_identity_key].setdefault("tool", {}).setdefault("driver", {}).setdefault("rules", [])
            else:
                target_run_for_tool = merged_runs_map[tool_identity_key]
                target_run_for_tool["results"].extend(current_run_item.get("results", []))

                # --- Artifact merging logic with de-duplication ---
                target_artifacts_list = target_run_for_tool.setdefault("artifacts", [])
                # Store existing artifact URIs in a set for efficient lookup for URI-based artifacts
                existing_artifact_uris = {
                    art.get("location", {}).get("uri")
                    for art in target_artifacts_list
                    if art.get("location", {}).get("uri") is not None
                }

                for new_artifact_candidate in current_run_item.get("artifacts", []):
                    candidate_uri = new_artifact_candidate.get("location", {}).get("uri")

                    if candidate_uri is not None:  # Artifact has a URI
                        if candidate_uri not in existing_artifact_uris:
                            target_artifacts_list.append(new_artifact_candidate)
                            existing_artifact_uris.add(candidate_uri)
                        # Else: URI already exists, it's a duplicate based on URI, skip.
                    else:  # Artifact does not have a URI
                        # De-duplicate URI-less artifacts by checking for object equality.
                        # This relies on the artifact (dict) being comparable.
                        if new_artifact_candidate not in target_artifacts_list:
                            target_artifacts_list.append(new_artifact_candidate)
                # --- End of artifact merging logic ---

                target_run_for_tool.setdefault("invocations", []).extend(current_run_item.get("invocations", []))
                target_run_for_tool.setdefault("originalUriBaseIds", {}).update(current_run_item.get("originalUriBaseIds", {}))

                # Merge tool.driver.rules uniquely by ID
                target_driver = target_run_for_tool.setdefault("tool", {}).setdefault("driver", {})
                target_rules_list = target_driver.setdefault("rules", [])
                existing_rule_ids_in_target = {rule.get("id") for rule in target_rules_list if rule.get("id")}

                for new_rule_to_add in current_run_item.get("tool", {}).get("driver", {}).get("rules", []):
                    new_rule_id = new_rule_to_add.get("id")
                    if new_rule_id and new_rule_id not in existing_rule_ids_in_target:
                        target_rules_list.append(new_rule_to_add)
                        existing_rule_ids_in_target.add(new_rule_id)
                    elif not new_rule_id:  # Rule without ID
                        target_rules_list.append(new_rule_to_add)

    final_runs_list = list(merged_runs_map.values())

    merged_sarif_output = {
        "$schema": final_schema or "https://raw.githubusercontent.com/oasis-tcs/sarif-spec/master/Schemata/sarif-schema-2.1.0.json",
        "version": final_version or "2.1.0",
        "runs": final_runs_list
    }

    output_file.parent.mkdir(parents=True, exist_ok=True)
    output_file.write_text(json.dumps(merged_sarif_output, indent=2), encoding='utf-8')
    print(f"[+] Wrote merged SARIF (runs combined by tool) → {output_file}")


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
