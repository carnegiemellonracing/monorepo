"""
can_id_lint.py

A practical CAN ID linter for can_ids.h.

Features:
- Parses enum-style CAN IDs: NAME = VALUE
- Supports simple expressions:
    - hex/decimal literals: 0x310, 123
    - references to earlier constants (#define NAME <number> or enum entries)
    - + and - operators, parentheses
- Ignores comments (// and /* ... */) robustly enough for typical headers
- Checks:
    1) ID range (default: standard 11-bit CAN: 0x000..0x7FF)
    2) Duplicate numeric IDs (with optional allowlist)
    3) Duplicate names
    4) Config packet structure for DIM/CDC config IDs (0..NUM_CONFIG_PACKETS-1), per driver

Usage:
    python3 can_id_lint.py can_ids.h
    python3 can_id_lint.py can_ids.h --extended
    python3 can_id_lint.py can_ids.h --allow-dup 0x310 --allow-dup 0x311
    python3 can_id_lint.py can_ids.h --allow-dup 0x310 0x311 0x7F0
"""

from __future__ import annotations

import argparse
import ast
import os
import re
import sys
from dataclasses import dataclass
from typing import Dict, List, Optional, Set, Tuple


# -----------------------------
# Comment stripping
# -----------------------------
def strip_comments(text: str) -> str:
    """
    Remove // comments and /* ... */ block comments from a whole file string.
    This is a basic stripper intended for C headers (not a full C parser).
    """
    # Remove block comments first (including multiline)
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.DOTALL)
    # Remove line comments
    text = re.sub(r"//.*?$", "", text, flags=re.MULTILINE)
    return text


# -----------------------------
# Safe expression evaluation
# -----------------------------
class SafeEvalError(Exception):
    pass


_ALLOWED_AST_NODES = (
    ast.Expression,
    ast.BinOp,
    ast.UnaryOp,
    ast.Add,
    ast.Sub,
    ast.USub,
    ast.UAdd,
    ast.Constant,
    ast.Name,
    ast.ParenExpr if hasattr(ast, "ParenExpr") else ast.AST,  # py>=3.12 has ParenExpr
)


def safe_eval_int(expr: str, symbols: Dict[str, int]) -> int:
    """
    Evaluate a restricted integer expression using Python AST:
    - literals: 0x..., decimal
    - names: looked up in symbols
    - + and - operators
    - parentheses
    """
    expr = expr.strip()
    if not expr:
        raise SafeEvalError("empty expression")

    try:
        tree = ast.parse(expr, mode="eval")
    except SyntaxError as e:
        raise SafeEvalError(f"syntax error: {e}") from e

    def check(node: ast.AST) -> None:
        # Disallow everything not explicitly allowed
        if not isinstance(node, _ALLOWED_AST_NODES):
            raise SafeEvalError(f"disallowed expression element: {type(node).__name__}")
        for child in ast.iter_child_nodes(node):
            check(child)

    def eval_node(node: ast.AST) -> int:
        if isinstance(node, ast.Expression):
            return eval_node(node.body)
        if isinstance(node, ast.Constant):
            if isinstance(node.value, (int,)):
                return int(node.value)
            raise SafeEvalError("non-integer constant")
        if isinstance(node, ast.Name):
            if node.id not in symbols:
                raise SafeEvalError(f"unknown symbol: {node.id}")
            return int(symbols[node.id])
        if isinstance(node, ast.UnaryOp):
            v = eval_node(node.operand)
            if isinstance(node.op, ast.UAdd):
                return +v
            if isinstance(node.op, ast.USub):
                return -v
            raise SafeEvalError("unsupported unary op")
        if isinstance(node, ast.BinOp):
            a = eval_node(node.left)
            b = eval_node(node.right)
            if isinstance(node.op, ast.Add):
                return a + b
            if isinstance(node.op, ast.Sub):
                return a - b
            raise SafeEvalError("unsupported binary op")
        # Parentheses in AST are represented structurally; no special case needed.
        raise SafeEvalError(f"unsupported node: {type(node).__name__}")

    check(tree)
    return eval_node(tree)


# -----------------------------
# Parsing
# -----------------------------
_DEFINE_RE = re.compile(r"^\s*#\s*define\s+(\w+)\s+(.+?)\s*$", re.MULTILINE)

# Matches enum assignments like: NAME = 0x123,  NAME = SOME_CONST + 1,
_ENUM_ASSIGN_RE = re.compile(
    r"""
    ^\s*
    (?P<name>[A-Za-z_]\w*)
    \s*=\s*
    (?P<expr>[^,}]+)
    """,
    re.VERBOSE | re.MULTILINE,
)

# DIM/CDC config naming pattern
_CONFIG_NAME_RE = re.compile(r"^CMR_CANID_(DIM|CDC)_CONFIG(\d+)_DRV(\d+)$")


@dataclass(frozen=True)
class Defn:
    name: str
    value: int
    expr: str
    kind: str  # "define" or "enum"


def parse_can_ids(path: str) -> Tuple[Dict[str, int], List[Defn]]:
    """
    Returns:
      - symbols: mapping of name -> integer value for #define constants and enum entries
      - defns: ordered list of resolved definitions
    """
    if not os.path.exists(path):
        raise FileNotFoundError(path)

    raw = open(path, "r", encoding="utf-8", errors="ignore").read()
    text = strip_comments(raw)

    symbols: Dict[str, int] = {}
    defns: List[Defn] = []

    # 1) Parse #defines first (numeric or simple expressions referencing earlier defines)
    #    We do a few passes to resolve forward-ish references only when possible.
    defines = [(m.group(1), m.group(2).strip()) for m in _DEFINE_RE.finditer(text)]

    unresolved = defines[:]
    progress = True
    while progress and unresolved:
        progress = False
        still: List[Tuple[str, str]] = []
        for name, expr in unresolved:
            # Skip macro function-like defines: #define FOO(x) ...
            if "(" in name:
                still.append((name, expr))
                continue
            try:
                val = safe_eval_int(expr, symbols)
            except SafeEvalError:
                still.append((name, expr))
                continue
            symbols[name] = val
            defns.append(Defn(name=name, value=val, expr=expr, kind="define"))
            progress = True
        unresolved = still

    # 2) Parse enum assignments (order matters; enum entries can reference earlier ones/defines)
    for m in _ENUM_ASSIGN_RE.finditer(text):
        name = m.group("name")
        expr = m.group("expr").strip()

        try:
            val = safe_eval_int(expr, symbols)
        except SafeEvalError:
            # If it looks like a plain hex/dec but failed (rare), skip; else warn later.
            continue

        # Record
        symbols[name] = val
        defns.append(Defn(name=name, value=val, expr=expr, kind="enum"))

    return symbols, defns


# -----------------------------
# Lint checks
# -----------------------------
def check_range(defns: List[Defn], max_id: int) -> List[str]:
    issues = []
    for d in defns:
        if d.kind != "enum":
            continue
        if d.value < 0 or d.value > max_id:
            issues.append(
                f"[RANGE] {d.name} = 0x{d.value:X} ({d.value}) outside 0x0..0x{max_id:X}"
            )
    return issues


def check_duplicate_names(defns: List[Defn]) -> List[str]:
    issues = []
    seen: Set[str] = set()
    for d in defns:
        if d.kind != "enum":
            continue
        if d.name in seen:
            issues.append(f"[DUP_NAME] enum name repeated: {d.name}")
        seen.add(d.name)
    return issues


def check_duplicate_values(defns: List[Defn], allow_dups: Set[int]) -> List[str]:
    issues = []
    by_val: Dict[int, List[str]] = {}
    for d in defns:
        if d.kind != "enum":
            continue
        by_val.setdefault(d.value, []).append(d.name)

    for val, names in sorted(by_val.items(), key=lambda x: x[0]):
        if len(names) > 1 and val not in allow_dups:
            issues.append(
                f"[DUP_VALUE] 0x{val:X} ({val}) shared by: {', '.join(sorted(names))}"
            )
    return issues


def check_config_packets(defns: List[Defn], symbols: Dict[str, int]) -> List[str]:
    """
    Validates:
      - NUM_CONFIG_PACKETS exists
      - For each driver observed, each of DIM and CDC has CONFIG0..CONFIG(N-1)
      - IDs are strictly increasing within each driver block:
          DIM0..DIM(N-1), CDC0..CDC(N-1)
      - Optional: blocks increase across drivers (not required but can be useful)
    """
    issues = []
    if "NUM_CONFIG_PACKETS" not in symbols:
        issues.append("[CONFIG] NUM_CONFIG_PACKETS not found (#define).")
        return issues

    n = symbols["NUM_CONFIG_PACKETS"]
    if not isinstance(n, int) or n <= 0 or n > 64:
        issues.append(f"[CONFIG] NUM_CONFIG_PACKETS has suspicious value: {n}")
        return issues

    # Collect config items
    configs: Dict[Tuple[str, int], Dict[int, Tuple[str, int]]] = {}
    # key: (kind DIM/CDC, drv) -> {idx -> (name, value)}
    for d in defns:
        if d.kind != "enum":
            continue
        m = _CONFIG_NAME_RE.match(d.name)
        if not m:
            continue
        kind = m.group(1)  # DIM/CDC
        idx = int(m.group(2))
        drv = int(m.group(3))
        configs.setdefault((kind, drv), {})[idx] = (d.name, d.value)

    # If no configs, nothing to check
    if not configs:
        return issues

    # Determine drivers seen
    drivers = sorted({drv for (_, drv) in configs.keys()})

    for drv in drivers:
        for kind in ("DIM", "CDC"):
            key = (kind, drv)
            if key not in configs:
                issues.append(f"[CONFIG] Missing all {kind} configs for DRV{drv}.")
                continue

            idx_map = configs[key]
            missing = [i for i in range(n) if i not in idx_map]
            extra = [i for i in sorted(idx_map.keys()) if i >= n]

            if missing:
                issues.append(
                    f"[CONFIG] {kind} DRV{drv} missing indices: {missing} (expected 0..{n-1})"
                )
            if extra:
                issues.append(
                    f"[CONFIG] {kind} DRV{drv} has extra indices: {extra} (NUM_CONFIG_PACKETS={n})"
                )

            # Check ascending values for expected indices
            seq = [idx_map[i][1] for i in range(n) if i in idx_map]
            if seq != sorted(seq):
                issues.append(f"[CONFIG] {kind} DRV{drv} IDs not ascending by index.")

        # Check DIM block then CDC block ordering if both exist and complete
        dim = configs.get(("DIM", drv), {})
        cdc = configs.get(("CDC", drv), {})
        if all(i in dim for i in range(n)) and all(i in cdc for i in range(n)):
            dim_max = max(dim[i][1] for i in range(n))
            cdc_min = min(cdc[i][1] for i in range(n))
            if not (cdc_min > dim_max):
                issues.append(f"[CONFIG] DRV{drv} CDC config IDs should come after DIM config IDs.")

    return issues


def print_summary(defns: List[Defn]) -> None:
    enums = [d for d in defns if d.kind == "enum"]
    print(f"Resolved {len(enums)} enum assignments.")
    print(f"Resolved {len([d for d in defns if d.kind=='define'])} #define constants (numeric/simple expr).")


def main() -> int:
    ap = argparse.ArgumentParser(description="Lint can_ids.h for CAN ID sanity.")
    ap.add_argument("path", nargs="?", default="can_ids.h", help="Path to can_ids.h")
    ap.add_argument("--extended", action="store_true", help="Allow 29-bit extended IDs (<= 0x1FFFFFFF)")
    ap.add_argument(
        "--allow-dup",
        nargs="*",
        default=[],
        help="Numeric IDs allowed to be duplicated (e.g., 0x310 0x311 0x7F0)",
    )
    ap.add_argument("--no-config-check", action="store_true", help="Disable DIM/CDC config packet checks.")
    args = ap.parse_args()

    max_id = 0x1FFFFFFF if args.extended else 0x7FF

    allow_dups: Set[int] = set()
    for s in args.allow_dup:
        try:
            allow_dups.add(int(s, 0))
        except ValueError:
            print(f"Invalid --allow-dup value: {s}", file=sys.stderr)
            return 2

    try:
        symbols, defns = parse_can_ids(args.path)
    except FileNotFoundError:
        print(f"Error: file not found: {args.path}", file=sys.stderr)
        return 2

    print_summary(defns)
    print("-" * 60)

    issues: List[str] = []
    issues += check_range(defns, max_id=max_id)
    issues += check_duplicate_names(defns)
    issues += check_duplicate_values(defns, allow_dups=allow_dups)
    if not args.no_config_check:
        issues += check_config_packets(defns, symbols)

    if issues:
        for msg in issues:
            print(msg)
        print("-" * 60)
        print(f"FAIL: {len(issues)} issue(s) found.")
        return 1

    print("PASS: no issues found.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
