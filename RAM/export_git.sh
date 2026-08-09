#!/bin/bash
set -euo pipefail

# --- Base commit hash (clean state) ---
HASH=$(git rev-parse --short=8 HEAD)

# --- Gather actual working-tree changes: tracked diffs + untracked file contents ---
DIFF_CONTENT=$(git diff HEAD --no-color)

while IFS= read -r f; do
  if [ -f "$f" ]; then
    DIFF_CONTENT+=$'\n'"--- untracked: $f ---"$'\n'
    DIFF_CONTENT+=$(cat "$f" 2>/dev/null || true)
  fi
done < <(git ls-files --others --exclude-standard)

IS_DIRTY=0
GIT_INFO_HEX="$HASH"

cat > ./Inc/gitcommit.h <<EOF
#include <stdint.h>
const uint32_t GIT_INFO = 0x$GIT_INFO_HEX;
const uint8_t IS_UNCOMMITTED = $IS_DIRTY;
EOF