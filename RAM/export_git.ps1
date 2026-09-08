# --- Base commit hash (clean state) ---
$Hash = git rev-parse --short=8 HEAD

# --- Gather actual working-tree changes: tracked diffs + untracked file contents ---
$diffContent = git diff HEAD --no-color
$untrackedFiles = git ls-files --others --exclude-standard

foreach ($f in $untrackedFiles) {
    if (Test-Path $f -PathType Leaf) {
        $diffContent += "`n--- untracked: $f ---`n"
        $diffContent += (Get-Content -Raw -Path $f -ErrorAction SilentlyContinue)
    }
}

$IsDirty = 0
$GitInfoHex = $Hash

$Content = @"
#pragma once
#ifndef GIT_H
#define GIT_H
#include <stdint.h>
#define GIT_INFO ((uint32_t)0x$GitInfoHex)
#define IS_UNCOMMITTED ((uint8_t)$IsDirty)
#endif
"@
Write-Output $Content | Out-File -Encoding ASCII -FilePath ./Inc/gitcommit.h

