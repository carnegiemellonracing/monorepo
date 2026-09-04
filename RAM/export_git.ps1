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
#include <stdint.h>
const uint32_t GIT_INFO = 0x$GitInfoHex;
const uint8_t IS_UNCOMMITTED = $IsDirty;
"@

$FilePath = "./Inc/gitcommit.h"

# Only write the file if its contents have changed
$ExistingContent = if (Test-Path $FilePath) {
    Get-Content -Raw -Path $FilePath
} else {
    $null
}

if ($ExistingContent -ne $Content) {
    Write-Output $Content | Out-File -Encoding ASCII -FilePath $FilePath -NoNewline
}