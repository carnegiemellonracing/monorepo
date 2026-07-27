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

if ($diffContent -and $diffContent.Trim().Length -gt 0) {
    $IsDirty = 1

    # Hash the actual diff/untracked content, not just "is it dirty"
    $bytes  = [System.Text.Encoding]::UTF8.GetBytes($diffContent)
    $sha256 = [System.Security.Cryptography.SHA256]::Create()
    $hashBytes = $sha256.ComputeHash($bytes)

    # Take first 4 bytes as a 32-bit content hash
    $DiffHash = -join ($hashBytes[0..3] | ForEach-Object { $_.ToString("x2") })

    # Fold it together with the base commit hash so GIT_INFO reflects
    # both "which commit" and "what changed" — different edits -> different value
    $baseVal = [Convert]::ToUInt32($Hash, 16)
    $diffVal = [Convert]::ToUInt32($DiffHash, 16)
    $combined = $baseVal -bxor $diffVal
    $GitInfoHex = $combined.ToString("x8")
}

$Content = @"
#include <stdint.h>
const uint32_t GIT_INFO = 0x$GitInfoHex;
const uint8_t IS_UNCOMMITTED = $IsDirty;
"@
Write-Output $Content | Out-File -Encoding ASCII -FilePath ./Inc/gitcommit.h