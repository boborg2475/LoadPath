# Sync the persistent scratch worktree (default ..\LoadPath-scratch) and print its path.
# It keeps its own build folder, so reviews, mutants and measurements build incrementally there.
#   Scripts/Scratch.ps1                 the working tree: HEAD plus uncommitted and untracked changes
#   Scripts/Scratch.ps1 -Rev a1b2c3d    exactly that commit
# Only files whose content differs are rewritten, with fresh timestamps, so every build is correct and incremental.
# All git commands that change files run inside the scratch worktree, never in this tree.
param([string]$Rev = "",
      [string]$Path = "")
$ErrorActionPreference = "Stop"
$Root = Split-Path -Parent $PSScriptRoot
if ($Path -eq "") { $Path = Join-Path (Split-Path -Parent $Root) "LoadPath-scratch" }

function Invoke-Git([string]$Dir, [string[]]$GitArgs) {
    & git -C $Dir @GitArgs
    if ($LASTEXITCODE -ne 0) { throw "git $($GitArgs -join ' ') failed in $Dir" }
}

if (-not (Test-Path (Join-Path $Path ".git"))) {
    if (Test-Path $Path) { throw "$Path exists but is not a worktree of this repository; move it aside first." }
    Invoke-Git $Root @("worktree", "add", "--detach", $Path, "HEAD")
}
$Main = (Resolve-Path (& git -C $Root rev-parse --git-common-dir)).Path
$Theirs = (Resolve-Path (& git -C $Path rev-parse --git-common-dir)).Path
if ($Main -ne $Theirs) { throw "$Path is a worktree of a different repository." }

$Target = if ($Rev -eq "") { "HEAD" } else { $Rev }
$Commit = (& git -C $Root rev-parse --verify "$Target^{commit}")
if ($LASTEXITCODE -ne 0) { throw "unknown revision $Target" }
Invoke-Git $Path @("checkout", "--quiet", "--detach", "--force", $Commit)
Invoke-Git $Path @("clean", "--quiet", "-fd")

$Overlaid = 0
if ($Rev -eq "") {
    $Diff = Join-Path ([IO.Path]::GetTempPath()) ("loadpath-scratch-" + [Guid]::NewGuid().ToString("N") + ".diff")
    & git -C $Root diff --binary HEAD --output=$Diff
    if ($LASTEXITCODE -ne 0) { throw "git diff failed" }
    if ((Get-Item $Diff).Length -gt 0) {
        Invoke-Git $Path @("apply", "--whitespace=nowarn", $Diff)
        $Overlaid += @(& git -C $Root diff --name-only HEAD).Count
    }
    Remove-Item $Diff
    foreach ($File in @(& git -C $Root ls-files --others --exclude-standard)) {
        $Destination = Join-Path $Path $File
        New-Item -ItemType Directory -Force -Path (Split-Path -Parent $Destination) | Out-Null
        [IO.File]::WriteAllBytes($Destination, [IO.File]::ReadAllBytes((Join-Path $Root $File)))
        $Overlaid++
    }
}
$Short = (& git -C $Root rev-parse --short $Commit)
if ($Rev -eq "") {
    Write-Output "[Scratch.ps1] $Path at $Short plus $Overlaid uncommitted or untracked file(s)"
} else {
    Write-Output "[Scratch.ps1] $Path at $Short"
}
