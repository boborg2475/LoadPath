# Prove a test bites: apply one find/replace to a file in the scratch worktree, build, run the named tests,
# report CAUGHT or SURVIVED, and restore the file byte for byte. This tree is never touched.
#   Scripts/Mutate.ps1 -File src/StaticsCommon.cpp -Find "!Model.IsLive(" -Replace "Model.IsLive(" -Tests "Statics.StaleLoads.*"
# -Find must occur exactly once in the file. -Tests is a doctest -tc filter (comma-separated patterns).
# The scratch worktree is synced to this working tree first unless -NoSync is given.
# Exit code: 0 caught, 1 survived, 2 usage or setup error, 3 the mutant does not compile.
param([Parameter(Mandatory = $true)][string]$File,
      [Parameter(Mandatory = $true)][string]$Find,
      [Parameter(Mandatory = $true)][AllowEmptyString()][string]$Replace,
      [Parameter(Mandatory = $true)][string]$Tests,
      [ValidateSet("Debug","Release")][string]$Config = "Release",
      [string]$Scratch = "",
      [switch]$NoSync)
$ErrorActionPreference = "Stop"
$Root = Split-Path -Parent $PSScriptRoot
if ($Scratch -eq "") { $Scratch = Join-Path (Split-Path -Parent $Root) "LoadPath-scratch" }
try {
    if (-not $NoSync) { & "$PSScriptRoot\Scratch.ps1" -Path $Scratch }
    $Target = Join-Path $Scratch $File
    if (-not (Test-Path $Target)) { throw "$Target does not exist" }
    $Original = [IO.File]::ReadAllBytes($Target)
    $HasBom = $Original.Length -ge 3 -and $Original[0] -eq 0xEF -and $Original[1] -eq 0xBB -and $Original[2] -eq 0xBF
    $Text = [IO.File]::ReadAllText($Target)
    $Count = 0
    $At = $Text.IndexOf($Find, [StringComparison]::Ordinal)
    while ($At -ge 0) { $Count++; $At = $Text.IndexOf($Find, $At + 1, [StringComparison]::Ordinal) }
    if ($Count -ne 1) { throw "-Find occurs $Count times in $File; it must occur exactly once" }
} catch {
    Write-Output "[Mutate.ps1] $($_.Exception.Message)"
    exit 2
}

$Verdict = 2
try {
    $Mutant = $Text.Replace($Find, $Replace)
    [IO.File]::WriteAllText($Target, $Mutant, (New-Object System.Text.UTF8Encoding($HasBom)))
    New-Item -ItemType Directory -Force -Path "$Scratch\build" | Out-Null
    Write-Output "[Mutate.ps1] $File`: '$Find' -> '$Replace'"
    & "$Scratch\Scripts\Build.ps1" -Config $Config *> "$Scratch\build\mutate-build.log"
    if ($LASTEXITCODE -ne 0) {
        Select-String -Path "$Scratch\build\mutate-build.log" -Pattern 'error' | Select-Object -First 5 | ForEach-Object { $_.Line }
        Write-Output "[Mutate.ps1] DOES NOT COMPILE: rewrite the mutant (full log: $Scratch\build\mutate-build.log)"
        $Verdict = 3
    } else {
        $Output = & "$Scratch\build\msvc\tests\$Config\loadpath_tests.exe" "-tc=$Tests" --no-colors
        $Code = $LASTEXITCODE
        $Output | Where-Object { $_ -match '^\[doctest\] (test cases|assertions)' } | Write-Output
        if ($Output -match 'test cases:\s*0 \|') {
            Write-Output "[Mutate.ps1] no test case matches -Tests '$Tests'"
            $Verdict = 2
        } elseif ($Code -ne 0) {
            Write-Output "[Mutate.ps1] CAUGHT"
            $Verdict = 0
        } else {
            Write-Output "[Mutate.ps1] SURVIVED"
            $Verdict = 1
        }
    }
} finally {
    [IO.File]::WriteAllBytes($Target, $Original)
}
exit $Verdict
