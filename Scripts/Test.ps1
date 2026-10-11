# Build, then run the doctest suite. Extra arguments go to doctest, e.g.
#   Scripts/Test.ps1 -tc="Model.*"        run the test cases matching a pattern
#   Scripts/Test.ps1 -ltc                 list test cases
# Exit code is doctest's: non-zero when any assertion fails.
param([ValidateSet("Debug","Release")][string]$Config = "Debug",
      [Parameter(ValueFromRemainingArguments = $true)][string[]]$DoctestArgs)
$ErrorActionPreference = "Stop"
$Root = Split-Path -Parent $PSScriptRoot
& "$PSScriptRoot\Build.ps1" -Config $Config
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
& "$Root\build\msvc\tests\$Config\loadpath_tests.exe" @DoctestArgs
exit $LASTEXITCODE
