# Build, then run the doctest suite. Extra arguments go to doctest, e.g.
#   Scripts/Test.ps1 -tc="Model.*"        run the test cases matching a pattern
#   Scripts/Test.ps1 -ltc                 list test cases
# Test cases named Deferred.*, Slow.* and Measure.* are excluded unless the caller passes its own -tc or -tce.
# Exit code is doctest's: non-zero when any assertion fails.
param([ValidateSet("Debug","Release")][string]$Config = "Debug",
      [Parameter(ValueFromRemainingArguments = $true)][string[]]$DoctestArgs)
$ErrorActionPreference = "Stop"
$Root = Split-Path -Parent $PSScriptRoot
& "$PSScriptRoot\Build.ps1" -Config $Config
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
$Exe = "$Root\build\msvc\tests\$Config\loadpath_tests.exe"
if ($null -eq $DoctestArgs) { $DoctestArgs = @() }
$CallerFilters = @($DoctestArgs | Where-Object { $_ -match '^--?(dt-)?(tc|tce|test-case|test-case-exclude)(=|$)' })
if ($CallerFilters.Count -gt 0) {
    & $Exe @DoctestArgs
    exit $LASTEXITCODE
}
& $Exe @DoctestArgs "-tce=Deferred.*,Slow.*,Measure.*" | Tee-Object -Variable Output
$Code = $LASTEXITCODE
if ((($Output | Out-String) -match '\|\s*[1-9][0-9]*\s+skipped')) {
    foreach ($Prefix in @("Deferred", "Slow", "Measure")) {
        $Count = (& $Exe -c "-tc=$Prefix.*" | Out-String) -replace '(?s).*passing the current filters:\s*(\d+).*', '$1'
        if ($Count -ne "0") {
            Write-Output "[Test.ps1] excluded $Count $($Prefix.ToLower()) test case(s) ($Prefix.*); pass -tc or -tce to run them."
        }
    }
}
exit $Code
