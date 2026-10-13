# Build, then run the doctest suite. Extra arguments go to doctest, e.g.
#   Scripts/Test.ps1 -tc="Model.*"        run the test cases matching a pattern
#   Scripts/Test.ps1 -ltc                 list test cases
# Test cases named Deferred.*, Slow.* and Measure.* are excluded unless the caller passes its own -tc;
# a caller's -tce alone adds to those exclusions.
# Exit code is doctest's: non-zero when any assertion fails.
param([ValidateSet("Debug","Release")][string]$Config = "Debug",
      [Parameter(ValueFromRemainingArguments = $true)][string[]]$DoctestArgs)
$ErrorActionPreference = "Stop"
$Root = Split-Path -Parent $PSScriptRoot
& "$PSScriptRoot\Build.ps1" -Config $Config
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
$Exe = "$Root\build\msvc\tests\$Config\loadpath_tests.exe"
if ($null -eq $DoctestArgs) { $DoctestArgs = @() }
$CallerIncludes = @($DoctestArgs | Where-Object { $_ -match '^--?(dt-)?(tc|test-case)(=|$)' })
if ($CallerIncludes.Count -gt 0) {
    & $Exe @DoctestArgs
    exit $LASTEXITCODE
}
$ExcludePattern = '^--?(dt-)?(tce|test-case-exclude)=(.*)$'
$CallerExcludes = @($DoctestArgs | Where-Object { $_ -match $ExcludePattern } | ForEach-Object { $_ -replace $ExcludePattern, '$3' })
$Excludes = (@($CallerExcludes) + "Deferred.*,Slow.*,Measure.*") -join ','
$Passed = @($DoctestArgs | Where-Object { $_ -notmatch $ExcludePattern })
& $Exe @Passed "-tce=$Excludes" | Tee-Object -Variable Output
$Code = $LASTEXITCODE
if ((($Output | Out-String) -match '\|\s*[1-9][0-9]*\s+skipped')) {
    foreach ($Prefix in @("Deferred", "Slow", "Measure")) {
        $Count = (& $Exe -c "-tc=$Prefix.*" | Out-String) -replace '(?s).*passing the current filters:\s*(\d+).*', '$1'
        if ($Count -ne "0") {
            Write-Output "[Test.ps1] excluded $Count $($Prefix.ToLower()) test case(s) ($Prefix.*); pass -tc to run them."
        }
    }
}
exit $Code
