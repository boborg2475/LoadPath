# Build, then run the doctest suite. Extra arguments go to doctest, e.g.
#   Scripts/Test.ps1 -tc="Model.*"        run the test cases matching a pattern
#   Scripts/Test.ps1 -ltc                 list test cases
# Test cases named Deferred.*, Parked.*, Slow.* and Measure.* are excluded unless the caller passes its own -tc;
# a caller's -tce alone adds to those exclusions.
# -Jobs N splits the run across N processes, each running a contiguous range of the same filtered
# order (doctest --first/--last), balanced by per-test durations cached in the build folder.
# The default run shards automatically; a -tc run shards only when -Jobs is given. -Jobs 1 runs serially.
# Exit code is doctest's (serial) or 1 when any shard failed or crashed (sharded).
# -Profile builds and runs the phase-timing build (build/msvc-profile, LOADPATH_PROFILE=ON); everything else is the same.
# The Release default run also rebuilds the E2E picture page (Scripts/Visuals.ps1 -NoScreenshots, build/visuals/index.html,
# about 11 s); an E2E case that wrote no picture fails the run. -NoVisuals skips it.
param([ValidateSet("Debug","Release")][string]$Config = "Debug",
      [int]$Jobs = 0,
      [switch]$Profile,
      [switch]$NoVisuals,
      [Parameter(ValueFromRemainingArguments = $true)][string[]]$DoctestArgs)
$ErrorActionPreference = "Stop"
$Root = Split-Path -Parent $PSScriptRoot
$BuildDir = if ($Profile) { "$Root\build\msvc-profile" } else { "$Root\build\msvc" }
& "$PSScriptRoot\Build.ps1" -Config $Config -Profile:$Profile
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
$Exe = "$BuildDir\tests\$Config\loadpath_tests.exe"
if ($null -eq $DoctestArgs) { $DoctestArgs = @() }

function Write-ExcludedCounts {
    foreach ($Prefix in @("Deferred", "Parked", "Slow", "Measure")) {
        $Count = (& $Exe -c "-tc=$Prefix.*" | Out-String) -replace '(?s).*passing the current filters:\s*(\d+).*', '$1'
        if ($Count -ne "0") {
            Write-Output "[Test.ps1] excluded $Count $($Prefix.ToLower()) test case(s) ($Prefix.*); pass -tc to run them."
        }
    }
}

function Invoke-Sharded([string[]]$FilterArgs, [int]$ShardCount, [bool]$ReportExcluded) {
    $Listing = @(& $Exe @FilterArgs -ltc --no-colors) | ForEach-Object { $_.TrimEnd("`r") }
    $Start = [Array]::IndexOf($Listing, '[doctest] listing all test case names')
    $Names = New-Object System.Collections.Generic.List[string]
    for ($I = $Start + 2; $I -lt $Listing.Count -and $Listing[$I] -notmatch '^={20,}$'; $I++) { $Names.Add($Listing[$I]) }
    if ($Names.Count -eq 0) {
        Write-Output "[Test.ps1] no test cases pass the filters."
        return 0
    }
    $ShardCount = [Math]::Max(1, [Math]::Min($ShardCount, $Names.Count))

    $CachePath = "$BuildDir\tests\$Config\test-durations.tsv"
    $Cache = @{}
    if (Test-Path $CachePath) {
        foreach ($Line in Get-Content $CachePath) {
            $Parts = $Line -split "`t", 2
            if ($Parts.Count -eq 2) { $Cache[$Parts[1]] = [double]::Parse($Parts[0], [Globalization.CultureInfo]::InvariantCulture) }
        }
    }
    $Known = @($Names | Where-Object { $Cache.ContainsKey($_) } | ForEach-Object { $Cache[$_] } | Sort-Object)
    $Fallback = if ($Known.Count -gt 0) { $Known[[int][Math]::Floor($Known.Count / 2)] } else { 1.0 }
    $Weights = @($Names | ForEach-Object { if ($Cache.ContainsKey($_)) { [Math]::Max($Cache[$_], 0.001) } else { $Fallback } })
    $Total = ($Weights | Measure-Object -Sum).Sum

    $Ranges = @()
    $First = 1
    $Cumulative = 0.0
    for ($I = 0; $I -lt $Names.Count; $I++) {
        $Cumulative += $Weights[$I]
        $Remaining = $Names.Count - ($I + 1)
        $ShardsLeft = $ShardCount - $Ranges.Count - 1
        $Target = $Total * ($Ranges.Count + 1) / $ShardCount
        if ($ShardsLeft -gt 0 -and ($Cumulative -ge $Target -or $Remaining -eq $ShardsLeft)) {
            $Ranges += ,@($First, ($I + 1))
            $First = $I + 2
        }
    }
    $Ranges += ,@($First, $Names.Count)

    $OutDir = Join-Path ([IO.Path]::GetTempPath()) ("loadpath-shards-" + [Guid]::NewGuid().ToString("N"))
    New-Item -ItemType Directory -Path $OutDir | Out-Null
    $Quoted = @($FilterArgs | ForEach-Object { if ($_ -match '\s') { '"' + $_ + '"' } else { $_ } })
    $Clock = [Diagnostics.Stopwatch]::StartNew()
    $Processes = @()
    for ($S = 0; $S -lt $Ranges.Count; $S++) {
        $ShardArgs = $Quoted + @("-f=$($Ranges[$S][0])", "-l=$($Ranges[$S][1])", "-d", "--no-colors")
        $P = Start-Process -FilePath $Exe -ArgumentList $ShardArgs -NoNewWindow -PassThru `
            -RedirectStandardOutput "$OutDir\shard$S.out" -RedirectStandardError "$OutDir\shard$S.err"
        $null = $P.Handle
        $Processes += $P
    }
    foreach ($P in $Processes) { $P.WaitForExit() }
    $Clock.Stop()

    $Executed = 0; $FailedCases = 0; $Asserts = 0; $AssertsPassed = 0; $AssertsFailed = 0
    $Broken = $false
    $ShardSkipped = 0
    for ($S = 0; $S -lt $Ranges.Count; $S++) {
        $Lines = @(Get-Content "$OutDir\shard$S.out") + @(Get-Content "$OutDir\shard$S.err")
        $Ran = @($Lines | Where-Object { $_ -match '^\d+\.\d+ s: ' })
        foreach ($Line in $Ran) {
            if ($Line -match '^(\d+\.\d+) s: (.+)$') { $Cache[$Matches[2]] = [double]::Parse($Matches[1], [Globalization.CultureInfo]::InvariantCulture) }
        }
        $Executed += $Ran.Count
        $ShardFailed = 0
        $Summary = $false
        foreach ($Line in $Lines) {
            if ($Line -match '^\[doctest\] test cases:.*\|\s*(\d+) failed') { $ShardFailed = [int]$Matches[1] }
            if ($Line -match '^\[doctest\] assertions:\s*(\d+)\s*\|\s*(\d+) passed\s*\|\s*(\d+) failed') {
                $Asserts += [int]$Matches[1]; $AssertsPassed += [int]$Matches[2]; $AssertsFailed += [int]$Matches[3]
            }
            if ($Line -match '^\[doctest\] Status: ') { $Summary = $true }
        }
        $FailedCases += $ShardFailed
        foreach ($Line in $Lines) { if ($Line -match '^\[doctest\] test cases:.*\|\s*(\d+) skipped') { $ShardSkipped = [int]$Matches[1] } }

        $Block = New-Object System.Collections.Generic.List[string]
        foreach ($Line in ($Lines + @('=' * 79))) {
            if ($Line -match '^={20,}$') {
                $Content = @($Block | Where-Object {
                    $_ -notmatch '^\s*$' -and $_ -notmatch '^\S.*\(\d+\):$' -and $_ -notmatch '^TEST CASE:' -and
                    $_ -notmatch '^\d+\.\d+ s: ' -and $_ -notmatch '^\[doctest\]' })
                if ($Content.Count -gt 0) {
                    Write-Output ('=' * 79)
                    $Block | Where-Object { $_ -notmatch '^\d+\.\d+ s: ' } | Write-Output
                }
                $Block.Clear()
            } else {
                $Block.Add($Line)
            }
        }

        $Expected = $Ranges[$S][1] - $Ranges[$S][0] + 1
        if (-not $Summary -or $Ran.Count -ne $Expected -or ($Processes[$S].ExitCode -ne 0 -and $ShardFailed -eq 0)) {
            $Broken = $true
            $Last = if ($Ran.Count -gt 0) { ($Ran[-1] -replace '^\d+\.\d+ s: ', '') } else { "(none)" }
            Write-Output "[Test.ps1] shard $S (tests $($Ranges[$S][0])..$($Ranges[$S][1])) crashed or stopped early: exit $($Processes[$S].ExitCode), ran $($Ran.Count) of $Expected, last finished $Last"
            Get-Content "$OutDir\shard$S.out" -Tail 20 | Write-Output
        }
    }
    Remove-Item -Recurse -Force $OutDir

    $Cache.GetEnumerator() | Sort-Object Name | ForEach-Object { "{0}`t{1}" -f $_.Value.ToString("R", [Globalization.CultureInfo]::InvariantCulture), $_.Name } |
        Set-Content -Encoding ascii $CachePath

    $Skipped = $ShardSkipped + $Names.Count - $Executed
    Write-Output ('=' * 79)
    Write-Output "[doctest] test cases: $Executed | $($Executed - $FailedCases) passed | $FailedCases failed | $Skipped skipped"
    Write-Output "[doctest] assertions: $Asserts | $AssertsPassed passed | $AssertsFailed failed |"
    $Ok = ($FailedCases -eq 0 -and $AssertsFailed -eq 0 -and -not $Broken -and $Executed -eq $Names.Count)
    Write-Output ("[doctest] Status: " + $(if ($Ok) { "SUCCESS!" } else { "FAILURE!" }))
    Write-Output ("[Test.ps1] {0} shards, {1:N1} s wall" -f $Ranges.Count, $Clock.Elapsed.TotalSeconds)
    if ($ReportExcluded -and $Skipped -gt 0) { Write-ExcludedCounts }
    if ($Ok) { return 0 } else { return 1 }
}

$CallerIncludes = @($DoctestArgs | Where-Object { $_ -match '^-+(dt-)?(tc|test-case)(=|$)' })
$IsQuery = @($DoctestArgs | Where-Object { $_ -match '^-+(dt-)?(ltc|lts|ltr|c|count|h|help|\?|v|version|list-[a-z-]+|r|reporters|f|first|l|last)(=|$)' }).Count -gt 0
$AutoJobs = [Math]::Max(1, [Math]::Min(8, [int][Math]::Floor([Environment]::ProcessorCount / 2)))

if ($CallerIncludes.Count -gt 0) {
    if ($Jobs -gt 1 -and -not $IsQuery) {
        $Result = @(Invoke-Sharded $DoctestArgs $Jobs $false)
        $Result[0..($Result.Count - 2)] | Write-Output
        exit $Result[-1]
    }
    & $Exe @DoctestArgs
    exit $LASTEXITCODE
}
$ExcludePattern = '^-+(dt-)?(tce|test-case-exclude)=(.*)$'
$CallerExcludes = @($DoctestArgs | Where-Object { $_ -match $ExcludePattern } | ForEach-Object { $_ -replace $ExcludePattern, '$3' })
$Excludes = (@($CallerExcludes) + "Deferred.*,Parked.*,Slow.*,Measure.*") -join ','
$Passed = @($DoctestArgs | Where-Object { $_ -notmatch $ExcludePattern })
function Invoke-Visuals([int]$Code) {
    if ($IsQuery -or $NoVisuals -or $Profile -or $Config -ne "Release") { return $Code }
    $ErrorActionPreference = "Continue"
    $Lines = @(& powershell -NoProfile -ExecutionPolicy Bypass -File "$PSScriptRoot\Visuals.ps1" -Config Release -NoScreenshots 2>&1 | ForEach-Object { "$_" })
    $VisualsCode = $LASTEXITCODE
    $Lines | Where-Object { $_ -match '^\[Visuals\.ps1\]|^  |passes: move it out of Deferred' } | Out-Host
    if ($VisualsCode -ne 0 -and $Code -eq 0) { return 1 }
    return $Code
}

if ($Jobs -eq 0) { $Jobs = $AutoJobs }
if ($Jobs -gt 1 -and -not $IsQuery) {
    $Result = @(Invoke-Sharded (@($Passed) + "-tce=$Excludes") $Jobs $true)
    $Result[0..($Result.Count - 2)] | Write-Output
    exit (Invoke-Visuals $Result[-1])
}
& $Exe @Passed "-tce=$Excludes" | Tee-Object -Variable Output
$Code = $LASTEXITCODE
if ((($Output | Out-String) -match '\|\s*[1-9][0-9]*\s+skipped')) { Write-ExcludedCounts }
exit (Invoke-Visuals $Code)
