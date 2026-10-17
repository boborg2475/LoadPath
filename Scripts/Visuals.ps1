# Build the end-to-end picture pages. Every E2E case (each E2E.* case, plus every case in tests/EndToEndTest.cpp
# and tests/ScenarioFilesTest.cpp) runs in its own loadpath_tests process with LOADPATH_VISUALS set, and writes
# one JSON file per picture (tests/VisualSupport.h); this script embeds them in viewer/index.html (the animated
# 3D viewer) and, unless -NoScreenshots, captures each entry's before and after frames with headless Edge into
# viewer/screenshots.html. Both pages are self-contained files.
#   Scripts/Visuals.ps1                  build Release, run every E2E case with recording on, write both pages
#   Scripts/Visuals.ps1 -NoScreenshots   the 3D viewer only
#   . Scripts/Visuals.ps1 -FunctionsOnly define the functions and return (Scripts/Visuals.Tests.ps1)
# Each entry is marked passed, failed or deferred from its case's exit code (a red Deferred.* case is deferred).
# Exit code: 1 when an executed E2E case wrote no picture (named; the page is still written), else 0.
param([ValidateSet("Debug","Release")][string]$Config = "Release",
      [string]$Out = "",
      [switch]$NoScreenshots,
      [int]$Parallel = 4,
      [switch]$Profile,
      [switch]$FunctionsOnly)
$ErrorActionPreference = "Stop"
$Root = Split-Path -Parent $PSScriptRoot
$BuildDir = if ($Profile) { "$Root\build\msvc-profile" } else { "$Root\build\msvc" }
if ($Out -eq "") { $Out = "$Root\build\visuals" }
$Utf8 = New-Object System.Text.UTF8Encoding($false)
$Dash = [string][char]0x2013; $Dot = [string][char]0xB7

# G2 to G5 of the scenario workbench plan: they run zero files legitimately or draw nothing.
$PictureGuardExempt = @("E2E.Scenarios.EverySlowInDebugScenarioFileMeetsItsExpectation", "Slow.Scenarios.EverySlowScenarioFileMeetsItsExpectation",
                        "Deferred.Scenarios.EveryDeferredScenarioFileMeetsItsExpectation", "E2E.Scenarios.FileNamesAreTheirSlugs")

function Get-TestNames([string]$Exe, [string[]]$FilterArgs) {
    $Listing = @(& $Exe @FilterArgs -ltc --no-colors) | ForEach-Object { $_.TrimEnd("`r") }
    $Start = [Array]::IndexOf($Listing, '[doctest] listing all test case names')
    $Names = @()
    for ($I = $Start + 2; $I -ge 2 -and $I -lt $Listing.Count -and $Listing[$I] -notmatch '^={20,}$'; $I++) { $Names += $Listing[$I] }
    return $Names
}

function Test-PictureGuard([string[]]$Cases, [string[]]$RecordedTests) {
    $Cases = @($Cases | Where-Object { $_ })
    if ($Cases.Count -eq 0) {
        return [pscustomobject]@{ Passed = $false; Missing = @(); Message = "no E2E cases found: the case listing is empty or the filter is wrong" }
    }
    $Recorded = New-Object System.Collections.Generic.HashSet[string]
    foreach ($Name in @($RecordedTests)) { [void]$Recorded.Add($Name) }
    $Missing = @($Cases | Where-Object {
        $Name = $_
        $Exempt = @($PictureGuardExempt | Where-Object { $Name -ceq $_ -or $Name -ceq "Slow.$_" }).Count -gt 0
        -not $Exempt -and -not $Recorded.Contains($Name)
    })
    if ($Missing.Count -eq 0) { return [pscustomobject]@{ Passed = $true; Missing = @(); Message = "" } }
    $Message = "these E2E cases ran but wrote no picture (every E2E case must; tests/VisualSupport.h): " + ($Missing -join ", ")
    return [pscustomobject]@{ Passed = $false; Missing = $Missing; Message = $Message }
}

function New-VisualIndex($Entries, [hashtable]$Results) {
    $Sorted = New-Object System.Collections.Generic.List[object]
    foreach ($E in @($Entries)) { $Sorted.Add($E) }
    $Sorted.Sort([System.Comparison[object]] {
        param($A, $B)
        $ByTest = [string]::CompareOrdinal([string]$A.test, [string]$B.test)
        if ($ByTest -ne 0) { return $ByTest }
        return [string]::CompareOrdinal([string]$A.slug, [string]$B.slug)
    })
    $Messages = @()
    $Reported = New-Object System.Collections.Generic.HashSet[string]
    $Counts = [ordered]@{ passed = 0; failed = 0; deferred = 0 }
    $Indexed = @()
    foreach ($E in $Sorted) {
        $Test = [string]$E.test
        $Passed = $Results.ContainsKey($Test) -and [bool]$Results[$Test]
        $Deferred = $Test -like "Deferred.*"
        $Status = if ($Passed) { "passed" } elseif ($Deferred) { "deferred" } else { "failed" }
        if ($Deferred -and $Passed -and $Reported.Add($Test)) { $Messages += "$Test passes: move it out of Deferred" }
        $Counts[$Status]++
        $Item = [ordered]@{ test = $Test; slug = [string]$E.slug; status = $Status }
        if ($E.PSObject.Properties.Name -contains "text") { $Item.text = $E.text }
        $Indexed += [pscustomobject]$Item
    }
    return [pscustomobject]@{ Entries = $Indexed; Messages = $Messages; Counts = $Counts }
}

function Read-VisualEntries([string]$DataDir) {
    $Entries = @()
    foreach ($File in (Get-ChildItem -Path $DataDir -Filter *.json | Where-Object { $_.Name -ne "index.json" })) {
        $Text = [IO.File]::ReadAllText($File.FullName).Trim()
        if ($Text -notmatch '^\{') { continue }
        $Test = if ($Text -match '"test"\s*:\s*"((?:[^"\\]|\\.)*)"') { $Matches[1] } else { "" }
        $Entries += [pscustomobject]@{ test = $Test; slug = $File.BaseName; text = $Text }
    }
    return $Entries
}

# Runner: { param($Case, $DataDir) ... } returns the case's exit code; each case is its own process, so an abort loses no other picture.
function Invoke-VisualRun([string[]]$Cases, [string]$DataDir, [scriptblock]$Runner) {
    if (Test-Path $DataDir) { Get-ChildItem -Path $DataDir -Force | Remove-Item -Recurse -Force }
    New-Item -ItemType Directory -Force -Path $DataDir | Out-Null
    $Results = @{}
    foreach ($Case in @($Cases)) {
        $Code = @(& $Runner $Case $DataDir)
        $Results[$Case] = ($Code.Count -gt 0 -and [int]$Code[-1] -eq 0)
    }
    $Entries = Read-VisualEntries $DataDir
    $Index = New-VisualIndex -Entries $Entries -Results $Results
    $Guard = Test-PictureGuard -Cases $Cases -RecordedTests @($Entries | ForEach-Object { $_.test })
    $Output = @()
    $Output += $Index.Messages
    $Output += ("[Visuals.ps1] {0} entries: {1} passed, {2} failed, {3} deferred" -f @($Index.Entries).Count, $Index.Counts.passed, $Index.Counts.failed, $Index.Counts.deferred)
    if (-not $Guard.Passed) { $Output += "[Visuals.ps1] $($Guard.Message)" }
    $IndexJson = "[" + ((@($Index.Entries) | ForEach-Object { '"' + $_.slug + '"' }) -join ",") + "]"
    [IO.File]::WriteAllText((Join-Path $DataDir "index.json"), $IndexJson, $Utf8)
    return [pscustomobject]@{ Index = $Index.Entries; Counts = $Index.Counts; Guard = $Guard; Output = $Output; ExitCode = $(if ($Guard.Passed) { 0 } else { 1 }) }
}

function Find-Edge {
    foreach ($P in @("${env:ProgramFiles(x86)}\Microsoft\Edge\Application\msedge.exe", "$env:ProgramFiles\Microsoft\Edge\Application\msedge.exe")) {
        if ($P -and (Test-Path $P)) { return $P }
    }
    return $null
}

# One headless Edge per entry, $Parallel at a time; each needs its own profile or it hands off to a running Edge and returns at once.
function Invoke-Captures([string]$Edge, [string]$PageUrl, [string[]]$Slugs, [string]$ShotDir) {
    $Pending = New-Object System.Collections.Queue
    $Slugs | ForEach-Object { $Pending.Enqueue($_) }
    $Running = @()
    $Index = 0
    while ($Pending.Count -gt 0 -or $Running.Count -gt 0) {
        while ($Pending.Count -gt 0 -and $Running.Count -lt $Parallel) {
            $Key = $Pending.Dequeue()
            $Slug, $View = $Key -split '~'
            $Query = if ($View) { "capture&$View" } else { "capture" }
            $ProfileDir = Join-Path $env:TEMP ("lp-visuals-edge-{0}-{1}" -f $PID, $Index++)
            $Shot = "$ShotDir\$($Key.Replace('~', '.')).png"
            $EdgeArgs = @("--headless=new", "--user-data-dir=`"$ProfileDir`"", "--no-first-run", "--use-angle=swiftshader",
                          "--enable-unsafe-swiftshader", "--hide-scrollbars", "--window-size=1600,700",
                          "--virtual-time-budget=10000", "--screenshot=`"$Shot`"", "`"$PageUrl`?$Query#$Slug`"")
            $Proc = Start-Process -FilePath $Edge -ArgumentList $EdgeArgs -PassThru -NoNewWindow -RedirectStandardError "$ProfileDir.err"
            $Running += [pscustomobject]@{ Proc = $Proc; Profile = $ProfileDir; Started = Get-Date }
        }
        Start-Sleep -Milliseconds 200
        $Still = @()
        foreach ($R in $Running) {
            if (-not $R.Proc.HasExited -and ((Get-Date) - $R.Started).TotalSeconds -gt 60) { try { $R.Proc.Kill() } catch {} }
            if ($R.Proc.HasExited) {
                Remove-Item -Recurse -Force -ErrorAction SilentlyContinue $R.Profile, "$($R.Profile).err"
            } else { $Still += $R }
        }
        $Running = $Still
    }
}

# Cuts a 1600-wide capture into its before (left) and after (right) halves as JPEG data URIs.
function Split-Capture([string]$Png) {
    Add-Type -AssemblyName System.Drawing
    $Codec = [System.Drawing.Imaging.ImageCodecInfo]::GetImageEncoders() | Where-Object { $_.MimeType -eq "image/jpeg" }
    $Params = New-Object System.Drawing.Imaging.EncoderParameters(1)
    $Params.Param[0] = New-Object System.Drawing.Imaging.EncoderParameter([System.Drawing.Imaging.Encoder]::Quality, [long]82)
    $Image = [System.Drawing.Bitmap]::FromFile($Png)
    try {
        $Half = [int][Math]::Floor($Image.Width / 2)
        $Halves = @()
        foreach ($Rect in @((New-Object System.Drawing.Rectangle(0, 0, $Half, $Image.Height)),
                            (New-Object System.Drawing.Rectangle($Half, 0, ($Image.Width - $Half), $Image.Height)))) {
            $Part = $Image.Clone($Rect, [System.Drawing.Imaging.PixelFormat]::Format24bppRgb)
            $Stream = New-Object System.IO.MemoryStream
            $Part.Save($Stream, $Codec, $Params)
            $Halves += "data:image/jpeg;base64," + [Convert]::ToBase64String($Stream.ToArray())
            $Part.Dispose(); $Stream.Dispose()
        }
        return $Halves
    } finally { $Image.Dispose() }
}

function Get-ShotEntry($Run, [string]$Status) {
    $Names = @{}
    foreach ($B in $Run.bodies) { $Names[[int]$B.id] = if ($B.name) { $B.name } else { "body $($B.id)" } }
    $Joints = @{}
    foreach ($I in $Run.interfaces) { $Joints[[int]$I.id] = "$($Names[[int]$I.a]) $Dash $($Names[[int]$I.b])" }
    $After = $Run.after
    $Failures = @($After.failures | Where-Object { $_ })
    $Describe = { param($F) "$($Joints[[int]$F.interface]) $Dot $($F.axis), pass $($F.pass)" }
    $Source = "$($After.source)"
    $Label = if ($Source.StartsWith("expected") -or $After.expected_only -eq $true) { "After $Dot expected by the test" } else { "After $Dot engine" }
    return [ordered]@{
        slug = $Run.slug; title = $Run.title; test = $Run.test; status = $Status; summary = $Run.summary
        verdict = $Run.before.verdict; state = $After.state; passes = $After.passes
        clusters = @($After.clusters | Where-Object { $_ }).Count
        released = @($After.removed | Where-Object { $null -ne $_ } | ForEach-Object { $Names[[int]$_] })
        severed = @($Failures | Where-Object { $_.severed } | ForEach-Object { & $Describe $_ })
        cracked = @($Failures | Where-Object { -not $_.severed } | ForEach-Object { & $Describe $_ })
        elapsed_ms = $After.elapsed_ms; after_label = $Label; before_img = $null; after_img = $null
        expected_state = if ($Run.expected) { $Run.expected.state } else { $null }
        expected_released = @($Run.expected.clusters | Where-Object { $_ } | ForEach-Object { $_.bodies } | ForEach-Object { $Names[[int]$_] })
        expected_img = $null
    }
}

function Write-Page([string]$TemplateName, [string]$Id, [string]$Json, [string]$Target) {
    $Template = [IO.File]::ReadAllText("$Root\viewer\$TemplateName")
    $Placeholder = '<script type="application/json" id="' + $Id + '">null</script>'
    if (-not $Template.Contains($Placeholder)) { Write-Output "[Visuals.ps1] viewer/$TemplateName has no data placeholder"; exit 2 }
    $Page = $Template.Replace($Placeholder, '<script type="application/json" id="' + $Id + '">' + $Json.Replace('</', '<\/') + '</script>')
    [IO.File]::WriteAllText($Target, $Page, $Utf8)
}

if ($FunctionsOnly) { return }

& "$PSScriptRoot\Build.ps1" -Config $Config -Profile:$Profile
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
$Exe = "$BuildDir\tests\$Config\loadpath_tests.exe"
$Cases = @((Get-TestNames $Exe @("-tc=E2E.*")) + (Get-TestNames $Exe @("-sf=*EndToEndTest.cpp,*ScenarioFilesTest.cpp")) | Sort-Object -Unique)
$CaseRunner = {
    param([string]$Case, [string]$Dir)
    $ErrorActionPreference = "Continue"
    $env:LOADPATH_VISUALS = $Dir
    try { & $Exe "-tc=$Case" --no-colors *> $null; return $LASTEXITCODE } finally { Remove-Item Env:\LOADPATH_VISUALS -ErrorAction SilentlyContinue }
}
$Run = Invoke-VisualRun -Cases $Cases -DataDir "$Out\data" -Runner $CaseRunner
$Entries = @($Run.Index)

$Edge = if ($NoScreenshots) { $null } else { Find-Edge }
if (-not $NoScreenshots -and -not $Edge) { Write-Output "[Visuals.ps1] Microsoft Edge not found; writing the 3D viewer without screenshots" }
$Shots = [bool]$Edge -and $Entries.Count -gt 0

$ErrorActionPreference = "Continue"
$Commit = (& git -C $Root rev-parse --short HEAD 2>$null)
$Dirty = if ((& git -C $Root status --porcelain 2>$null | Measure-Object).Count -gt 0) { "+uncommitted" } else { "" }
$ErrorActionPreference = "Stop"
$When = (Get-Date).ToString("yyyy-MM-dd HH:mm")
$Generated = '{"when":"' + $When + '","commit":"' + $Commit + $Dirty + '","passed":' + $Run.Counts.passed + ',"failed":' + $Run.Counts.failed +
             ',"deferred":' + $Run.Counts.deferred + ',"screenshots":' + $Shots.ToString().ToLower() + '}'
$Data = '{"generated":' + $Generated + ',"runs":[' + (($Entries | ForEach-Object { '{"status":"' + $_.status + '",' + $_.text.Substring(1) }) -join ',') + ']}'

New-Item -ItemType Directory -Force -Path $Out | Out-Null
Write-Page "index.html" "lp-data" $Data "$Out\index.html"

if ($Shots) {
    $ShotDir = "$Out\shots"
    if (Test-Path $ShotDir) { Remove-Item -Recurse -Force $ShotDir }
    New-Item -ItemType Directory -Force -Path $ShotDir | Out-Null
    $Clock = [Diagnostics.Stopwatch]::StartNew()
    $Keys = @($Entries | ForEach-Object { $_.slug }) + @($Entries | Where-Object { $_.text -match '"expected"\s*:' } | ForEach-Object { "$($_.slug)~expected" })
    Invoke-Captures $Edge ([Uri]"$Out\index.html").AbsoluteUri $Keys $ShotDir
    $ShotEntries = @()
    $NoShot = @()
    foreach ($E in $Entries) {
        $Item = Get-ShotEntry ($E.text | ConvertFrom-Json) $E.status
        $Png = "$ShotDir\$($E.slug).png"
        if (Test-Path $Png) { $Halves = Split-Capture $Png; $Item.before_img = $Halves[0]; $Item.after_img = $Halves[1] } else { $NoShot += $E.slug }
        $ExpectedPng = "$ShotDir\$($E.slug).expected.png"
        if (Test-Path $ExpectedPng) { $Item.expected_img = (Split-Capture $ExpectedPng)[1] }
        $ShotEntries += New-Object psobject -Property $Item
    }
    $ShotJson = '{"generated":' + $Generated + ',"viewer":"index.html","entries":' + (ConvertTo-Json -InputObject @($ShotEntries) -Depth 4 -Compress) + '}'
    Write-Page "screenshots.html" "lp-shots" $ShotJson "$Out\screenshots.html"
    Write-Output ("[Visuals.ps1] {0} screenshots in {1:N0} s -> {2}\screenshots.html" -f ($Entries.Count - $NoShot.Count), $Clock.Elapsed.TotalSeconds, $Out)
    if ($NoShot.Count -gt 0) { Write-Output "[Visuals.ps1] no screenshot captured for: $($NoShot -join ', ')" }
}

$Run.Output | Write-Output
Write-Output "[Visuals.ps1] $($Cases.Count) E2E cases run; page -> $Out\index.html"
exit $Run.ExitCode
