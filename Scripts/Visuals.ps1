# Build the end-to-end picture pages: every test that records a picture (tests/VisualSupport.h) writes
# one JSON file per entry into the folder named by LOADPATH_VISUALS; this script embeds them all in
# viewer/index.html (the animated 3D viewer) and, unless -NoScreenshots, captures each entry's before and
# after frames with headless Edge into viewer/screenshots.html. Both pages are self-contained files.
#   Scripts/Visuals.ps1                       build Release, run E2E.* and Deferred.* with recording on, write both pages
#   Scripts/Visuals.ps1 -NoScreenshots        the 3D viewer only (no Edge, a few seconds faster per entry)
#   Scripts/Visuals.ps1 -Tests "Cascade.*"    record a different selection
#   Scripts/Visuals.ps1 -Assemble -DataDir d -FailedTests a,b    pages only, from a run that already recorded into d
# Each entry is marked passed, failed or deferred (Deferred.* tests are expected red).
# Exit code: 1 when an executed E2E.* test recorded no picture (named), else 0; test failures do not change it.
param([ValidateSet("Debug","Release")][string]$Config = "Release",
      [string]$Tests = "E2E.*,Deferred.*",
      [string]$Out = "",
      [switch]$Assemble,
      [string]$DataDir = "",
      [string[]]$FailedTests = @(),
      [string[]]$RanTests = @(),
      [switch]$NoScreenshots,
      [int]$Parallel = 4,
      [switch]$Profile)
$ErrorActionPreference = "Stop"
$Root = Split-Path -Parent $PSScriptRoot
$BuildDir = if ($Profile) { "$Root\build\msvc-profile" } else { "$Root\build\msvc" }
if ($Out -eq "") { $Out = "$Root\build\visuals" }
$Utf8 = New-Object System.Text.UTF8Encoding($false)
$Dash = [string][char]0x2013; $Dot = [string][char]0xB7

# powershell -File passes a list as one comma-joined string.
function Split-Names([string[]]$Names) { return @($Names | ForEach-Object { $_ -split ',' } | ForEach-Object { $_.Trim() } | Where-Object { $_ }) }

function Get-FailedTestNames([string[]]$Lines) {
    $Failed = New-Object System.Collections.Generic.HashSet[string]
    $Current = $null
    foreach ($Line in $Lines) {
        if ($Line -match '^TEST CASE:\s+(.+?)\s*$') { $Current = $Matches[1] }
        elseif ($Line -match '^={20,}$') { $Current = $null }
        elseif ($null -ne $Current -and $Line -match '(ERROR|FATAL ERROR): ') { [void]$Failed.Add($Current) }
    }
    return @($Failed)
}

function Get-TestNames([string]$Exe, [string]$Filter) {
    $Listing = @(& $Exe "-tc=$Filter" -ltc --no-colors) | ForEach-Object { $_.TrimEnd("`r") }
    $Start = [Array]::IndexOf($Listing, '[doctest] listing all test case names')
    $Names = @()
    for ($I = $Start + 2; $I -ge 2 -and $I -lt $Listing.Count -and $Listing[$I] -notmatch '^={20,}$'; $I++) { $Names += $Listing[$I] }
    return $Names
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
            $Slug = $Pending.Dequeue()
            $ProfileDir = Join-Path $env:TEMP ("lp-visuals-edge-{0}-{1}" -f $PID, $Index++)
            $Shot = "$ShotDir\$Slug.png"
            $EdgeArgs = @("--headless=new", "--user-data-dir=`"$ProfileDir`"", "--no-first-run", "--use-angle=swiftshader",
                          "--enable-unsafe-swiftshader", "--hide-scrollbars", "--window-size=1600,700",
                          "--virtual-time-budget=10000", "--screenshot=`"$Shot`"", "`"$PageUrl`?capture#$Slug`"")
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
    }
}

$FailedTests = Split-Names $FailedTests
$RanTests = Split-Names $RanTests

if (-not $Assemble) {
    & "$PSScriptRoot\Build.ps1" -Config $Config -Profile:$Profile
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
    $Exe = "$BuildDir\tests\$Config\loadpath_tests.exe"
    $DataDir = "$Out\recordings"
    if (Test-Path $DataDir) { Remove-Item -Recurse -Force $DataDir }
    New-Item -ItemType Directory -Force -Path $DataDir | Out-Null
    $env:LOADPATH_VISUALS = $DataDir
    $Output = @(& $Exe "-tc=$Tests" --no-colors 2>&1 | ForEach-Object { "$_" })
    Remove-Item Env:\LOADPATH_VISUALS
    $Output | Where-Object { $_ -match '^\[doctest\] (test cases|assertions|Status)' } | Write-Output
    $FailedTests = Get-FailedTestNames $Output
    $RanTests = Get-TestNames $Exe $Tests
}

if ($DataDir -eq "" -or -not (Test-Path $DataDir)) {
    Write-Output "[Visuals.ps1] no recordings folder ($DataDir)"
    exit 2
}

$Failed = New-Object System.Collections.Generic.HashSet[string]
foreach ($Name in $FailedTests) { [void]$Failed.Add($Name) }
$Entries = @()
$Recorded = New-Object System.Collections.Generic.HashSet[string]
$Counts = @{ passed = 0; failed = 0; deferred = 0 }
foreach ($File in (Get-ChildItem -Path $DataDir -Filter *.json | Sort-Object Name)) {
    $Text = [IO.File]::ReadAllText($File.FullName).Trim()
    if ($Text -notmatch '^\{') { Write-Output "[Visuals.ps1] skipped $($File.Name): not a JSON object"; continue }
    $Test = if ($Text -match '"test"\s*:\s*"((?:[^"\\]|\\.)*)"') { $Matches[1] } else { "" }
    [void]$Recorded.Add($Test)
    $Status = if ($Test -like "Deferred.*") { "deferred" } elseif ($Failed.Contains($Test)) { "failed" } else { "passed" }
    $Counts[$Status]++
    $Entries += [pscustomobject]@{ Test = $Test; Slug = $File.BaseName; Status = $Status; Text = $Text
                                   Json = '{"status":"' + $Status + '",' + $Text.Substring(1) }
}
$Entries = @($Entries | Sort-Object Test, Slug)

$Missing = @($RanTests | Where-Object { $_ -like "E2E.*" -and -not $Recorded.Contains($_) })

$Edge = if ($NoScreenshots) { $null } else { Find-Edge }
if (-not $NoScreenshots -and -not $Edge) { Write-Output "[Visuals.ps1] Microsoft Edge not found; writing the 3D viewer without screenshots" }
$Shots = [bool]$Edge -and $Entries.Count -gt 0

$Commit = (& git -C $Root rev-parse --short HEAD 2>$null)
$Dirty = if ((& git -C $Root status --porcelain 2>$null | Measure-Object).Count -gt 0) { "+uncommitted" } else { "" }
$When = (Get-Date).ToString("yyyy-MM-dd HH:mm")
$Generated = '{"when":"' + $When + '","commit":"' + $Commit + $Dirty + '","passed":' + $Counts.passed + ',"failed":' + $Counts.failed +
             ',"deferred":' + $Counts.deferred + ',"screenshots":' + $Shots.ToString().ToLower() + '}'
$Data = '{"generated":' + $Generated + ',"runs":[' + (($Entries | ForEach-Object { $_.Json }) -join ',') + ']}'

function Write-Page([string]$TemplateName, [string]$Id, [string]$Json, [string]$Target) {
    $Template = [IO.File]::ReadAllText("$Root\viewer\$TemplateName")
    $Placeholder = '<script type="application/json" id="' + $Id + '">null</script>'
    if (-not $Template.Contains($Placeholder)) { Write-Output "[Visuals.ps1] viewer/$TemplateName has no data placeholder"; exit 2 }
    $Page = $Template.Replace($Placeholder, '<script type="application/json" id="' + $Id + '">' + $Json.Replace('</', '<\/') + '</script>')
    [IO.File]::WriteAllText($Target, $Page, $Utf8)
}

New-Item -ItemType Directory -Force -Path $Out | Out-Null
Write-Page "index.html" "lp-data" $Data "$Out\index.html"

if ($Shots) {
    $ShotDir = "$Out\shots"
    if (Test-Path $ShotDir) { Remove-Item -Recurse -Force $ShotDir }
    New-Item -ItemType Directory -Force -Path $ShotDir | Out-Null
    $Clock = [Diagnostics.Stopwatch]::StartNew()
    Invoke-Captures $Edge ([Uri]"$Out\index.html").AbsoluteUri @($Entries | ForEach-Object { $_.Slug }) $ShotDir
    $ShotEntries = @()
    $NoShot = @()
    foreach ($E in $Entries) {
        $Item = Get-ShotEntry ($E.Text | ConvertFrom-Json) $E.Status
        $Png = "$ShotDir\$($E.Slug).png"
        if (Test-Path $Png) { $Halves = Split-Capture $Png; $Item.before_img = $Halves[0]; $Item.after_img = $Halves[1] } else { $NoShot += $E.Slug }
        $ShotEntries += New-Object psobject -Property $Item
    }
    $ShotJson = '{"generated":' + $Generated + ',"viewer":"index.html","entries":' + (ConvertTo-Json -InputObject @($ShotEntries) -Depth 4 -Compress) + '}'
    Write-Page "screenshots.html" "lp-shots" $ShotJson "$Out\screenshots.html"
    Write-Output ("[Visuals.ps1] {0} screenshots in {1:N0} s -> {2}\screenshots.html" -f ($Entries.Count - $NoShot.Count), $Clock.Elapsed.TotalSeconds, $Out)
    if ($NoShot.Count -gt 0) { Write-Output "[Visuals.ps1] no screenshot captured for: $($NoShot -join ', ')" }
}

Write-Output ("[Visuals.ps1] {0} entries ({1} passed, {2} failed, {3} deferred) -> {4}\index.html" -f $Entries.Count, $Counts.passed, $Counts.failed, $Counts.deferred, $Out)
if ($Missing.Count -gt 0) {
    Write-Output "[Visuals.ps1] these E2E tests ran but recorded no picture (every E2E test must; tests/VisualSupport.h):"
    $Missing | ForEach-Object { Write-Output "  $_" }
    exit 1
}
exit 0
