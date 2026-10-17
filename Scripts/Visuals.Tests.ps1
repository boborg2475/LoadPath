# Plain PowerShell tests for Scripts/Visuals.ps1 (no Pester): W10 to W13 of claude_plans/test_plans/2026-10-16-scenario-workbench.md.
# Dot-sources Visuals.ps1 with -FunctionsOnly (loads its functions, runs nothing) and feeds them fixture inputs. Exit 1 when any case fails.
$ErrorActionPreference = "Stop"
$Script:Failures = @()
$Script:Passed = 0

function Assert-True([string]$Case, [bool]$Condition, [string]$Detail) {
    if ($Condition) { return }
    $Script:Failures += "$Case`: $Detail"
    throw "assertion failed: $Detail"
}

function Invoke-Case([string]$Name, [scriptblock]$Body) {
    $Before = $Script:Failures.Count
    try { & $Body } catch {
        if ($Script:Failures.Count -eq $Before) { $Script:Failures += "$Name`: $($_.Exception.Message)" }
    }
    if ($Script:Failures.Count -eq $Before) { $Script:Passed++; Write-Output "PASS $Name" } else { Write-Output "FAIL $Name"; Write-Output "  $($Script:Failures[-1])" }
}

function New-Entry([string]$Test, [string]$Slug) { return [pscustomobject]@{ test = $Test; slug = $Slug } }

function Write-EntryFile([string]$Dir, [string]$Test, [string]$Slug) {
    $Json = '{"slug":"' + $Slug + '","test":"' + $Test + '","after":{"elapsed_ms":1.5}}'
    [IO.File]::WriteAllText((Join-Path $Dir "$Slug.json"), $Json, (New-Object System.Text.UTF8Encoding($false)))
}

$Loaded = $true
$Ast = [System.Management.Automation.Language.Parser]::ParseFile("$PSScriptRoot\Visuals.ps1", [ref]$null, [ref]$null)
$Declared = @($Ast.ParamBlock.Parameters | ForEach-Object { $_.Name.VariablePath.UserPath })
if ($Declared -notcontains "FunctionsOnly") {
    $Loaded = $false
    $LoadError = "Visuals.ps1 declares no -FunctionsOnly switch, so dot-sourcing it would run the whole script"
} else {
    try { . "$PSScriptRoot\Visuals.ps1" -FunctionsOnly } catch { $Loaded = $false; $LoadError = $_.Exception.Message }
}
if (-not $Loaded) {
    foreach ($Case in @("Visuals.Script.TheGuardNamesEveryE2ECaseThatWroteNoEntry", "Visuals.Script.TheIndexIsOrderedByTestNameAndMarksStatus",
                        "Visuals.Script.ARedCaseKeepsItsPictureOnThePage", "Visuals.Script.TheDataFolderIsEmptiedBeforeTheRun")) {
        Write-Output "FAIL $Case"
        Write-Output "  cannot dot-source Visuals.ps1 -FunctionsOnly: $LoadError"
    }
    Write-Output "[Visuals.Tests.ps1] 0 passed, 4 failed"
    exit 1
}
$ErrorActionPreference = "Stop"

$G2 = "E2E.Scenarios.EverySlowInDebugScenarioFileMeetsItsExpectation"
$G3 = "Slow.Scenarios.EverySlowScenarioFileMeetsItsExpectation"
$G4 = "Deferred.Scenarios.EveryDeferredScenarioFileMeetsItsExpectation"
$G5 = "E2E.Scenarios.FileNamesAreTheirSlugs"

Invoke-Case "Visuals.Script.TheGuardNamesEveryE2ECaseThatWroteNoEntry" {
    $C = "Visuals.Script.TheGuardNamesEveryE2ECaseThatWroteNoEntry"
    $Cases = @("E2E.A", "E2E.B", "Deferred.Slice6.C", $G4, $G2, "Slow.$G2", $G3, $G5)
    $Missing = Test-PictureGuard -Cases $Cases -RecordedTests @("E2E.A", "Deferred.Slice6.C")
    Assert-True $C (-not $Missing.Passed) "the guard passed with E2E.B unrecorded"
    Assert-True $C (@($Missing.Missing).Count -eq 1 -and @($Missing.Missing)[0] -eq "E2E.B") "missing should be exactly E2E.B, got: $(@($Missing.Missing) -join ', ')"
    Assert-True $C ($Missing.Message -like "*E2E.B*") "the message must name E2E.B: $($Missing.Message)"
    foreach ($Other in @("E2E.A", "Deferred.Slice6.C", $G4, $G2, $G3, $G5)) {
        Assert-True $C ($Missing.Message -notlike "*$Other*") "the message must not name $Other`: $($Missing.Message)"
    }
    $All = Test-PictureGuard -Cases $Cases -RecordedTests @("E2E.A", "E2E.B", "Deferred.Slice6.C")
    Assert-True $C $All.Passed "the guard failed with every case recorded: $($All.Message)"
    $Empty = Test-PictureGuard -Cases @() -RecordedTests @("E2E.A")
    Assert-True $C (-not $Empty.Passed) "an empty case list passed"
    Assert-True $C ($Empty.Message -like "*no E2E cases found*") "the empty-list message must say 'no E2E cases found': $($Empty.Message)"
    $Dir = Join-Path ([IO.Path]::GetTempPath()) ("lp-visuals-test-" + [Guid]::NewGuid().ToString("N"))
    New-Item -ItemType Directory -Force -Path $Dir | Out-Null
    try {
        $Runner = { param([string]$Case, [string]$DataDir) if ($Case -eq "E2E.A") { Write-EntryFile $DataDir "E2E.A" "a" }; return 0 }
        $Run = Invoke-VisualRun -Cases @("E2E.A", "E2E.B") -DataDir $Dir -Runner $Runner
        Assert-True $C ($Run.ExitCode -eq 1) "a run with E2E.B unrecorded must exit 1, got $($Run.ExitCode)"
        $IndexPath = Join-Path $Dir "index.json"
        Assert-True $C (Test-Path $IndexPath) "index.json must be written even when the guard fails"
        Assert-True $C ([IO.File]::ReadAllText($IndexPath) -like '*"a"*') "index.json must list a, got $([IO.File]::ReadAllText($IndexPath))"
        $Output = @($Run.Output) -join "`n"
        Assert-True $C ($Output -like "*E2E.B*") "the run output must name E2E.B: $Output"
    } finally { Remove-Item -Recurse -Force -ErrorAction SilentlyContinue $Dir }
}

Invoke-Case "Visuals.Script.TheIndexIsOrderedByTestNameAndMarksStatus" {
    $C = "Visuals.Script.TheIndexIsOrderedByTestNameAndMarksStatus"
    $Entries = @((New-Entry "E2E.B" "b1"), (New-Entry "E2E.A" "a2"), (New-Entry "E2E.A" "a1"), (New-Entry "Deferred.Slice6.C" "c1"), (New-Entry "Deferred.Slice6.D" "d1"))
    $Results = @{ "E2E.A" = $true; "E2E.B" = $false; "Deferred.Slice6.C" = $false; "Deferred.Slice6.D" = $true }
    $Index = New-VisualIndex -Entries $Entries -Results $Results
    $Slugs = @($Index.Entries | ForEach-Object { $_.slug }) -join ","
    Assert-True $C ($Slugs -ceq "c1,d1,a1,a2,b1") "order should be c1,d1,a1,a2,b1, got $Slugs"
    $Statuses = @($Index.Entries | ForEach-Object { $_.status }) -join ","
    Assert-True $C ($Statuses -ceq "deferred,passed,passed,passed,failed") "statuses should be deferred,passed,passed,passed,failed, got $Statuses"
    $Messages = @($Index.Messages) -join "`n"
    Assert-True $C ($Messages -like "*Deferred.Slice6.D passes: move it out of Deferred*") "missing the move-out message, got: $Messages"
    Assert-True $C ($Messages -notlike "*Deferred.Slice6.C passes*") "a red deferred case must not be reported as passing: $Messages"
}

Invoke-Case "Visuals.Script.ARedCaseKeepsItsPictureOnThePage" {
    $C = "Visuals.Script.ARedCaseKeepsItsPictureOnThePage"
    $Dir = Join-Path ([IO.Path]::GetTempPath()) ("lp-visuals-test-" + [Guid]::NewGuid().ToString("N"))
    New-Item -ItemType Directory -Force -Path $Dir | Out-Null
    try {
        $Runner = {
            param([string]$Case, [string]$DataDir)
            switch ($Case) {
                "E2E.A" { Write-EntryFile $DataDir "E2E.A" "a"; return 0 }
                "E2E.B" { Write-EntryFile $DataDir "E2E.B" "b"; return 1 }
                "E2E.C" { Write-EntryFile $DataDir "E2E.C" "c"; return 3 }
            }
        }
        $Run = Invoke-VisualRun -Cases @("E2E.A", "E2E.B", "E2E.C") -DataDir $Dir -Runner $Runner
        $Status = @{}
        foreach ($E in @($Run.Index)) { $Status[$E.slug] = $E.status }
        Assert-True $C (@($Run.Index).Count -eq 3) "all three entries must be in the index, got $(@($Run.Index).Count)"
        Assert-True $C ($Status["a"] -eq "passed") "a should be passed, got $($Status['a'])"
        Assert-True $C ($Status["b"] -eq "failed") "b should be failed, got $($Status['b'])"
        Assert-True $C ($Status["c"] -eq "failed") "c (aborted) should be failed, got $($Status['c'])"
        Assert-True $C ($Run.ExitCode -eq 0) "the run should exit 0 when the guard passes, got $($Run.ExitCode)"
        $Output = @($Run.Output) -join "`n"
        Assert-True $C ($Output -like "*1 passed, 2 failed, 0 deferred*") "the output must count 1 passed, 2 failed, 0 deferred: $Output"
    } finally { Remove-Item -Recurse -Force -ErrorAction SilentlyContinue $Dir }
}

Invoke-Case "Visuals.Script.TheDataFolderIsEmptiedBeforeTheRun" {
    $C = "Visuals.Script.TheDataFolderIsEmptiedBeforeTheRun"
    $Dir = Join-Path ([IO.Path]::GetTempPath()) ("lp-visuals-test-" + [Guid]::NewGuid().ToString("N"))
    New-Item -ItemType Directory -Force -Path $Dir | Out-Null
    try {
        Write-EntryFile $Dir "E2E.Gone" "old"
        $Runner = { param([string]$Case, [string]$DataDir) Write-EntryFile $DataDir "E2E.A" "a"; return 0 }
        $Run = Invoke-VisualRun -Cases @("E2E.A") -DataDir $Dir -Runner $Runner
        Assert-True $C (-not (Test-Path (Join-Path $Dir "old.json"))) "old.json must be deleted before the run"
        $Slugs = @($Run.Index | ForEach-Object { $_.slug })
        Assert-True $C ($Slugs -notcontains "old") "old must not be in the index"
        Assert-True $C ($Slugs -contains "a") "a must be in the index"
    } finally { Remove-Item -Recurse -Force -ErrorAction SilentlyContinue $Dir }
}

Write-Output ("[Visuals.Tests.ps1] {0} passed, {1} failed" -f $Script:Passed, $Script:Failures.Count)
if ($Script:Failures.Count -gt 0) { exit 1 }
exit 0
