# Configure (first run) and build LoadPath with the Visual Studio 2022 generator. Exit code is the build's.
#   -Config Debug|Release (default Debug)   -Clean removes the build folder first
#   -Profile builds the phase-timing preset (msvc-profile, LOADPATH_PROFILE=ON) into build/msvc-profile
param([ValidateSet("Debug","Release")][string]$Config = "Debug", [switch]$Clean, [switch]$Profile)
$ErrorActionPreference = "Stop"
$Root = Split-Path -Parent $PSScriptRoot
$CMake = "C:\Program Files\Microsoft Visual Studio\2022\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe"
$Preset = if ($Profile) { "msvc-profile" } else { "msvc" }
$BuildPreset = if ($Profile) { "profile-" + $Config.ToLower() } else { $Config.ToLower() }
$BuildDir = "$Root\build\$Preset"
if ($Clean -and (Test-Path $BuildDir)) { Remove-Item -Recurse -Force $BuildDir }
Push-Location $Root
try {
	if (-not (Test-Path "$BuildDir\CMakeCache.txt")) {
		& $CMake --preset $Preset
		if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
	}
	& $CMake --build --preset $BuildPreset
	exit $LASTEXITCODE
}
finally { Pop-Location }
