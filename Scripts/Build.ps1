# Configure (first run) and build LoadPath with the Visual Studio 2022 generator. Exit code is the build's.
#   -Config Debug|Release (default Debug)   -Clean removes build/msvc first
param([ValidateSet("Debug","Release")][string]$Config = "Debug", [switch]$Clean)
$ErrorActionPreference = "Stop"
$Root = Split-Path -Parent $PSScriptRoot
$CMake = "C:\Program Files\Microsoft Visual Studio\2022\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe"
if ($Clean -and (Test-Path "$Root\build\msvc")) { Remove-Item -Recurse -Force "$Root\build\msvc" }
Push-Location $Root
try {
	if (-not (Test-Path "$Root\build\msvc\CMakeCache.txt")) {
		& $CMake --preset msvc
		if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
	}
	& $CMake --build --preset $Config.ToLower()
	exit $LASTEXITCODE
}
finally { Pop-Location }
