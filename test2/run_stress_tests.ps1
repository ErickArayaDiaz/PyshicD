$ErrorActionPreference = "Stop"

$root = Split-Path -Parent $PSScriptRoot
$compiler = "C:\ProgramData\mingw64\mingw64\bin\g++.exe"
$executable = Join-Path $PSScriptRoot "PhysicsStressTests.exe"
$report = Join-Path $PSScriptRoot "PhysicsStressResults.txt"

if (-not (Test-Path $compiler)) {
    throw "C++ compiler not found: $compiler"
}

& $compiler `
    -std=c++17 `
    -Wall `
    -Wextra `
    -Wpedantic `
    "-I$PSScriptRoot" `
    "-I$root" `
    (Join-Path $PSScriptRoot "PhysicsStressTests.cpp") `
    (Join-Path $root "Integrator.cpp") `
    (Join-Path $root "RigidBody.cpp") `
    (Join-Path $root "World.cpp") `
    (Join-Path $root "CollisionDetector.cpp") `
    (Join-Path $root "CollisionResolver.cpp") `
    -o $executable

if ($LASTEXITCODE -ne 0) {
    throw "C++ compilation failed with exit code $LASTEXITCODE"
}

& $executable $report
if ($LASTEXITCODE -ne 0) {
    throw "Physics stress tests failed with exit code $LASTEXITCODE; see $report"
}

Write-Output "Full step-by-step summary: $report"
