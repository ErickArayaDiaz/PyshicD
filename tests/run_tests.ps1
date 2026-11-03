$ErrorActionPreference = "Stop"

$root = Split-Path -Parent $PSScriptRoot
$compiler = "C:\ProgramData\mingw64\mingw64\bin\g++.exe"
$executable = Join-Path $PSScriptRoot "PhysicsTests.exe"
$report = Join-Path $PSScriptRoot "PhysicsTestResults.txt"

if (-not (Test-Path $compiler)) {
    throw "C++ compiler not found: $compiler"
}

& $compiler `
    -std=c++17 `
    -Wall `
    -Wextra `
    -Wpedantic `
    "-I$(Join-Path $root 'tests\support')" `
    "-I$root" `
    (Join-Path $PSScriptRoot "PhysicsTests.cpp") `
    (Join-Path $root "Integrator.cpp") `
    (Join-Path $root "RigidBody.cpp") `
    (Join-Path $root "World.cpp") `
    -o $executable

if ($LASTEXITCODE -ne 0) {
    throw "C++ compilation failed with exit code $LASTEXITCODE"
}

& $executable $report
if ($LASTEXITCODE -ne 0) {
    throw "Physics tests failed with exit code $LASTEXITCODE"
}

Write-Output "Full step-by-step report: $report"
