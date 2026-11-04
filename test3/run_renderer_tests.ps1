$ErrorActionPreference = "Stop"

$root = Split-Path -Parent $PSScriptRoot
$compiler = "C:\ProgramData\mingw64\mingw64\bin\g++.exe"
$renderer = Join-Path $root "Renderer.cpp"
$includePaths = @(
    "-I$(Join-Path $root 'test2')",
    "-I$root"
)

if ($env:SFML_INCLUDE_DIR) {
    if (-not (Test-Path (Join-Path $env:SFML_INCLUDE_DIR "SFML\Graphics.hpp"))) {
        throw "SFML_INCLUDE_DIR does not contain SFML\Graphics.hpp: $env:SFML_INCLUDE_DIR"
    }
    $includePaths += "-I$env:SFML_INCLUDE_DIR"
}

if (-not (Test-Path $compiler)) {
    throw "C++ compiler not found: $compiler"
}

if (-not (Test-Path $renderer)) {
    throw "Renderer source not found: $renderer"
}

$compileArgs = @(
    "-std=c++17",
    "-Wall",
    "-Wextra",
    "-Wpedantic"
) + $includePaths

$previousErrorActionPreference = $ErrorActionPreference
$ErrorActionPreference = "Continue"
try {
    $baselineOutput = & $compiler @compileArgs -fsyntax-only $renderer 2>&1
    $baselineExitCode = $LASTEXITCODE
}
finally {
    $ErrorActionPreference = $previousErrorActionPreference
}
if ($baselineExitCode -ne 0) {
    $baselineOutput | Write-Output
    if (($baselineOutput | Out-String) -match 'SFML/Graphics\.hpp') {
        throw "SFML headers are unavailable. Install SFML or set SFML_INCLUDE_DIR to its include directory."
    }
    throw "Renderer compilation failed with exit code $baselineExitCode"
}

$rendererText = [System.IO.File]::ReadAllText($renderer)
$lambdaPattern = '(?m)^(\s*m_world\.forEachBody\(\[this\]\(const RigidBody& b\)\r?\n\s*\{)'
$matches = [regex]::Matches($rendererText, $lambdaPattern)
if ($matches.Count -ne 1) {
    throw "Expected exactly one const RigidBody renderer lambda; found $($matches.Count)"
}

$mutatedText = [regex]::Replace(
    $rendererText,
    $lambdaPattern,
    [System.Text.RegularExpressions.MatchEvaluator]{
        param($match)
        $match.Value + [Environment]::NewLine + "        b.position.x = 0;"
    },
    1
)

$temporaryDirectory = Join-Path $env:TEMP ("PyshicD-ttest3-" + [guid]::NewGuid().ToString("N"))
$mutatedRenderer = Join-Path $temporaryDirectory "Renderer.cpp"

try {
    New-Item -ItemType Directory -Path $temporaryDirectory | Out-Null
    [System.IO.File]::WriteAllText($mutatedRenderer, $mutatedText)

    $previousErrorActionPreference = $ErrorActionPreference
    $ErrorActionPreference = "Continue"
    try {
        $mutationOutput = & $compiler @compileArgs -fsyntax-only $mutatedRenderer 2>&1
        $mutationExitCode = $LASTEXITCODE
    }
    finally {
        $ErrorActionPreference = $previousErrorActionPreference
    }
    if ($mutationExitCode -eq 0) {
        throw "The mutation compiled successfully; const is not being propagated to the renderer lambda."
    }

    $mutationDiagnostics = $mutationOutput | Out-String
    if ($mutationDiagnostics -notmatch 'b\.position\.x\s*=\s*0' -or
        $mutationDiagnostics -notmatch '(?i)read-only|const') {
        $mutationOutput | Write-Output
        throw "The mutated renderer failed for a reason other than the expected const-assignment error."
    }
}
finally {
    if (Test-Path -LiteralPath $mutatedRenderer) {
        Remove-Item -LiteralPath $mutatedRenderer -Force
    }
    if (Test-Path -LiteralPath $temporaryDirectory) {
        Remove-Item -LiteralPath $temporaryDirectory -Force
    }
}

Write-Output "Renderer compiles, and the temporary mutation fails because the body is const."
$global:LASTEXITCODE = 0
