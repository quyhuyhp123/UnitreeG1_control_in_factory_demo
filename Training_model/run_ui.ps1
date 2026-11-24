param([string]$Python)
$ErrorActionPreference='Stop'
$trainingRoot=$PSScriptRoot
if (-not $Python) { $Python=$env:G1_TRAINING_PYTHON }
if (-not $Python) {
    $gpuPython=Join-Path $trainingRoot '.venv\Scripts\python.exe'
    if (Test-Path -LiteralPath $gpuPython) { $Python=$gpuPython }
}
if (-not $Python) { $Python=Join-Path (Split-Path $trainingRoot -Parent) '.venv\yolo\Scripts\python.exe' }
if (-not (Test-Path -LiteralPath $Python)) { throw 'Missing isolated Python. Set G1_TRAINING_PYTHON. No packages installed automatically.' }
$env:G1_TRAINING_PYTHON=$Python
$env:PYTHONUTF8='1'
Set-Location -LiteralPath $trainingRoot
& $Python (Join-Path $trainingRoot 'training_ui.py')
if ($LASTEXITCODE -ne 0) { throw 'Training UI failed; training is never started automatically.' }
