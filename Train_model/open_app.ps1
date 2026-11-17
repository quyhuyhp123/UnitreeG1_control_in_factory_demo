$ErrorActionPreference='Stop'
$python='D:\unitree_g1\.venv\yolo\Scripts\python.exe'
if (-not (Test-Path -LiteralPath $python)) { throw 'Missing existing isolated YOLO environment. No packages installed automatically.' }
Set-Location 'D:\unitree_g1\Train_model'
& $python 'D:\unitree_g1\Train_model\app.py'
if ($LASTEXITCODE -ne 0) { throw 'Dataset GUI failed' }
