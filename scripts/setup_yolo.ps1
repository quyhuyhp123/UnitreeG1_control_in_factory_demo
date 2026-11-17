param([string]$BasePython)
$ErrorActionPreference='Stop'
$projectRoot='D:\unitree_g1'
$env:YOLO_CONFIG_DIR=Join-Path $projectRoot 'logs\yolo_settings'
New-Item -ItemType Directory -Path $env:YOLO_CONFIG_DIR -Force | Out-Null
$inferencePython=Join-Path $projectRoot '.venv\yolo\Scripts\python.exe'
if (-not $BasePython) { $BasePython=(Get-Command python -ErrorAction Stop).Source }
if (-not (Test-Path -LiteralPath $inferencePython)) {
    & uv venv --python $BasePython (Join-Path $projectRoot '.venv\yolo')
    if ($LASTEXITCODE -ne 0) { throw 'YOLO venv creation failed' }
}
& uv pip install --python $inferencePython 'torch==2.7.1' 'torchvision==0.22.1' --index-url https://download.pytorch.org/whl/cpu
if ($LASTEXITCODE -ne 0) { throw 'CPU PyTorch install failed' }
& uv pip install --python $inferencePython 'ultralytics==8.3.221'
if ($LASTEXITCODE -ne 0) { throw 'Ultralytics install failed' }
& $inferencePython -c "import torch,torchvision; assert torch.__version__=='2.7.1+cpu' and torchvision.__version__=='0.22.1+cpu' and torch.version.cuda is None, 'Unexpected inference backend'"
if ($LASTEXITCODE -ne 0) { throw 'Inference backend verification failed' }
& $inferencePython (Join-Path $projectRoot 'scripts\prepare_yolo_assets.py')
if ($LASTEXITCODE -ne 0) { throw 'YOLO assets preparation failed' }
& uv pip check --python $inferencePython
if ($LASTEXITCODE -ne 0) { throw 'YOLO dependency check failed' }
