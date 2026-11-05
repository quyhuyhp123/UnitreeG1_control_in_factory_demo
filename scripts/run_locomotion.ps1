param([switch]$Gui,[switch]$NoCapture,[switch]$StandOnly,[switch]$InPlaceTurns)
$ErrorActionPreference='Stop'
$projectRoot='D:\unitree_g1'
$config=Get-Content (Join-Path $projectRoot 'configs\environment.json') -Raw | ConvertFrom-Json
$launcher=Join-Path $config.isaac_sim_root 'python.bat'
$pythonExe=Join-Path $config.isaac_sim_root 'kit\python\python.exe'
$stamp=Get-Date -Format yyyyMMdd_HHmmss
$log=Join-Path $projectRoot "logs\locomotion_$stamp.log"
$runArgs=@((Join-Path $projectRoot 'scripts\test_locomotion.py'),'--no-ros-env')
if ($Gui) { $runArgs+='--gui' }
if ($NoCapture) { $runArgs+='--no-capture' }
if ($StandOnly) { $runArgs+='--stand-only' }
if ($InPlaceTurns) { $runArgs+='--in-place-turns' }
$oldPythonExe=$env:PYTHONEXE
try {
    $env:PYTHONEXE=$pythonExe
    & $launcher @runArgs *> $log
    $code=$LASTEXITCODE
    $errors=@(Select-String -LiteralPath $log -Pattern '\[Error\]|\[Fatal\]|Traceback \(most recent call last\)')
    $result=Get-Content (Join-Path $projectRoot 'logs\locomotion_result.json') -Raw | ConvertFrom-Json
    $summary=[ordered]@{exit_code=$code;error_lines=$errors.Count;status=$result.status;log=$log;checked_at=(Get-Date -Format o)}
    $summary | ConvertTo-Json | Set-Content (Join-Path $projectRoot 'logs\locomotion_run_summary.json') -Encoding utf8
    $summary | ConvertTo-Json
    Get-Content -LiteralPath $log -Tail 8
    if ($code -ne 0 -or $errors.Count -gt 0 -or $result.status -ne 'passed') { throw 'Locomotion not accepted; inspect result and telemetry.' }
} finally { $env:PYTHONEXE=$oldPythonExe }
