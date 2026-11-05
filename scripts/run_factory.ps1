param([switch]$Gui, [switch]$Validate, [int]$Frames=0)
$ErrorActionPreference='Stop'
$projectRoot='D:\unitree_g1'
$config=Get-Content (Join-Path $projectRoot 'configs\environment.json') -Raw | ConvertFrom-Json
$launcher=Join-Path $config.isaac_sim_root 'python.bat'
$pythonExe=Join-Path $config.isaac_sim_root 'kit\python\python.exe'
if (!(Test-Path -LiteralPath $launcher) -or !(Test-Path -LiteralPath $pythonExe)) { throw 'Verify Isaac Sim paths in environment.json' }
$stamp=Get-Date -Format yyyyMMdd_HHmmss
$log=Join-Path $projectRoot "logs\factory_physics_$stamp.log"
$runArgs=@((Join-Path $projectRoot 'scripts\load_factory.py'),'--no-ros-env')
if ($Gui) { $runArgs+='--gui' }
if ($Validate -or !$Gui) { $runArgs+='--validate' }
if ($Frames -gt 0) { $runArgs+=@('--frames',"$Frames") }
$oldPythonExe=$env:PYTHONEXE
try {
    $env:PYTHONEXE=$pythonExe
    & $launcher @runArgs *> $log
    $code=$LASTEXITCODE
    $errors=@(Select-String -LiteralPath $log -Pattern '\[Error\]|\[Fatal\]|Traceback \(most recent call last\)')
    $summary=[ordered]@{exit_code=$code;error_lines=$errors.Count;log=$log;checked_at=(Get-Date -Format o)}
    $summary | ConvertTo-Json | Set-Content (Join-Path $projectRoot 'logs\factory_run_summary.json') -Encoding utf8
    $summary | ConvertTo-Json
    if ($code -ne 0 -or $errors.Count -gt 0) { Get-Content -LiteralPath $log -Tail 30; throw 'Factory run failed; inspect log/result JSON.' }
    $result=Get-Content (Join-Path $projectRoot 'logs\factory_physics_result.json') -Raw | ConvertFrom-Json
    if ($result.status -eq 'failed' -or (($Validate -or !$Gui) -and $result.status -ne 'passed')) { throw 'Missing successful validation result.' }
    $result | Select-Object status,physics_engine,mesh_count,collider_count,checks | ConvertTo-Json -Depth 4
} finally { $env:PYTHONEXE=$oldPythonExe }
