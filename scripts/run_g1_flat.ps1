param([switch]$Gui,[switch]$Passive,[switch]$AutoClose,[switch]$NoCapture)
$ErrorActionPreference='Stop'
$projectRoot='D:\unitree_g1'
$environmentConfig=Get-Content (Join-Path $projectRoot 'configs\environment.json') -Raw | ConvertFrom-Json
$launcher=Join-Path $environmentConfig.isaac_sim_root 'python.bat'
$pythonExe=Join-Path $environmentConfig.isaac_sim_root 'kit\python\python.exe'
if (!(Test-Path -LiteralPath $launcher) -or !(Test-Path -LiteralPath $pythonExe)) { throw 'Verify environment.json installation paths' }
$stamp=Get-Date -Format yyyyMMdd_HHmmss
$log=Join-Path $projectRoot "logs\g1_flat_$stamp.log"
$runArgs=@((Join-Path $projectRoot 'scripts\g1_flat_test.py'),'--no-ros-env')
if ($Gui) { $runArgs+='--gui' }
if ($Passive) { $runArgs+='--passive' }
if ($AutoClose) { $runArgs+='--exit-after-test' }
if ($NoCapture) { $runArgs+='--no-capture' }
$oldPythonExe=$env:PYTHONEXE
try {
    $env:PYTHONEXE=$pythonExe
    & $launcher @runArgs *> $log
    $code=$LASTEXITCODE
    $errorLines=@(Select-String -LiteralPath $log -Pattern '\[Error\]|\[Fatal\]|Traceback \(most recent call last\)')
    $summary=[ordered]@{exit_code=$code;error_lines=$errorLines.Count;log=$log;checked_at=(Get-Date -Format o)}
    $summary | ConvertTo-Json | Set-Content (Join-Path $projectRoot 'logs\g1_run_summary.json') -Encoding utf8
    $summary | ConvertTo-Json
    if ($code -ne 0 -or $errorLines.Count -gt 0) { Get-Content $log -Tail 25; throw 'G1 test failed; inspect log' }
    $result=Get-Content (Join-Path $projectRoot 'logs\g1_articulation_report.json') -Raw | ConvertFrom-Json
    if ($result.status -ne 'passed') { throw 'G1 articulation checks did not pass' }
    $result | Select-Object status,dof_count,body_count,collider_count,fallen_during_test,checks | ConvertTo-Json -Depth 4
} finally { $env:PYTHONEXE=$oldPythonExe }
