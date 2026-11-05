param([switch]$SelfTest,[switch]$Gui,[double]$Duration=0)
$ErrorActionPreference='Stop'
$projectRoot='D:\unitree_g1'
$config=Get-Content (Join-Path $projectRoot 'configs\environment.json') -Raw | ConvertFrom-Json
$launcher=Join-Path $config.isaac_sim_root 'python.bat'
$pythonExe=Join-Path $config.isaac_sim_root 'kit\python\python.exe'
$log=Join-Path $projectRoot ('logs\keyboard_'+(Get-Date -Format yyyyMMdd_HHmmss)+'.log')
$runArgs=@((Join-Path $projectRoot 'scripts\test_keyboard.py'),'--no-ros-env')
if ($SelfTest) { $runArgs+='--self-test' }
if ($Gui) { $runArgs+='--gui' }
if ($Duration -gt 0) { $runArgs+=@('--duration',"$Duration") }
$oldPythonExe=$env:PYTHONEXE
try {
    $env:PYTHONEXE=$pythonExe
    & $launcher @runArgs *> $log
    $exitCode=$LASTEXITCODE
    $errors=@(Select-String -LiteralPath $log -Pattern '\[Error\]|\[Fatal\]|Traceback \(most recent call last\)')
    $summary=[ordered]@{exit_code=$exitCode;error_lines=$errors.Count;log=$log;checked_at=(Get-Date -Format o)}
    $summary | ConvertTo-Json | Set-Content (Join-Path $projectRoot 'logs\keyboard_run_summary.json') -Encoding utf8
    $summary | ConvertTo-Json
    if ($SelfTest) { Get-Content (Join-Path $projectRoot 'logs\keyboard_test_results.json') -Tail 18 }
    if ($exitCode -ne 0 -or $errors.Count -gt 0) { Get-Content $log -Tail 20; throw 'Keyboard run failed; see log.' }
} finally { $env:PYTHONEXE=$oldPythonExe }
