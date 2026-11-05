param([switch]$Flat,[switch]$SelfTest,[switch]$Headless,[switch]$Overview,[double]$Duration=0)
$ErrorActionPreference='Stop'
$projectRoot='D:\unitree_g1'
$cfg=Get-Content (Join-Path $projectRoot 'configs\environment.json') -Raw | ConvertFrom-Json
$launcher=Join-Path $cfg.isaac_sim_root 'python.bat'
if (-not (Test-Path -LiteralPath $launcher)) { throw "Missing Isaac launcher: $launcher" }
$log=Join-Path $projectRoot ('logs\factory_g1_'+(Get-Date -Format yyyyMMdd_HHmmss)+'.log')
$runArgs=@((Join-Path $projectRoot 'scripts\run_factory_g1.py'),'--no-ros-env')
if ($Flat) { $runArgs+='--flat' }; if ($SelfTest) { $runArgs+='--self-test' }
if ($Headless) { $runArgs+='--headless' }; if ($Overview) { $runArgs+='--overview' }
if ($Duration -gt 0) { $runArgs+=@('--duration',"$Duration") }
$oldPythonExe=$env:PYTHONEXE
$runStarted=Get-Date
Write-Host "Starting G1 simulation. Log: $log"
try {
    $env:PYTHONEXE=Join-Path $cfg.isaac_sim_root 'kit\python\python.exe'
    # Windows PowerShell 5.1 promotes native stderr to NativeCommandError.
    # Preserve stderr in the log, then assess the real process exit code.
    $savedErrorPreference=$ErrorActionPreference
    try {
        $ErrorActionPreference='Continue'
        & $launcher @runArgs *> $log
        $exitCode=$LASTEXITCODE
    } finally { $ErrorActionPreference=$savedErrorPreference }
    $errors=@(Select-String -LiteralPath $log -Pattern '\[Error\]|\[Fatal\]|Traceback \(most recent call last\)')
    $summary=[ordered]@{exit_code=$exitCode;error_lines=$errors.Count;log=$log;flat_mode=[bool]$Flat;self_test=[bool]$SelfTest;checked_at=(Get-Date -Format o)}
    $summary['native_stderr_diagnostics']=@(Select-String -LiteralPath $log -Pattern 'Unsupported:|NativeCommandError').Count
    $summary['finished_marker_seen']=[bool](Select-String -LiteralPath $log -Pattern '^FACTORY_FINISHED ')
    $summary['status']=if ($exitCode -eq 0 -and $errors.Count -eq 0 -and $summary['finished_marker_seen']) {'completed'} else {'failed_or_crashed'}
    if ($SelfTest) {
        $resultName=if ($Flat) {'factory_g1_flat_results.json'} else {'factory_g1_results.json'}
        $resultPath=Join-Path $projectRoot "logs\$resultName"
        if ((Test-Path $resultPath) -and (Get-Item $resultPath).LastWriteTime -ge $runStarted) {
            $result=Get-Content $resultPath -Raw | ConvertFrom-Json; $summary['test_status']=$result.status; $summary['failed_tests']=$result.failed_tests
        } else { $summary['test_status']='not_produced_for_this_run' }
    }
    $summary | ConvertTo-Json -Depth 4 | Set-Content (Join-Path $projectRoot 'logs\factory_g1_run_summary.json') -Encoding utf8
    $summary | ConvertTo-Json -Depth 4
    if ($summary['status'] -ne 'completed') { Get-Content $log -Tail 22; throw "Simulation failed/crashed. See $log" }
} finally { $env:PYTHONEXE=$oldPythonExe }
