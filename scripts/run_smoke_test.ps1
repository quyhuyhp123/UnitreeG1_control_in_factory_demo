$ErrorActionPreference = 'Stop'
$projectRoot = 'D:\unitree_g1'
$config = Get-Content (Join-Path $projectRoot 'configs\environment.json') -Raw | ConvertFrom-Json
$launcher = Join-Path $config.isaac_sim_root 'python.bat'
$pythonExe = Join-Path $config.isaac_sim_root 'kit\python\python.exe'
if (!(Test-Path -LiteralPath $launcher) -or !(Test-Path -LiteralPath $pythonExe)) {
    throw 'Isaac Sim paths no longer exist; verify environment.json.'
}
$stamp = Get-Date -Format 'yyyyMMdd_HHmmss'
$log = Join-Path $projectRoot "logs\smoke_$stamp.log"
$oldPythonExe = $env:PYTHONEXE
try {
    # Avoid python.bat copying python.exe to kit.exe in the installation.
    $env:PYTHONEXE = $pythonExe
    & $launcher (Join-Path $projectRoot 'scripts\smoke_test.py') --no-ros-env *> $log
    $smokeExit = $LASTEXITCODE
    Write-Output "Exit code: $smokeExit; log: $log"
    Get-Content (Join-Path $projectRoot 'logs\smoke_result.json')
    if ($smokeExit -ne 0) { throw "Isaac Sim failed: $smokeExit" }
    $result = Get-Content (Join-Path $projectRoot 'logs\smoke_result.json') -Raw | ConvertFrom-Json
    if ($result.status -notin @('passed', 'stage_updates_passed') -or $result.updates -ne 120) {
        throw 'Process exited without completing the empty-stage probe.'
    }
} finally {
    $env:PYTHONEXE = $oldPythonExe
}
