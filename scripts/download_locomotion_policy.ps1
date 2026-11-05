$ErrorActionPreference='Stop'
$projectRoot='D:\unitree_g1'
$folder=Join-Path $projectRoot 'checkpoints\unitree_g1_velocity_v0'
$provenance=Get-Content (Join-Path $folder 'provenance.json') -Raw | ConvertFrom-Json
$prefix='deploy/robots/g1_29dof/config/policy/velocity/v0'
foreach ($entry in @(@{name='policy.onnx';path="$prefix/exported/policy.onnx"},@{name='deploy.yaml';path="$prefix/params/deploy.yaml"})) {
    $target=Join-Path $folder $entry.name
    $expected=($provenance.files | Where-Object file -eq $entry.name).sha256
    if (Test-Path -LiteralPath $target) {
        if ((Get-FileHash -LiteralPath $target -Algorithm SHA256).Hash -ne $expected) { throw "Existing file differs: $target" }
        Write-Output "Verified $($entry.name)"
        continue
    }
    $partial=$target+'.partial'
    if (Test-Path -LiteralPath $partial) { throw "Inspect partial file: $partial" }
    Invoke-WebRequest "https://raw.githubusercontent.com/unitreerobotics/unitree_rl_lab/$($provenance.commit)/$($entry.path)" -OutFile $partial
    if ((Get-FileHash -LiteralPath $partial -Algorithm SHA256).Hash -ne $expected) { throw 'Checkpoint hash mismatch' }
    Move-Item -LiteralPath $partial -Destination $target
    Write-Output "Downloaded verified $($entry.name)"
}
