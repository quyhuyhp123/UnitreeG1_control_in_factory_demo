$ErrorActionPreference='Stop'
$projectRoot='D:\unitree_g1'
$destinationRoot=Join-Path $projectRoot 'assets\robot\unitree_model'
$manifest=Get-Content (Join-Path $projectRoot 'assets\robot\upstream\download_manifest.json') -Raw | ConvertFrom-Json
foreach ($entry in $manifest) {
    $destination=[IO.Path]::GetFullPath((Join-Path $destinationRoot $entry.path))
    if (!$destination.StartsWith($destinationRoot+'\',[StringComparison]::OrdinalIgnoreCase)) { throw 'Asset destination outside robot folder' }
    if (Test-Path -LiteralPath $destination) {
        if ((Get-FileHash -LiteralPath $destination -Algorithm SHA256).Hash -ne $entry.sha256) { throw "Existing asset differs; inspect before replacing: $destination" }
        Write-Output "Verified existing $($entry.path)"
        continue
    }
    New-Item -ItemType Directory -Path (Split-Path $destination) -Force | Out-Null
    $partial=$destination+'.partial'
    if (Test-Path -LiteralPath $partial) { throw "Inspect partial download first: $partial" }
    Invoke-WebRequest -Uri $entry.url -OutFile $partial
    if ((Get-FileHash -LiteralPath $partial -Algorithm SHA256).Hash -ne $entry.sha256) { throw "Downloaded hash mismatch: $partial" }
    Move-Item -LiteralPath $partial -Destination $destination
    Write-Output "Downloaded and verified $($entry.path)"
}
