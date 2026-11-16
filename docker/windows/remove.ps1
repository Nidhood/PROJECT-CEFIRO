# Safe cleanup. -RemoveImages removes only CEFIRO-tagged images.
# -PurgeData requires typing an exact confirmation before deleting build/install/log/records.
[CmdletBinding()]
param([switch]$RemoveImages,[switch]$PurgeData)
$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'common.ps1')
Assert-CefiroDocker
Enter-CefiroDocker
try {
    Invoke-CefiroCompose -Arguments @('down','--remove-orphans')
    if ($RemoveImages) {
        foreach ($img in @('cefiro:xpra-jazzy-harmonic','cefiro:jazzy-harmonic')) {
            & docker image inspect $img *> $null
            if ($LASTEXITCODE -eq 0) {
                & docker image rm $img
                if ($LASTEXITCODE -ne 0) { Write-Warning "Image still in use or could not be removed: $img" }
            }
        }
    }
    if ($PurgeData) {
        Write-Warning 'This will PERMANENTLY DELETE compiled output AND experimental records/logs in .docker-data.'
        $answer = Read-Host 'Type DELETE CEFIRO DATA to confirm'
        if ($answer -cne 'DELETE CEFIRO DATA') {
            Write-Host '[CEFIRO] Data deletion cancelled.'
        } else {
            $path = Join-Path $script:CefiroRepoRoot '.docker-data'
            if (Test-Path -LiteralPath $path) {
                Remove-Item -LiteralPath $path -Recurse -Force
                Write-Host '[CEFIRO] Removed only .docker-data directory.'
            }
        }
    }
    Write-Host '[CEFIRO] Project source, docker/.env, Docker Desktop, and Windows Xpra client were NOT removed.'
} finally { Pop-Location }
