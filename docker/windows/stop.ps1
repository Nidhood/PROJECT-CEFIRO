# Stop and remove CEFIRO containers/network. Preserve bind-mounted data/images.
[CmdletBinding()]
param()
$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'common.ps1')
Assert-CefiroDocker
Enter-CefiroDocker
try {
    Invoke-CefiroCompose -Arguments @('down','--remove-orphans')
    Write-Host '[CEFIRO] Stopped. Images, project files, .env and .docker-data are preserved.'
} finally { Pop-Location }
