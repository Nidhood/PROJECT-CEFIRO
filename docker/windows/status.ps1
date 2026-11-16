# Inspect containers and read-only ROS topic discovery without launching motors.
[CmdletBinding()]
param([switch]$Topics)
$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'common.ps1')
Assert-CefiroDocker
Enter-CefiroDocker
try {
    Invoke-CefiroCompose -Arguments @('ps','-a')
    if ($Topics) {
        Write-Host '=== Topics visible to Xpra GUI ==='
        Invoke-CefiroCompose -Arguments @('exec','-T','xpra','cefiro-run','ros2','topic','list')
    }
} finally { Pop-Location }
