# Build base ROS/Gazebo/Qt image, Xpra image, then all eight ROS packages.
[CmdletBinding()]
param([switch]$SkipColcon)
$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'common.ps1')
& (Join-Path $PSScriptRoot 'configure.ps1')
Assert-CefiroDocker
Enter-CefiroDocker
try {
    Write-Host '=== CEFIRO / Build Ubuntu 24.04 ROS Jazzy + Gazebo + Qt6 ==='
    # Sequential because Dockerfile.xpra FROM cefiro:jazzy-harmonic.
    Invoke-CefiroCompose -Arguments @('build','sim')
    Write-Host '=== CEFIRO / Build Xpra seamless server ==='
    Invoke-CefiroCompose -Arguments @('build','xpra')
    if (-not $SkipColcon) {
        Write-Host '=== CEFIRO / Compile all source ROS packages ==='
        Invoke-CefiroCompose -Arguments @(
            'run','--rm','--no-deps','sim','cefiro-run','colcon','build',
            '--base-paths','/cefiro_ws/src/PROJECT-CEFIRO',
            '--symlink-install','--parallel-workers','2',
            '--event-handlers','console_direct+'
        )
        Write-Host '[CEFIRO] Validating primary ROS package prefixes...'
        foreach ($pkg in @('cefiro_msgs','cefiro_gui','cefiro_control','cefiro_hardware_interface','cefiro_gz','cefiro_description','cefiro_sensors','cefiro_actuators')) {
            Invoke-CefiroCompose -Arguments @('run','--rm','--no-deps','sim','cefiro-run','ros2','pkg','prefix',$pkg)
        }
    }
    Write-Host '[CEFIRO] Installation completed. Use windows/start.ps1.'
} finally { Pop-Location }
