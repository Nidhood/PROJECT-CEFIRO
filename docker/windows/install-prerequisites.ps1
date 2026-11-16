# Verify/install Windows prerequisites. -InstallMissing explicitly permits installers.
[CmdletBinding()]
param([switch]$InstallMissing)
$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'common.ps1')

Write-Host '=== CEFIRO / Windows prerequisites ==='
if ([Environment]::OSVersion.Platform -ne [PlatformID]::Win32NT) {
    throw 'This script is for Windows PowerShell. Linux dependencies live in Dockerfile.'
}

$winget = Get-Command winget.exe -ErrorAction SilentlyContinue
$dockerExe = Get-Command docker.exe -ErrorAction SilentlyContinue
$gitExe = Get-Command git.exe -ErrorAction SilentlyContinue
$xpraExe = Find-CefiroXpraClient

Write-Host ("Docker CLI: " + $(if($dockerExe){'FOUND'}else{'MISSING'}))
Write-Host ("Git:        " + $(if($gitExe){'FOUND'}else{'MISSING (optional if repository already exists)'}))
Write-Host ("Xpra:       " + $(if($xpraExe){'FOUND'}else{'MISSING / unknown install path'}))
Write-Host ("winget:     " + $(if($winget){'FOUND'}else{'NOT AVAILABLE'}))

if ($InstallMissing) {
    if (-not $dockerExe) {
        if ($winget) {
            & winget install --id Docker.DockerDesktop --exact --source winget --accept-source-agreements --accept-package-agreements
            if ($LASTEXITCODE -ne 0) { throw 'Docker Desktop installer failed; install from https://www.docker.com/products/docker-desktop/' }
        } else {
            Write-Warning 'winget is unavailable. Opening official Docker Desktop download page.'
            Start-Process 'https://www.docker.com/products/docker-desktop/'
        }
    }
    if (-not $gitExe) {
        if ($winget) {
            & winget install --id Git.Git --exact --source winget --accept-source-agreements --accept-package-agreements
            if ($LASTEXITCODE -ne 0) { Write-Warning 'Git install failed; install manually if needed.' }
        } else { Write-Host 'Git is optional when the repository is already present.' }
    }
    if (-not $xpraExe) {
        $url = 'https://xpra.org/stable/windows/Xpra-x86_64_Setup.exe'
        $installer = Join-Path $env:TEMP 'CEFIRO-Xpra-x86_64_Setup.exe'
        Write-Host '[CEFIRO] Downloading official Xpra Windows installer (interactive)...'
        [Net.ServicePointManager]::SecurityProtocol = [Net.ServicePointManager]::SecurityProtocol -bor [Net.SecurityProtocolType]::Tls12
        Invoke-WebRequest -UseBasicParsing -Uri $url -OutFile $installer
        Write-Host "Verify publisher and Windows security dialog before installing: $installer"
        Start-Process -FilePath $installer -Wait
    }
}

Write-Host ''
Write-Host 'ROS 2 Jazzy, Gazebo Harmonic, Qt6, colcon, and Xpra Linux server are installed INSIDE Docker images.'
Write-Host 'Docker Desktop may use its internal WSL 2 backend; no separate Ubuntu installation is needed.'
Write-Host 'Next: windows/configure.ps1, then windows/install-project.ps1.'
