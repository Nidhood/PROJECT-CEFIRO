# Shared CEFIRO Windows helpers. Compatible with Windows PowerShell 5.1.
Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$script:CefiroDockerDir = Split-Path -Parent $PSScriptRoot
$script:CefiroRepoRoot = Split-Path -Parent $script:CefiroDockerDir

function Enter-CefiroDocker {
    if (!(Test-Path (Join-Path $script:CefiroDockerDir 'compose.yaml'))) {
        throw "Missing docker/compose.yaml. Use the complete CEFIRO Docker bundle first."
    }
    if (!(Test-Path (Join-Path $script:CefiroDockerDir 'compose.xpra.yaml'))) {
        throw "Missing docker/compose.xpra.yaml. Restore your Xpra overlay first."
    }
    Push-Location $script:CefiroDockerDir
}

function Assert-CefiroDocker {
    if (-not (Get-Command docker -ErrorAction SilentlyContinue)) {
        throw "Docker CLI missing. Run windows/install-prerequisites.ps1 first."
    }
    & docker info *> $null
    if ($LASTEXITCODE -ne 0) {
        $desktop = Join-Path $env:ProgramFiles 'Docker\Docker\Docker Desktop.exe'
        if (Test-Path $desktop) {
            Write-Host '[CEFIRO] Starting Docker Desktop...'
            Start-Process -FilePath $desktop | Out-Null
            for ($i = 0; $i -lt 45; $i++) {
                Start-Sleep -Seconds 2
                & docker info *> $null
                if ($LASTEXITCODE -eq 0) { break }
            }
        }
    }
    & docker info *> $null
    if ($LASTEXITCODE -ne 0) { throw 'Docker Desktop Engine is not running. Open Docker Desktop and retry.' }
    & docker compose version *> $null
    if ($LASTEXITCODE -ne 0) { throw 'Docker Compose v2 is unavailable.' }
}

function Invoke-CefiroCompose {
    param([Parameter(Mandatory=$true)][string[]]$Arguments)
    & docker compose --profile pi -f compose.yaml -f compose.xpra.yaml @Arguments
    if ($LASTEXITCODE -ne 0) { throw "Docker Compose failed (exit code $LASTEXITCODE)." }
}

function Get-CefiroEnvValue {
    param([string]$Name)
    $path = Join-Path $script:CefiroDockerDir '.env'
    if (!(Test-Path $path)) { return $null }
    $pattern = '^\s*' + [regex]::Escape($Name) + '\s*='
    $line = Get-Content -Path $path | Where-Object { $_ -match $pattern } | Select-Object -Last 1
    if ($null -eq $line) { return $null }
    return (($line -split '=',2)[1]).Trim().Trim('"').Trim("'")
}

function Set-CefiroEnvValue {
    param([string]$Name, [string]$Value)
    $path = Join-Path $script:CefiroDockerDir '.env'
    $lines = @()
    if (Test-Path $path) { $lines = @(Get-Content -Path $path) }
    $pattern = '^\s*' + [regex]::Escape($Name) + '\s*='
    $lines = @($lines | Where-Object { $_ -notmatch $pattern })
    $lines += "$Name=$Value"
    [System.IO.File]::WriteAllLines($path, [string[]]$lines, (New-Object System.Text.UTF8Encoding($false)))
}

function New-CefiroPassword {
    $bytes = New-Object byte[] 24
    $rng = [System.Security.Cryptography.RandomNumberGenerator]::Create()
    try { $rng.GetBytes($bytes) } finally { $rng.Dispose() }
    return [BitConverter]::ToString($bytes).Replace('-', '')
}

function Test-CefiroPort {
    param([string]$Address='127.0.0.1',[int]$Port=14500)
    $tcp = New-Object System.Net.Sockets.TcpClient
    try {
        $task = $tcp.ConnectAsync($Address, $Port)
        return ($task.Wait(2000) -and $tcp.Connected)
    } catch { return $false } finally { $tcp.Dispose() }
}

function Find-CefiroXpraClient {
    $cmd = Get-Command xpra.exe -ErrorAction SilentlyContinue
    if ($cmd) { return $cmd.Source }
    $candidates = @(
        (Join-Path $env:ProgramFiles 'Xpra\Xpra.exe'),
        (Join-Path ${env:ProgramFiles(x86)} 'Xpra\Xpra.exe'),
        (Join-Path $env:LOCALAPPDATA 'Programs\Xpra\Xpra.exe')
    )
    foreach ($candidate in $candidates) { if (Test-Path $candidate) { return $candidate } }
    return $null
}
