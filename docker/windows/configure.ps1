# Idempotent project configuration. Preserves existing .env variables/password.
[CmdletBinding()]
param(
    [ValidatePattern('^\d{1,3}(\.\d{1,3}){3}$')][string]$RaspberryIp = '192.168.68.59',
    [ValidateRange(0,232)][int]$RosDomainId = 42,
    [switch]$SkipRaspberryConfig
)
$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'common.ps1')
Enter-CefiroDocker
try {
    if (!(Test-Path '.env')) {
        if (Test-Path '.env.example') { Copy-Item '.env.example' '.env' }
        else { [System.IO.File]::WriteAllText((Join-Path (Get-Location) '.env'), '', (New-Object System.Text.UTF8Encoding($false))) }
    }
    $defaults = @{ LOCAL_UID='1000'; LOCAL_GID='1000'; NOVNC_PORT='6080'; XPRA_PORT='14500' }
    foreach ($name in $defaults.Keys) {
        if (-not (Get-CefiroEnvValue $name)) { Set-CefiroEnvValue $name $defaults[$name] }
    }
    Set-CefiroEnvValue 'ROS_DOMAIN_ID' ([string]$RosDomainId)
    if (-not (Get-CefiroEnvValue 'XPRA_PASSWORD')) {
        Set-CefiroEnvValue 'XPRA_PASSWORD' (New-CefiroPassword)
        Write-Host '[CEFIRO] Generated an Xpra password in docker/.env (not displayed).'
    }
    foreach ($dir in @('build','install','log','records')) {
        New-Item -ItemType Directory -Path (Join-Path $script:CefiroRepoRoot ('.docker-data\' + $dir)) -Force | Out-Null
    }
    if (-not $SkipRaspberryConfig) {
        $configPath = Join-Path $script:CefiroDockerDir 'config\zenoh-windows.json5'
        if (!(Test-Path $configPath)) { throw 'Missing config/zenoh-windows.json5' }
        $config = [System.IO.File]::ReadAllText($configPath)
        $config = [regex]::Replace($config, 'tcp/\d{1,3}(?:\.\d{1,3}){3}:7447', "tcp/$($RaspberryIp):7447")
        # Avoid rewriting when unchanged; this also prevents unnecessary file watcher restarts.
        if ($config -ne [System.IO.File]::ReadAllText($configPath)) {
            Copy-Item -LiteralPath $configPath -Destination ($configPath + '.bak') -Force
            [System.IO.File]::WriteAllText($configPath, $config, (New-Object System.Text.UTF8Encoding($false)))
        }
        Write-Host "[CEFIRO] Raspberry Zenoh endpoint: ${RaspberryIp}:7447"
    }
    if (Get-Command git -ErrorAction SilentlyContinue) {
        $tracked = & git -C $script:CefiroRepoRoot ls-files 'docker/.env'
        if ($tracked) {
            Write-Warning 'docker/.env is tracked by Git. Rotate any exposed password and untrack with: git rm --cached docker/.env'
        }
    }
    Write-Host "[CEFIRO] ROS_DOMAIN_ID=$RosDomainId; preserving existing Xpra password."
    if (Get-Command docker -ErrorAction SilentlyContinue) {
        & docker compose --profile pi -f compose.yaml -f compose.xpra.yaml config --quiet
        if ($LASTEXITCODE -ne 0) { throw 'Docker Compose config validation failed.' }
        Write-Host '[CEFIRO] Docker Compose validation passed.'
    } else { Write-Host '[CEFIRO] Docker CLI not installed yet; Compose validation skipped.' }
} finally { Pop-Location }
