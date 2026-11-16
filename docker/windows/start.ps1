# Start sim/observer/Zenoh/Xpra, and launch cefiro_gui in Xpra display :100.
[CmdletBinding()]
param([switch]$NoPi, [switch]$SkipGui, [switch]$SkipClient)
$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'common.ps1')
Assert-CefiroDocker
Enter-CefiroDocker
try {
    if (-not (Get-CefiroEnvValue 'XPRA_PASSWORD')) { throw 'Run windows/configure.ps1 first (XPRA_PASSWORD missing).' }
    Invoke-CefiroCompose -Arguments @('config','--quiet')
    $services = @('sim','observer','xpra')
    if (-not $NoPi) { $services += 'zenoh' }
    Invoke-CefiroCompose -Arguments (@('up','-d','--no-build') + $services)

    $xpraPort = Get-CefiroEnvValue 'XPRA_PORT'
    if (-not $xpraPort) { $xpraPort = '14500' }
    $serverReady = $false
    for ($attempt=0; $attempt -lt 15; $attempt++) {
        if (Test-CefiroPort -Port ([int]$xpraPort)) { $serverReady = $true; break }
        Start-Sleep -Seconds 1
    }
    if (-not $serverReady) {
        Invoke-CefiroCompose -Arguments @('logs','--tail=80','xpra')
        throw "Xpra isn't accepting local TCP connections on port $xpraPort."
    }

    if (-not $SkipGui) {
        Write-Host '[CEFIRO] Launching Qt6 GUI in the Xpra container...'
        # Avoid duplicate launch. No real hardware process or motor control is started.
        & docker compose --profile pi -f compose.yaml -f compose.xpra.yaml exec -T xpra bash -lc "pgrep -f '[c]efiro_gui_node' >/dev/null"
        $alreadyRunning = ($LASTEXITCODE -eq 0)
        if (-not $alreadyRunning) {
            & docker compose --profile pi -f compose.yaml -f compose.xpra.yaml exec -d -e 'DISPLAY=:100' xpra bash -lc 'cefiro-run ros2 launch cefiro_gui cefiro_gui.launch.py >> /cefiro_data/log/cefiro-gui-xpra.log 2>&1'
            if ($LASTEXITCODE -ne 0) { throw 'Could not launch the Qt6 GUI. Run windows/install-project.ps1 first.' }
            Start-Sleep -Seconds 3
            & docker compose --profile pi -f compose.yaml -f compose.xpra.yaml exec -T xpra bash -lc "pgrep -f '[c]efiro_gui_node' >/dev/null || pgrep -f '[r]os2 launch cefiro_gui' >/dev/null"
            if ($LASTEXITCODE -ne 0) {
                Write-Warning 'Qt6 GUI may have exited. Check .docker-data/log/cefiro-gui-xpra.log'
            }
        } else { Write-Host '[CEFIRO] Qt6 GUI is already running.' }
    }

    if (-not $SkipClient) {
        $client = Find-CefiroXpraClient
        if ($client) {
            Write-Host '[CEFIRO] Opening Xpra Windows client...'
            Start-Process -FilePath $client -ArgumentList @('attach', "tcp://127.0.0.1:$xpraPort") | Out-Null
        } else {
            Write-Host "[CEFIRO] Open Xpra manually: Seamless / TCP / 127.0.0.1 / port $xpraPort / Start & Attach"
        }
    }
    Write-Host '[CEFIRO] Docker services are running.'
    Write-Host 'GUI log: .docker-data/log/cefiro-gui-xpra.log'
    Write-Host 'noVNC fallback: http://127.0.0.1:6080/vnc.html?autoconnect=1'
    Write-Host 'The Raspberry Pi Zenoh process must be started separately on the Pi.'
} finally { Pop-Location }
