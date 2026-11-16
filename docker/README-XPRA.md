# CEFIRO + Xpra (Windows native GUI, optional Compose overlay)

Xpra displays your Linux Qt/Gazebo/RViz windows on Windows without noVNC.
This does not add GPU acceleration: the existing image forces Mesa software GL.
The overlay keeps the working sim/observer/Zenoh setup and adds the independent `xpra` service.

## Official client on Windows
Download and install the signed **stable x64 EXE** from:
https://xpra.org/stable/windows/Xpra-x86_64_Setup.exe
Official instructions: https://github.com/Xpra-org/xpra#installation

## 1. Prepare password and build (PowerShell 5.1+ from `PROJECT-CEFIRO\docker`)

Download the ZIP from this chat first. `Expand-Archive` does not download files; use the actual location of the downloaded file.

```powershell
$zip = Join-Path $env:USERPROFILE "Downloads\CEFIRO_Xpra_Windows_Overlay_v2.zip"
if (!(Test-Path $zip)) { throw "ZIP not found at $zip. Download the file or update the path." }
Expand-Archive -LiteralPath $zip -DestinationPath (Resolve-Path ..).Path -Force
```


```powershell
if (!(Test-Path .env)) { Copy-Item .env.example .env }
if (-not (Select-String -Path .env -Pattern '^XPRA_PASSWORD=' -Quiet)) {
  $bytes = New-Object byte[] 16
  $rng = [System.Security.Cryptography.RandomNumberGenerator]::Create()
  try { $rng.GetBytes($bytes) } finally { $rng.Dispose() }
  $pass = [BitConverter]::ToString($bytes).Replace("-", "")
  Add-Content .env "XPRA_PASSWORD=$pass"
  Write-Host 'Xpra password written to docker/.env; use it in the Xpra client.'
}

docker compose -f compose.yaml -f compose.xpra.yaml config --quiet
docker compose -f compose.yaml -f compose.xpra.yaml build xpra
docker compose -f compose.yaml -f compose.xpra.yaml up -d xpra

docker compose -f compose.yaml -f compose.xpra.yaml ps -a
```

For password retrieval:

```powershell
(Get-Content .env | Where-Object { $_ -match '^XPRA_PASSWORD=' }) -replace '^XPRA_PASSWORD=', ''
```

## 2. Connect native Windows client

Open **Xpra** client in Windows; select TCP; host `127.0.0.1`; port `14500`;
enter the password from `.env`. The startup `xterm` should appear as an independent Windows window.

Do not use `http://localhost:6080/` for this mode; that is the old noVNC.

## 3. Launch only Qt GUI (Gazebo is NOT required)

```powershell
docker compose -f compose.yaml -f compose.xpra.yaml exec -e DISPLAY=:100 xpra cefiro-run ros2 launch cefiro_gui cefiro_gui.launch.py
```

Or Gazebo only:

```powershell
docker compose -f compose.yaml -f compose.xpra.yaml exec -e DISPLAY=:100 xpra cefiro-run ros2 launch cefiro_gz start_gz.launch.py
```

Or RViz:

```powershell
docker compose -f compose.yaml -f compose.xpra.yaml exec -e DISPLAY=:100 xpra cefiro-run rviz2
```

## 4. Test performance and stop

```powershell
docker compose -f compose.yaml -f compose.xpra.yaml exec -e DISPLAY=:100 xpra cefiro-run glxinfo -B
docker compose -f compose.yaml -f compose.xpra.yaml logs --tail=80 xpra
docker compose -f compose.yaml -f compose.xpra.yaml stop xpra
```

If `glxinfo -B` reports `llvmpipe`, graphics are still software-rendered (CPU).
Xpra helps window responsiveness, but not necessarily Gazebo 3D FPS.

## Security notes

- Binds host port only to `127.0.0.1`; unencrypted TCP must stay local.
- Password is configured via Xpra's official `auth=env` mechanism.
- `.env` contains a secret; do not commit or share it.
- Do not enable remote physical motor actuation merely to validate GUI connectivity.

## Status

Dockerfiles and Compose configuration were validated statically. Xpra session runtime,
Windows client performance, and GPU support were **not tested** on the user's computer.
