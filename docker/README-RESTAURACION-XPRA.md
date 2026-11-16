# CEFIRO Docker + Xpra (bundle completo)

Este ZIP combina **Docker v4** con **Xpra Overlay v2**. Incluye `docker/compose.yaml`, `docker/compose.xpra.yaml`, Dockerfiles, scripts y configuraciones.

No incluye `.env`: mantén el archivo existente, que contiene `XPRA_PASSWORD` y tus preferencias. No hace falta hacer commit ni push.

## Restaurar desde Windows PowerShell

Desde la carpeta `PROJECT-CEFIRO\docker`, descomprime el ZIP en la **raíz del repositorio**, no dentro de `docker`:

```powershell
# Usa la ruta REAL del archivo descargado:
$zip = 'C:\ruta\al\archivo\CEFIRO_Docker_Completo_Xpra_v5.zip'
Expand-Archive -LiteralPath $zip -DestinationPath (Resolve-Path ..).Path -Force
Test-Path .\compose.yaml
Test-Path .\compose.xpra.yaml
```

Si falta `.env`, crea uno desde `.env.example` y ejecuta `./setup-xpra-password.ps1`. Si `.env` ya existe, no lo borres.

```powershell
if (!(Test-Path .env)) { Copy-Item .env.example .env }
.\setup-xpra-password.ps1
docker compose -f compose.yaml -f compose.xpra.yaml config --quiet
docker image inspect cefiro:jazzy-harmonic
```

Si la imagen base no existe, constrúyela una sola vez:
`docker compose -f compose.yaml build sim`

```powershell
docker compose -f compose.yaml -f compose.xpra.yaml build xpra
docker compose -f compose.yaml -f compose.xpra.yaml up -d xpra
docker compose -f compose.yaml -f compose.xpra.yaml ps -a
```

En Windows Xpra Client: `Seamless Session` → `Remote` → protocolo `TCP`, host `127.0.0.1`, puerto `14500`, contraseña de `.env`. Debe aparecer una ventana xterm.

Para abrir **solo la GUI CEFIRO** (sin Gazebo):

```powershell
docker compose -f compose.yaml -f compose.xpra.yaml exec -e DISPLAY=:100 xpra cefiro-run ros2 launch cefiro_gui cefiro_gui.launch.py
```

La GUI y la simulación comparten los directorios de build/install configurados en `docker/compose.yaml`. El servicio `zenoh` sigue siendo opcional y no se inicia por defecto. Xpra no garantiza aceleración 3D por GPU.
