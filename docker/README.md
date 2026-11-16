# CEFIRO Docker v4 — Windows + Ubuntu (ROS 2 Jazzy / Gazebo Harmonic / Qt 6)

Esta carpeta contiene un entorno **completo de infraestructura Docker** para agregar a
una copia local existente de `PROJECT-CEFIRO`: no incluye ni reemplaza los paquetes
`cefiro_*`, que deben permanecer al nivel superior del repositorio. No requiere
instalar Ubuntu WSL, ROS 2 ni Gazebo en Windows; Docker Desktop utiliza su propia
infraestructura Linux. No se realiza ningún push o commit.

**Estado de validación:** el usuario ya compiló con éxito la imagen v3 y ejecutó
`sim` y `observer` en Docker Desktop. Esta versión v4 añade `iputils-ping` en una
capa pequeña, integra Zenoh opcional para la Raspberry y corrige CRLF en scripts.
Se ha revisado la estructura de los archivos, pero **no se ha probado la ruta TCP
real hacia la Raspberry ni una nueva construcción v4 en este entorno**.

## Estructura

```text
PROJECT-CEFIRO/
  cefiro_gui/          ... código ROS original
  cefiro_control/      ... código ROS original
  cefiro_gz/           ... código ROS original
  docker/
    Dockerfile
    compose.yaml
    compose.native-linux.yaml
    compose.nvidia.yaml
    compose.wslg.yaml
    config/cyclonedds.xml
    config/zenoh-windows.json5
    raspberry/zenoh-pi.json5
    raspberry/install-zenoh.sh
    scripts/cefiro-run.sh
    scripts/entrypoint.sh
    scripts/start-desktop.sh
    scripts/quick-check.ps1
    scripts/fix-ros-python-shebangs.ps1
    .env.example
    ...
  .docker-data/          ... build, install, log, records (persistentes)
```

## Preparación desde Windows PowerShell

A) Descomprimir este ZIP en la **raíz** de `PROJECT-CEFIRO`. El ZIP tiene la
carpeta `docker/` y sustituye los archivos Docker v3; **guardar antes una copia de
tu `docker/.env` y cualquier cambio manual**. No uses `git push`.

B) Desde `PROJECT-CEFIRO\docker`:

```powershell
if (!(Test-Path .env)) { Copy-Item .env.example .env }
@('build','install','log','records') | ForEach-Object {
   New-Item -ItemType Directory -Force "../.docker-data/$_" | Out-Null
}
docker compose config --quiet
docker compose build
docker compose up -d sim observer
docker compose ps
```

La instalación adicional de ping está en una capa propia y debería conservar
la caché costosa de ROS 2/Gazebo si el Dockerfile/base no han cambiado.

**Docker Desktop no instala Ubuntu WSL por separado**. El código compartido está
en Windows. Para compilaciones `colcon` más rápidas, evitar OneDrive/NTFS si es
posible, y copiar el repositorio a `C:\dev\PROJECT-CEFIRO`.

## Red: ping e Internet

```powershell
docker compose exec sim cefiro-run ping -c 3 observer
docker compose exec sim cefiro-run ping -c 3 8.8.8.8
docker compose exec sim cefiro-run ping -c 3 192.168.68.59
docker compose exec sim cefiro-run curl -I --max-time 10 https://www.google.com
```

`ping` comprueba ICMP, no confirma por sí solo DDS ni la disponibilidad de TCP.
Si la Raspberry bloquea ICMP, `Test-NetConnection ... -Port 7447` es más útil.

## GUI Qt sin ejecutar Gazebo

```powershell
docker compose up -d sim
docker compose exec sim cefiro-run colcon build --packages-up-to cefiro_gui --parallel-workers 2
docker compose exec sim cefiro-run ros2 launch cefiro_gui cefiro_gui.launch.py
```

Ver la interfaz desde el navegador:
`http://127.0.0.1:6080/vnc.html?autoconnect=1`
La visualización es Xvfb/noVNC (software/OpenGL CPU). Para trabajar con gráficos
más fluidos, se debe configurar por separado el acceso a OpenGL/WSLg/X11.

Para ejecutar Gazebo explícitamente (en una segunda terminal):

```powershell
docker compose exec sim cefiro-run ros2 launch cefiro_gz start_gz.launch.py
```

**Si `cefiro_balance_node.py` falla con `python3\r`:** Windows convirtió un script
Python a CRLF. Detén primero la simulación; luego ejecuta de forma voluntaria
`./scripts/fix-ros-python-shebangs.ps1` desde PowerShell (modifica solo los scripts
locales de Python con shebang y CRLF). Después recompila los paquetes afectados.
No cambia nada en GitHub sin tu intervención.

## Raspberry Pi — Zenoh TCP (solo telemetría)

Arquitectura:

- Raspberry Pi: IP `192.168.68.59`, ROS 2 DDS dominio **30**.
- Contenedores en Windows: ROS 2 DDS dominio **42**, red Docker Compose.
- Dos puentes Zenoh 1.x conectados por TCP en puerto **7447**.
- La configuración de `allow` envía exclusivamente estos tópicos:
  `/real/imu/data`, `/real/magnetometer/data`, `/real/bmp581/air_pressure`,
  `/real/cefiro/hardware/status`, `/real/cefiro/control/status`,
  `/real/cefiro_actuators/telemetry` y `/real/cefiro_actuators/measured_rpm`.
- **No se transportan comandos**, servicios ni acciones hacia el hardware.
  Es un filtro de software, no una frontera de seguridad criptográfica.
  Mantén parada la actuación física y utiliza una LAN de confianza.

### A. Enviar configuración a la Raspberry (PowerShell)

Desde `PROJECT-CEFIRO\docker`:

```powershell
scp .\raspberry\zenoh-pi.json5 nidhood@192.168.68.59:~/zenoh-pi.json5
scp .\raspberry\install-zenoh.sh nidhood@192.168.68.59:~/install-zenoh.sh
```

### B. Instalar y arrancar el bridge en la Raspberry

Desde su propia terminal (no hace falta Docker allí):

```bash
bash ~/install-zenoh.sh
# Keep this terminal running during the connectivity test.
ROS_DOMAIN_ID=30 zenoh-bridge-ros2dds -c ~/zenoh-pi.json5
```

Si ya tienes Zenoh instalado, omite `install-zenoh.sh`.
Asegura que el bridge escuche `192.168.68.59:7447`. Si cambia la IP,
actualiza los dos archivos JSON5 antes de levantar los puentes.
Si los puentes tienen versiones incompatibles, usa versiones compatibles de
Zenoh en ambos extremos; el servicio Docker está fijado a `1.10.1`.

### C. Iniciar Zenoh en Docker/Windows

Desde otra terminal PowerShell, con el bridge de Raspberry encendido:

```powershell
Test-NetConnection 192.168.68.59 -Port 7447
docker compose --profile pi up -d sim observer zenoh
docker compose --profile pi ps
docker compose --profile pi logs --tail=80 zenoh
docker compose exec sim cefiro-run ros2 daemon stop
docker compose exec sim cefiro-run ros2 topic list
docker compose exec sim cefiro-run ros2 topic echo /real/imu/data --qos-reliability best_effort
```

Si `Test-NetConnection` responde `False`, revisa el bridge en Pi, el Wi-Fi,
aislamiento de clientes o el firewall. Si la conexión TCP responde `True`
pero no hay topics, revisa `docker compose --profile pi logs zenoh`, el RMW
de Raspberry y los permisos DDS/Zenoh.

Los puentes **no redirigen todo ROS 2 a Internet**. El canal está limitado
al destino TCP, dentro de la LAN. Nunca publiques 7447 en el router de Internet.

### D. Apagar solo el bridge (manteniendo la GUI y Gazebo)

```powershell
docker compose --profile pi stop zenoh
# GUI and observer remain running.
```

Para apagar todo: `docker compose --profile pi down`.

## Persistencia y registro de datos

- Código fuente: bind mount `../` -> `/cefiro_ws/src/PROJECT-CEFIRO`.
- Compilación: `../.docker-data/build`.
- Instalación: `../.docker-data/install`.
- Logs: `../.docker-data/log`.
- ROS bag/CSV: `../.docker-data/records`.

Ejemplo de grabación solo de telemetría:

```powershell
docker compose exec observer cefiro-run ros2 bag record `
  -o /cefiro_data/records/pi_imu_test /real/imu/data
```

Durante las grabaciones revisa espacio con `docker system df` y `docker stats`.

## Ubuntu nativo (opcional)

`compose.native-linux.yaml` permite un servidor X11 nativo, y
`compose.nvidia.yaml` añade GPU visible al contenedor. No utilices esta
configuración en Windows PowerShell. Pasar GPU/CUDA no garantiza OpenGL 3D.
Revisa el script `prepare-xauth.sh` y la documentación histórica de CEFIRO v3.

## Seguridad y límites

- noVNC está publicado **solo en localhost** (`127.0.0.1:6080`), sin contraseña.
- No se montan puertos USB ni dispositivos `/dev/ttyUSB*` por defecto.
- El dominio ROS de Windows (42) es distinto del de Pi (30): evita descubrir
  de forma nativa a la vez y por puente los mismos topics.
- El `zenoh` del contenedor se habilita solo con `--profile pi`.
- `ping` verifica conectividad ICMP, no conexión ROS 2 ni calidad de señal.

Referencias oficiales:
- https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds
- https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds/blob/main/DEFAULT_CONFIG.json5
- https://gazebosim.org/docs/harmonic/install_ubuntu/
- https://docs.docker.com/desktop/features/wsl/
