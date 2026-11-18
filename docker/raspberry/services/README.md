# CEFIRO — Servicios Raspberry Pi (systemd)

Requiere Ubuntu con ROS 2 Jazzy, `~/cefiro_ws/install/setup.bash`, interfaz CEFIRO compilada, `zenoh-bridge-ros2dds` instalado y `~/zenoh-pi.json5` funcionando (v9 bidireccional, dominio 42). No cambia firmware, Docker, configuración Zenoh ni el código ROS.

## Instalación

1. Fusionar la carpeta `docker/` del ZIP con el repositorio, o copiar `services/` a la Raspberry (ejemplo desde PowerShell raíz del repo):

   ```powershell
   scp -r .\docker\raspberry\services nidhood@192.168.68.59:~/cefiro-services
   ```

2. En la Pi, quitar hélices, desconectar alimentación de ESC y detener las dos terminales que estaban ejecutando manualmente los nodos (Ctrl+C). Conectar el adaptador CEFIRO una vez para identificarlo. Revisar `ls -l /dev/serial/by-id/ /dev/serial/by-path/`.
3. Instalar con el usuario normal (no `sudo bash`):

   ```bash
   cd ~/cefiro-services
   bash install-services.sh --device /dev/ttyUSB0
   ```

   Si es el único adaptador conectado, basta `bash install-services.sh`. Si aparece como ttyUSB1, usar ese número en la instalación. El instalador conserva la ruta persistente y no el número USB. Si no hay número de serie, usa `by-path` (mantener el mismo puerto físico). Si ya conoces la ruta persistente, `--serial-path /dev/serial/by-id/…` permite instalar incluso desconectado. Si encuentra varios dispositivos sin selección explícita, se detiene; no elige uno al azar.

   Default: TX habilitado, DShot máximo 300 (conservador). Para reproducir el límite original de 2047, solo después de validar el banco y mantener protecciones FMU, añade `--dshot-max 2047`. Para diagnóstico sin comandos físicos añade `--telemetry-only`. Ninguna variante publica ARM automáticamente. `--no-start` instala y habilita al próximo arranque sin reiniciar servicios ahora. `--workspace`, `--domain` y `--zenoh-config` permiten personalizar; el dominio debe coincidir con la configuración del puente Pi y con los nodos locales.

## Operación

```bash
systemctl status cefiro-zenoh cefiro-hardware --no-pager
journalctl -u cefiro-hardware -f
journalctl -u cefiro-zenoh -f
sudo systemctl restart cefiro-zenoh cefiro-hardware
sudo systemctl stop cefiro-hardware cefiro-zenoh
sudo systemctl start cefiro-zenoh cefiro-hardware
sudo systemctl disable --now cefiro-hardware cefiro-zenoh
```

`cefiro-hardware` aparece activo aun esperando el USB; `journalctl` distingue espera y detección. Después de conectar, el servicio lanza ROS. Si se desconecta durante operación, el nodo v0.5.0 gestiona la reconexión serial y no restaura el armado. Verificarlo físicamente antes de alimentar motores. El servicio reinicia ROS si el proceso sale. Zenoh usa network-online y reintenta si falla; el endpoint de escucha de `~/zenoh-pi.json5` debe corresponder a la IP actual de la Pi. No inicia otra instancia manual de los servicios. El puente v9 no tiene autenticación/TLS: usar solo red privada aislada de confianza; conservar el watchdog y el paro físico.

## Ajuste y desinstalación

Configuración: `/etc/cefiro/pi-services.env`. Unidades: `/etc/systemd/system/cefiro-{hardware,zenoh}.service`. Ejecutables: `/usr/local/lib/cefiro/`. Para modificar valores, reinstalar con los argumentos deseados o editar el archivo de entorno con `sudoedit`, luego reiniciar los servicios.

```bash
sudo systemctl disable --now cefiro-hardware cefiro-zenoh
sudo rm -f /etc/systemd/system/cefiro-hardware.service /etc/systemd/system/cefiro-zenoh.service
sudo rm -f /etc/cefiro/pi-services.env /usr/local/lib/cefiro/run-hardware.sh /usr/local/lib/cefiro/run-zenoh.sh
sudo systemctl daemon-reload
```

Esto no borra ROS, Zenoh, tu workspace ni los datos. Pruebas estáticas únicamente; falta validar físicamente systemd, udev y USB en la Pi.
