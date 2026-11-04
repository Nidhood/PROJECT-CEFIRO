# CEFIRO — estabilidad del motor manual y diagnóstico de DENIED

Versión: hardware_interface 0.4.0. No cambia el firmware STM32 ni sus límites de seguridad.

## Causa probable del fallo anterior

El firmware CEFIRO_FMU_V3 desarma la salida manual si pasan más de 300 ms
sin recibir `CMD_DSHOT`. La GUI emite `/real/cefiro_actuators/target_rpm`
mediante un QTimer de Qt que intenta publicar cada 50 ms. En un retraso
intermitente de Qt / DDS, la Raspberry anterior dejaba de enviar DShot; la
STM32 desarmaba y el siguiente DShot recibía ACK `DENIED`. La causa exacta
requiere correlacionar el estado STM32 en el instante del rechazo. Otros
interlocks que provocan DENIED siguen intactos.

## Corrección

La Raspberry recibe las RPM nominales y conserva la última consigna válida
por **un periodo acotado** de 0,60 s. Su ciclo de ROS 2/USART de 10 ms
retransmite la consigna actual a la STM32 cada 40 ms.

No se retransmite si la GUI lleva más de 0,60 s sin enviar mensajes.
Un SAFE STOP / ESTOP, la pérdida de enlace, las fallas STM32 y el
watchdog de la GUI siguen deteniendo la salida. No hay rearmado automático.

## Raspberry — instalación del paquete C++ únicamente

1. Sustituir `~/cefiro_ws/src/cefiro_hardware_interface/` por la carpeta del ZIP
   (si tus paquetes están en otra subcarpeta bajo `src/`, respeta esa ubicación).
2. Con la terminal fuera de PlatformIO venv, ejecutar:

```bash
cd ~/cefiro_ws
source /opt/ros/jazzy/setup.bash
rm -rf build/cefiro_hardware_interface install/cefiro_hardware_interface
colcon build --symlink-install --packages-select cefiro_hardware_interface \
  --cmake-clean-cache \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

Verifica primero sin enviar comandos:

```bash
ros2 launch cefiro_hardware_interface hardware.launch.py \
  port:=/dev/ttyUSB1 protocol_mode:=cf1 enable_command_tx:=false
```

## Banco sin hélices

Con ESC y dron fijados y corte físico de alimentación al alcance,
verifica que las tramas IMU, STAT y ACK funcionan correctamente antes de
habilitar el control manual.

```bash
ros2 launch cefiro_hardware_interface hardware.launch.py \
  port:=/dev/ttyUSB1 protocol_mode:=cf1 \
  enable_command_tx:=true rpm_to_dshot_enabled:=true
```

En la ASUS:

```bash
ros2 launch cefiro_hardware_interface gui_real.launch.py
```

Modo MANUAL, todos los motores cero, Forward, Output enabled, START TEST.
No aumentar el mando hasta ver `armed_acknowledged:true`.

## Diagnóstico del nuevo status JSON

```bash
ros2 topic echo /real/cefiro/hardware/status --full-length --once
ros2 topic echo /real/cefiro/hardware/command_ack
```

- `gui_setpoints_received` debe aumentar ~20/s durante START TEST.
- `manual_keepalive_sent` debe aumentar ~25/s mientras ARM está confirmado.
- `gui_setpoint_age_s` debe permanecer cerca de 0–0.1 s cuando la GUI funciona.
- `last_dshot_tx_age_s` debe permanecer cerca de 0–0.04 s mientras manda DShot.
- `gui_timeout_stops` indica paradas porque se perdió la publicación de la GUI.
- `dshot_denials` cuenta rechazos de la STM32; revisa `mcu_status`
  y tensión real bajo carga si crece sin que aumente `gui_timeout_stops`.

**No elimines el watchdog del STM32 para forzar un giro continuo.**
Si desaparece la GUI o la red, se detiene como muy tarde tras la ventana
acotada del host más el timeout independiente de la FMU. Para cortar
motores de forma inmediata ante un peligro, utiliza el corte físico
de alimentación del ESC.
