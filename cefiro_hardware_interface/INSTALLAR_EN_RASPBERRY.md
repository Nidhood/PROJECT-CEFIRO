# CEFIRO hardware_interface v0.4.1 — instalación en Raspberry Pi

Este paquete corrige la **discrepancia entre el binario v0.3.x y hardware.yaml v0.4.x** que provocaba:

```
Fatal error: Require valid open-loop DShot map and command_timeout_s in (0.05, 0.30)
```

La versión 0.4.1 acepta `command_timeout_s: 0.60` y `manual_keepalive_period_s: 0.040`. El firmware STM32 conserva sus watchdogs, límites DShot y condiciones de seguridad; este ZIP **solo actualiza el nodo ROS 2**. No cambia el giro de los ESC.

## Instalación

Cierra antes `ros2 launch` y cualquier aplicación que tenga abierto el puerto. Copia el ZIP a la Raspberry y descomprímelo en el `src` del workspace:

```bash
cd ~/cefiro_ws
unzip -o ~/Downloads/CEFIRO_HARDWARE_INTERFACE_v0_4_1_INSTALL_VERIFIED.zip -d src/
bash src/cefiro_hardware_interface/tools/install_and_verify.sh
```

Si colocaste el ZIP en otra carpeta, cambia la ruta. El script elimina **solo** `build/cefiro_hardware_interface` y `install/cefiro_hardware_interface`, recompila el paquete junto a las dependencias necesarias y comprueba la versión real del ejecutable.

En una terminal NUEVA:

```bash
source /opt/ros/jazzy/setup.bash
source ~/cefiro_ws/install/setup.bash
ros2 run cefiro_hardware_interface hardware_node --version
```

Debe responder `cefiro_hardware_interface 0.4.1`.

## Prueba inicial sin transmisión hacia motores

Con hélices retiradas, inicia en modo de solo lectura:

```bash
ros2 launch cefiro_hardware_interface hardware.launch.py \
  port:=/dev/ttyUSB1 protocol_mode:=cf1 \
  enable_command_tx:=false rpm_to_dshot_enabled:=false
```

En otra terminal:

```bash
source ~/cefiro_ws/install/setup.bash
ros2 topic echo /real/cefiro/hardware/status --full-length --once
```

Busca `hardware_interface_version: 0.4.1` en el JSON del campo `data` y `rx_valid_frames` creciente.

Solo después de confirmar comunicación, se podrá habilitar TX de manera supervisada para pruebas de banco con las hélices retiradas y el dron sujeto.

## Por qué `cat /dev/ttyUSB1` puede mostrar caracteres extraños

`cat` no configura la velocidad del puerto. El monitor serial puede configurarlo automáticamente a 115200 8N1. Para leer en Linux sin el nodo ROS abierto:

```bash
stty -F /dev/ttyUSB1 115200 cs8 -parenb -cstopb -crtscts \
  -ixon -ixoff raw -echo min 1 time 0
timeout 5s cat /dev/ttyUSB1
```

No abras `cat`, Serial Monitor ni otra herramienta mientras esté abierto `hardware_node`.
