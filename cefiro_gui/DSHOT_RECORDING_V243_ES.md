# CEFIRO GUI 2.4.3 — DShot y RPM nominales

Reemplaza la carpeta `cefiro_gui` completa, no es un parche. Requiere `cefiro_msgs` 0.2.0 con `IdentificationSample.msg` (sin cambios respecto a 2.4.2).

## Monitor y adquisición

- La IMU real conserva su suscripción `/real/imu/data`; Gazebo conserva `/imu/data`.
- El tópico `/real/cefiro/hardware/commanded_dshot` alimenta las curvas HOST M1–M4 y el CSV. Es un comando solicitado por el host, no un ACK ni RPM medida. QoS best effort compatible con publicadores reliable y best effort.
- Si falta el tópico dedicado, se usa `last_commanded_dshot` del JSON `/real/cefiro/hardware/status` cuando el transporte está conectado y `last_dshot_tx_age_s` es válido (máximo 300 ms). No se asigna cero a datos ausentes.
- `/real/cefiro/hardware/identification_sample` alimenta las curvas FMU M1–M4 incluso sin grabación activa. El CSV conserva DShot FMU solamente si coincide el timestamp exacto de la IMU. Los comandos HOST independientes se asocian causalmente por el último comando recibido antes de la muestra de IMU, con límite de edad de 300 ms.
- El nuevo panel `RPM / DSHOT EQUIVALENT (NOMINAL)` muestra la inversa del mapeo configurado por el hardware interface. Los gráficos de referencia y velocidad medida se mantienen. Las etiquetas de control manual también muestran DShot del host y RPM equivalentes.
- La inversa es `rpm_deadband + (rpm_full_scale - rpm_deadband)*(DShot-dshot_min)/(dshot_max-dshot_min)` para DShot no cero dentro del rango. Para DShot cero, el equivalente es cero. DShot 48 corresponde al límite inferior nominal, no a una velocidad física garantizada. Se usa `configured_dshot_min`, `configured_dshot_max`, `nominal_rpm_full_scale` del estado JSON; si este no incluye `rpm_deadband`, se usa el parámetro GUI `nominal_rpm_deadband` (25.0 por defecto; debe coincidir con el hardware interface). Si el mapa es inválido o está desactualizado, no hay cálculo. Las RPM equivalentes no son RPM medidas, ni una caracterización de empuje.
- El CSV agrega `host_dshot_source`, `rpm_map_valid`, los cuatro parámetros de mapeo, `rpm_eq_host_m1..m4`, `rpm_eq_fmu_m1..m4`; conserva todas las columnas previas. El Data Recorder muestra disponibilidad de HOST, FMU y mapa, y conteos de filas válidas. La antigüedad de HOST en el tópico sin header representa tiempo desde recepción por la GUI, no latencia de transmisión física.
- Los límites físicos, comandos de actuación y firmware no se modifican. El precheck GUI del ensayo motorizado sigue exigiendo una muestra válida de identificación FMU reciente (ahora se consulta directamente el mensaje validado), además de IMU, grabación y demás condiciones existentes.

## Instalación

```
cd ~/cefiro_ws
source /opt/ros/jazzy/setup.bash
rm -rf build/cefiro_gui install/cefiro_gui
colcon build --symlink-install --packages-up-to cefiro_gui --cmake-clean-cache
source install/setup.bash
ros2 launch cefiro_gui cefiro_gui.launch.py use_sim_time:=false
```

## Diagnóstico (motores desarmados)

```
ros2 topic info /real/cefiro/hardware/commanded_dshot -v
ros2 topic echo /real/cefiro/hardware/status --once
ros2 topic echo /real/cefiro/hardware/identification_sample --once
```

El tópico de comandos puede no emitir cuando no hay una orden activa. Si ninguno de los canales proporciona DShot válido, la GUI no puede inferir el comando real. No iniciar un ensayo motorizado para corregir un fallo de adquisición. Para verificar cambios de comando, utilizar las pruebas de banco supervisadas sin hélices y con corte físico independiente.

## Validación

Cuatro ejecutables unitarios C++ (modelos, respuesta al escalón, unión IMU/identificación y observaciones DShot/RPM) aprobados con GCC C++20 y -Werror. Comprobaciones estáticas de paquete, temas, enrutamiento motor, referencias y gráficos aprobadas. Dos comprobaciones antiguas de diseño ya no corresponden a la versión original (v2.3 y layout); no se afirma compilación integral ROS 2/Qt6 ni validación física, porque este entorno no dispone de esas dependencias.
