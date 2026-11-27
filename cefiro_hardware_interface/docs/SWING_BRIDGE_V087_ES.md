> Nota histórica de v0.8.7: el rango manual de v0.8.8 se documenta en MANUAL_RANGE_2047_RELEASE.md.

# CEFIRO hardware interface 0.8.7 — Swing-Up bridge

Paquete completo basado en el ZIP hardware_interface(3) del operador. No requiere parchear GUI ni STM32. Requiere cefiro_msgs con IdentificationSample.msg (v0.2.0 de CEFIRO).

- Suscripción relativa `cefiro/control/swing_tuning`, namespace `real`, tipo Float64MultiArray. Cuatro valores: Kp, Kd, duración, signo de Roll, enviados como CMD_SWING_SHADOW. Array vacío: CMD_SWING_STOP. La ruta de ensayo motorizado conserva las comprobaciones de alimentación, calibración del FMU y lease; el firmware actual tiene CF_REAL_PROFILE_READY=0 y no autoriza POWERED. No modificar esas protecciones.
- Feedback `cefiro/control/swing_tuning_status` (ACK, SWG, SWR, SWP).
- IMU CF1 7/8/14 campos, IMX opcional anterior a IMU del mismo tick. IMU normal independiente de metadatos; ATT usa la política original de frescura. No fabrica orientación ni DShot. Si el firmware transmite 8 campos sin IMX, fmu_dshot_valid=false es esperado.
- Modo manual: límite host 300, coherente con el firmware actual; keepalive, watchdog y power gate conservados.

## Prueba Shadow (ESC sin alimentación, banco asegurado)

Detener instancias anteriores del hardware interface. Reemplazar exclusivamente src/cefiro_hardware_interface; respaldar fuera de src. Compilar desde ~/cefiro_ws con entorno Jazzy:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to cefiro_hardware_interface --cmake-clean-cache
source install/setup.bash
ros2 launch cefiro_hardware_interface hardware_wifi.launch.py enable_command_tx:=true rpm_to_dshot_enabled:=false allow_direct_dshot:=false dshot_max:=300
```

Si el enlace habitual es USB/USART, usar `hardware.launch.py port:=/dev/ttyUSB0 protocol_mode:=cf1` con los mismos parámetros. Si usa Wi-Fi LAN con IP fija, conservar sus argumentos de red habituales. No lanzar los dos transportes simultáneamente.

Desde otra terminal con el mismo overlay: `ros2 topic info /real/cefiro/control/swing_tuning -v` (1 publicador GUI y 1 suscriptor backend al abrir GUI), `ros2 topic hz /real/imu/data`, `ros2 topic echo /real/cefiro/control/swing_tuning_status`. Abrir GUI con `ros2 launch cefiro_gui cefiro_gui.launch.py use_sim_time:=false`. Seleccionar REAL, salida OFF, SHADOW. El FMU puede rechazar por IMU/AHRS o pose inválidos. No habilitar POWERED en esta etapa.

## Validación local
Pruebas nativas de codec CF1, manual keepalive, manual power gate, serial PTY, TCP, UDP discovery y LAN discovery: aprobadas. Test del callback y parser de producción con dobles C++: aprobado (`python3 tools/test_swing_imu_bridge.py`). La prueba fake-NetworkManager no se completó dentro del tiempo de ejecución. Compilación integral ROS 2 y prueba física pendientes; este entorno no dispone de ROS 2 Jazzy.
