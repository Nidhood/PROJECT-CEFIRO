# CEFIRO GUI 2.4.1 — paquete completo

Este directorio sustituye a `cefiro_gui` (base: archivo cefiro_gui(2).zip del usuario). No es un parche. Conserva las seis plantillas Qt, gráficos, controles manuales y automáticos, y agrega Swing-Up Tuning, Safety Reset SIM y el CSV de identificación.

## Dependencias
Usar con STM32 RealPilot DShotRecording, hardware interface actualizado para publicar `/real/cefiro/hardware/identification_sample` y `cefiro_msgs` con `IdentificationSample.msg`. El ZIP de dependencias contiene `cefiro_msgs` completo. Si el hardware interface y cefiro_control ya tienen la actualización RealPilot DShotRecording, no se modifica nada más. Si siguen en main o Safety Pilot, usar el instalador auxiliar del ZIP de dependencias, solo para esos dos paquetes. No ejecutar el instalador antiguo sobre esta GUI.

Compilar desde el workspace con `colcon build --symlink-install --packages-up-to cefiro_gui cefiro_hardware_interface cefiro_control` y cargar `install/setup.bash`. ROS Jazzy y Qt6 requeridos. No se ha compilado la aplicación completa en este entorno.

## Registro
Seleccionar REAL HARDWARE, DATA RECORDER, Continuous, Start delay 0, Save interval 0; conservar attitude y angular rates. Las entradas de los cuatro motores, timestamp, estados, voltaje y cuaternión se añaden siempre. `fmu_dshot_valid=1` indica un snapshot recibido y validado del FMU, no RPM ni verificación del pulso físico. `host_dshot_valid=1` indica comando host reciente; no prueba entrega. Valores no disponibles se guardan en blanco. `fmu_sample_ms` es uptime del STM32, no reloj de ROS. `host_dshot_age_s` es edad del comando. La GUI no inventa valores para SIM. Cambiar SIM/REAL detiene la adquisición. Si se usa el hardware interface antiguo, el parámetro `real_identification_input: false` restaura la IMU estándar, pero no proporciona DShot FMU. Verificar primero sin hélices: `ros2 topic echo /real/cefiro/hardware/identification_sample --once`.

## Seguridad
Se conserva el bloqueo de actuación real hasta contar con un perfil de actuadores medido y validado. No sustituye paro físico independiente ni contención del banco. El ensayo motorizado piloto es Roll, no 3 GDL. La GUI requiere grabación continua válida y corta el envío de la autorización temporal cuando deja de grabar; el FMU conserva sus límites y watchdog. No se certifica estabilidad física ni tiempo de parada mecánica.

## Validación
Pruebas C++ independientes de ROS/Qt (modelos, respuesta al escalón), comprobaciones estáticas y prueba de columnas CSV. Dos pruebas estáticas antiguas (layout v1.3 y responsive Tron Ares) ya fallaban en el ZIP original por expectativas de la versión anterior; no se alteraron sus aserciones. Compilación ROS/Qt y ensayos físicos pendientes.
