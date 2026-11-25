# Hardware interface 0.8.5 — diagnóstico y protección de modo manual

El log `CMD_DSHOT ... BENCH_ZERO_ONLY` significa que el firmware STM32 permite únicamente DShot=0 en su configuración de banco. El ARM ACK no autoriza un movimiento en ese modo. No es un timeout de GUI. No se anula este rechazo en ROS.

- Se decodifica BAT flags (bit 4) y PWR campo 9 (índice desde cero) para reconocer el modo de banco, junto con validez de tensión y frescura del ADC.
- El nodo exige BAT/PWR recientes y válidos antes de armar para movimiento; el STM32 sigue siendo la autoridad sobre batería crítica, límites, watchdog, etc.
- `firmware_bench_zero_only`, `manual_power_ready` y `manual_power_inhibit_reason` son campos nuevos en hardware/status. Un rechazo conserva su causa en `last_stop_reason` aun si llegan otros ACK de STOP/MODE.
- La detección por ACK BENCH_ZERO_ONLY queda enclavada durante la sesión; reiniciar el nodo después de comprobar una configuración normal del STM32.
- Los watchdogs de 600 ms (GUI) y 300 ms (STM32) y la selección de transporte se conservan. No se rearma automáticamente.

## STM32
El archivo original V0.8.7 `STM32/MDK-ARM/Core/Inc/cf_config.h` ya define CF_BENCH_ARM_WITHOUT_VBAT=0U por defecto (si no existe una definición externa). Verificar las macros del target, limpiar, recompilar y programar el target normal. Verificar alimentación real y telemetría BAT/PWR con motores desconectados. No eliminar la protección de cero en el modo de banco ni simular una tensión válida. PWR contiene 12 campos y su campo 10 (contando desde 1) debe ser 0 para el modo normal. Si sigue en 1, comprobar el binario cargado. No es necesario actualizar el ESP32.

## Build
Salir del venv de PlatformIO; source /opt/ros/jazzy/setup.bash; desde ~/cefiro_ws ejecutar:
```
colcon build --symlink-install --packages-select cefiro_hardware_interface --cmake-clean-cache --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```
Primero usar launch con enable_command_tx:=false. Verificar `ros2 topic echo /real/cefiro/hardware/status --full-length`. En modo normal y con alimentación válida, `firmware_bench_zero_only=false`, `manual_power_ready=true`. El firmware puede imponer restricciones adicionales. Solo tras verificar el banco sin hélices se puede habilitar la transmisión de comandos con el launch habitual; dshot_max admite hasta 2047 y no modifica la protección del STM32.

## Validación
PASS: gate C++20, keepalive, CF1, PTY serial y TCP (pruebas locales). Compilación ROS y prueba física pendientes. No se modificó el firmware.
