# GUI 2.3.4 — estado real de armado

Paquete Qt completo, basado en la GUI adjunta. Incluye el ajuste de supervisión V0.8.6, publicación periódica ligera a 20 Hz y armado explícito con referencias iniciales de cero. No se rearma automáticamente. En REAL/MANUAL se requiere un hardware interface 0.8.5 con BAT/PWR válidos y recientes y modo normal de firmware. El mensaje BENCH_ZERO_ONLY explica que el STM32 admite solo cero; la GUI no intenta saltarse ese modo. No cambia SIM ni las ganancias ni los sensores. El estado ARM se confirma por ACK, no por el botón. El motivo de inhibición o stop se muestra en la interfaz.

Reemplazar cefiro_gui en el workspace del PC y compilar con ROS Jazzy/Qt6:
```
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select cefiro_gui --cmake-clean-cache --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

Pruebas estáticas existentes: 8/10 pasan. Las dos restantes (check_layout_v1_3.py y check_responsive_tron_ares.py) fallan también en el ZIP original por expectativas antiguas de título/layout; no se cambió ese diseño. Compilación Qt/ROS y pruebas físicas pendientes.
