# CEFIRO GUI 2.4.6 — Lecturas estables

Basado en 2.4.5. El cambio se limita a la presentación de las lecturas del control manual y el resumen de comandos. La telemetría, CSV, ROS bridge, configuración y lógica de actuación y seguridad no se alteran.

* Una única función `refreshMotorReadouts()` actualiza los cuatro motores desde el slider y el temporizador. No hay formatos alternados de una y varias líneas.
* Tres líneas reservadas por motor: Req RPM, DShot HOST observado y Eq RPM nominal. No se usa ajuste automático de línea; el ancho se reserva en función de la tipografía, no de los valores. Si falta telemetría, se muestra `--`.
* El indicador maestro y el resumen de comandos tienen altura reservada y no fuerzan el ancho del panel. El texto completo queda accesible mediante tooltip (incluido el motivo de inhibición, que también sigue en el estado existente).
* La reserva de espacio se recalcula únicamente al aplicar un tema o escala de UI. La distribución responsive se conserva.

## Instalación
Sustituir `cefiro_gui` y recompilar con `colcon build --symlink-install --packages-select cefiro_gui --cmake-clean-cache` tras cargar el entorno ROS y del workspace.

## Pruebas
Las comprobaciones estáticas y las pruebas C++ independientes de Qt se pueden ejecutar offline. Para verificar geometría real en un entorno con Qt6 Widgets instalado:

```bash
g++ -std=c++20 -Iinclude test/test_stable_readout_qt.cpp $(pkg-config --cflags --libs Qt6Widgets) -o /tmp/cefiro_readout_test
QT_QPA_PLATFORM=offscreen /tmp/cefiro_readout_test
```

El test Qt no se pudo ejecutar en el entorno de entrega porque no dispone de Qt6. Compilación ROS 2/Qt6 y prueba visual en hardware pendientes. Verificar con motores desarmados, cambiar sliders y variar la escala 100–200%; el panel no debe desplazarse cuando cambian las lecturas.
