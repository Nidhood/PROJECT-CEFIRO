# CEFIRO GUI 2.4.33 — V15 build fix

El panel Swing-up Tuning no define `hover`. Su prefijo de grabación debe ser fijo. El método `startEmbeddedRoll()` mantiene su selector de prefijo para ON HOVER y SWING-UP. No se cambia el control ni el protocolo.

Los archivos del ZIP tienen una fecha fija anterior a la fecha de entrega para evitar marcas de tiempo futuras al extraer en equipos con diferencias de zona horaria.

Reemplazar solo `cefiro_gui` de V15. Compilar con `colcon build --packages-up-to cefiro_gui --cmake-clean-cache --cmake-clean-first`. Verificación estática incluida; compilación Qt6/ROS 2 no disponible en este entorno.
