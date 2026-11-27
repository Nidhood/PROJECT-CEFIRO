# CEFIRO GUI 2.4.12 — fusión de actitud para diagnóstico

Base: GUI 2.4.11 (AHRS original). No requiere cambiar STM32 ni hardware interface.

- Cuando llega una orientación válida del STM32, se conserva el flujo original y se grafica `REAL`.
- Si no llega una orientación válida, el estimador de visualización usa aceleración y giroscopio del tópico IMU original. Arranque inmóvil durante al menos 1,2 s de muestras coherentes. Filtra aceleración (constante 0,22 s), estima sesgo inicial del giroscopio e integra un cuaternión con corrección gravitacional lenta. Las curvas `GUI FUSION / DIAG` y `GUI FUSION / REL YAW` y el visor 3D se identifican como diagnóstico.
- El yaw del respaldo empieza en cero y puede derivar. No se promete rumbo absoluto. La fusión no calibra ni corrige físicamente el acelerómetro. En maniobras con aceleración lineal, el estimador puede presentar errores.
- No modifica `orientation_valid`, orientación AHRS, la estimación de torque, los datos CSV originales, la autorización de actuación, ganancias, comandos, ni el protocolo CF1. El observador de torque y el control siguen requiriendo orientación válida del STM32. No habilita el modo POWERED.
- Se invalida ante IMU no válida, huecos de tiempo, retroceso del timestamp o velocidad fuera de rango.

## Compilación
Reemplazar `src/cefiro_gui`, ejecutar `colcon build --packages-select cefiro_gui --cmake-clean-cache`, cargar `install/setup.bash` y lanzar `ros2 launch cefiro_gui cefiro_gui.launch.py use_sim_time:=false`.

## Validación offline
8 pruebas C++ puras aprobadas; también comprobaciones de paquete, tipografía y etiquetas estables. Prueba sintética de ruido con la aceleración y velocidad estacionarias compartidas por el usuario: RMS roll directo 0,429 grados frente a 0,060 grados filtrado. No es medición experimental del hardware. No se dispone de ROS 2 Jazzy/Qt6 en este entorno; compilación integral y ensayo real pendientes.
