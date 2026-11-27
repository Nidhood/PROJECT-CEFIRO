# CEFIRO GUI 2.4.4 — Responsive typography

Base: paquete completo `cefiro_gui` v2.4.3 DShot/RPM. Este cambio es visual;
no modifica el protocolo CF1, los tópicos, el mapeo de motores, el CSV, las
referencias, las ganancias, los límites de seguridad ni el firmware.

## Uso

En **System → DISPLAY / UI SCALE** selecciona `AUTO (window size)` o un
porcentaje fijo (100, 120, 140, 160, 180, 200%). `Ctrl+0` restaura AUTO
cuando la ventana principal tiene el foco. AUTO es el valor inicial en
cada ejecución. Ajusta el tamaño antes de iniciar un ensayo físico.

La escala automática se calcula sobre las dimensiones lógicas de la
ventana principal (no los píxeles físicos del monitor), con base de
1280×740, cuantización de 10% y rango 100–200%. Qt se encarga del DPI
del monitor; no se vuelve a multiplicar por el device pixel ratio.

Ejemplos para ventanas lógicas de tamaño completo (el tamaño útil real
puede ser menor por los paneles y la decoración del escritorio):

| Tamaño | Escala AUTO | Texto base | Título de panel | Título de página |
|---|---:|---:|---:|---:|
| 1280×740 | 100% | 12 px | 14 px | 24 px |
| 1530×910 | 120% | 14 px | 17 px | 29 px |
| 1920×1080 | 150% | 18 px | 21 px | 36 px |
| 2560×1440 | 190% | 23 px | 27 px | 46 px |
| 3840×2160 | 200% | 24 px | 28 px | 48 px |

El escalado afecta los seis temas QSS, las etiquetas y valores,
botones, cuadros de edición, casillas, sliders, pestañas, tablas,
ejes/leyendas y texto de las gráficas, el visor 3D y el panel circular.
La cuadrícula usa el ancho disponible dividido por la escala, de manera
que una tipografía grande no obligue a mantener demasiadas columnas.
El grabador adapta sus columnas y estadísticas al ancho de su propio
diálogo; hereda la escala de la ventana principal. El diálogo de Swing-Up
usa un área desplazable y conserva los botones de acción fuera de ella.
La barra nativa del título de ventana depende del sistema operativo.
En ventanas muy estrechas y a escala manual alta puede ser necesario
usar scroll o reducir el porcentaje con Ctrl+0.

## Instalación

Sustituir solo la carpeta `cefiro_gui` en `~/cefiro_ws/src` (sin dejar
una copia de otro paquete del mismo nombre dentro de `src`). Mantener
`cefiro_msgs` 0.2.0 con `IdentificationSample.msg` y el hardware interface
compatible usados en v2.4.3.

```bash
cd ~/cefiro_ws
source /opt/ros/jazzy/setup.bash
rm -rf build/cefiro_gui install/cefiro_gui
colcon build --symlink-install --packages-up-to cefiro_gui --cmake-clean-cache
source install/setup.bash
```

## Verificación

- Cinco pruebas C++ independientes de Qt/ROS: modelos, escalón, motor
  observation, recording join y escala UI: PASS.
- Comprobación estática de tipografía en seis temas y su integración: PASS.
- Ocho comprobaciones estáticas heredadas: PASS. Dos scripts antiguos
  conservan expectativas obsoletas de versión 2.3 y un umbral de
  reflow de 980 px; fallan también en la versión 2.4.3 sin cambios.
- No se realizó compilación integral ni prueba visual interactiva en Qt6:
  este entorno no incluye Qt6 Charts/Widgets ni ROS 2 Jazzy.

Comprobar en el equipo con los motores desarmados: maximizar/restaurar,
cambiar de tema, cambiar de escala, abrir el grabador y el panel Swing-Up,
comprobar que la IMU, DShot y las RPM siguen mostrándose y registrándose.
