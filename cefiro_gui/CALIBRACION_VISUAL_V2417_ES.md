# CEFIRO GUI 2.4.17 — Calibración visual de acelerómetro

Basada en 2.4.16 Predictive Roll. Solo se modifica el asistente de calibración, se añade su máquina de estados y prueba independiente, y se actualiza la versión. No modifica el firmware, los controladores, la comunicación, el formato CSV ni la fusión IMU.

## Uso

1. Retirar hélices y desconectar físicamente la alimentación ESC. Mantener STM32/IMU alimentados. Asegurar el conjunto; no forzar el gimbal. Desactivar salida y grabación, finalizar cualquier ensayo.
2. Seleccionar REAL. Abrir CONTROL ARCHITECTURE / SWING-UP TUNING / CALIBRATE EXTERNAL ACCEL / 6 FACES. Confirmar la advertencia.
3. INICIAR / REINICIAR. Seguir el modelo 3D (punto amarillo = nariz). El asistente muestra la postura esperada, el eje detectado y la norma del acelerómetro. El orden es normal (+Z), invertido (-Z), nariz arriba (+X), nariz abajo (-X), derecha abajo (+Y), izquierda abajo (-Y). Convención asumida: +X nariz, +Y izquierda, +Z arriba, tal como el modelo de la GUI. Si el montaje del sensor difiere, verificar el marco físico; el asistente no cambia ni deduce el montaje de los ejes.
4. Colocar cada cara con precisión en un soporte nivelado. Rojo = postura incorrecta/movimiento; amarillo = acumulando muestras; verde = estable y captura automática. Se requieren al menos 60 muestras distintas y 1,8 s, más 0,6 s estables en verde (a 25 Hz, aproximadamente 3 s por cara). Los cortes de datos o movimientos reinician solo la ventana activa. PAUSAR conserva las caras terminadas; INICIAR / REINICIAR borra la sesión.
5. Solo si el ajuste de seis caras cumple las tolerancias se habilita EXPORT STM32 CALIBRATION HEADER. Guardar como cf_accel_calibration_values.h dentro de STM32/MDK-ARM/Core/Inc. Conservar el resto del proyecto.
6. Recompilar y cargar STM32 una vez con ESC desconectados. Reiniciar inmóvil y verificar STAT (calibrated=1, ready=1) y orientación ROS. La calibración no garantiza que el perfil automático esté validado ni habilitado.

El asistente exige IMU externa I2C (STAT campo 18 = 2), estado FMU inactivo, enlace/IMU recientes, salida OFF y ausencia de armado o ensayo activo. Usa aceleraciones y giros crudos publicados por ROS, no la orientación GUI/AHRS. No exige tensión de batería. No publica comandos. No modifica la lógica original de ajuste ni sus tolerancias. Un ajuste fuera de tolerancia no se exporta; repetir las seis caras con un montaje preciso.

Pruebas: máquina de estados C++20 con muestras sintéticas, rechazo de ejes equivocados, ruido, tiempos duplicados/discontinuos, interrupción y ajuste fuera de tolerancia. Compilación integral Qt6/ROS 2 y validación física pendientes.
