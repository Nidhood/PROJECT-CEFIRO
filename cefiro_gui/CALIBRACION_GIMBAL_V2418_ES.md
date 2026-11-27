# CEFIRO GUI v2.4.18 — Calibración en gimbal

Solo cambia el asistente de calibración y su versión. No envía comandos a los motores ni cambia firmware, AHRS, DShot, control, recorder o protocolo.

Con las hélices retiradas, ESC físicamente desconectados y soporte asegurado, seleccionar REAL, abrir SWING-UP TUNING / CALIBRATE EXTERNAL ACCEL / GIMBAL y pulsar INICIAR. Mover lentamente (hasta 12 grados/s) por distintas inclinaciones y diagonales, sin forzar el gimbal. No es obligatorio detenerlo ni nivelar exactamente. Las tarjetas muestran la cobertura de seis sectores, no pasos obligatorios. Verde indica dirección registrada. El modelo 3D sugiere el sector con menos cobertura. Se aceptan ventanas cortas con baja variación de norma; la sesión conserva direcciones durante pausas y rechazos.

Se ajustan offsets y escalas diagonales usando la norma de gravedad y un conjunto de al menos 14 de 26 direcciones, con cobertura de ambos signos en cada eje y rechazo de datos mal condicionados. Validación leave-one-direction-out: RMS <= 0,20 m/s², máximo <= 0,45 m/s², residual de entrenamiento <= 0,35 m/s². Estos criterios son de consistencia de magnitud, no una validación independiente de orientación o de sesgo de gyro. La calibración presupone que el acelerómetro publicado no lleva ya una corrección previa activa. Si la geometría limita la cobertura, no se exporta: no forzar el soporte.

Exportar cf_accel_calibration_values.h a STM32/MDK-ARM/Core/Inc y recompilar/programar una vez. Verificar AHRS inicializado y calibrado después del reinicio. No garantiza habilitación del Swing-Up físico.

Pruebas nativas sintéticas; compilación integral ROS 2/Qt y validación física pendientes.
