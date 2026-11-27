# CEFIRO GUI 2.4.9 — Observador integrado

Base: `cefiro_gui(4).zip` (2.4.8), suministrado por el usuario. No se cambia ni se genera ningún proyecto STM32, hardware interface ni nodo adicional. El control de actitud sigue siendo responsabilidad del firmware. El observador es estrictamente de lectura y se ejecuta en el proceso `cefiro_gui_node` existente, en el callback de la IMU real.

## Uso
1. Con los ESC sin alimentación, arranque el hardware interface existente y la GUI habitual.
2. Seleccione REAL HARDWARE y compruebe la orientación AHRS válida.
3. Con la salida desactivada, abra SWING-UP TUNING. La sección ROLL TORQUE OBSERVER / GUI DIAGNOSTIC se actualiza automáticamente, sin pulsar START SELECTED TRIAL.
4. Para adquirir datos, utilice el Data Recorder existente (adquisición continua, tasa completa, antes de abrir el diálogo de tuning si quiere registrar un ensayo). Se anexan cinco columnas, sin reordenar ninguna anterior.

El botón START SELECTED TRIAL sigue enviando exactamente los comandos existentes del STM32. El firmware anterior aún puede terminar Shadow con `TORQUE_SATURATION`. El observador no cambia esa decisión ni habilita POWERED. El mensaje de parada se muestra por separado de la calidad del observador. No se cambia ningún interlock de IMU, potencia, DShot, perfil, lease ni parada. No se requiere una fuente de batería para el cálculo de diagnóstico, pero el perfil de actuación motorizada sigue pendiente de calibración y supervisión de la fuente de potencia real. No se habilita ninguna actuación nueva.

## Modelo
`tau_ext = J * dp/dt - kg * sin(roll - phi0) + b * p`, con p en rad/s, ángulos en radianes, J=0.03378 kg m², kg=0.820915 N m, b=0.020867 N m s/rad y phi0=6.5 deg. Coeficientes provisionales de los ensayos previos, no calibración validada. `p` es la velocidad del giroscopio en el eje x, aproximación de la derivada de roll para el modelo restringido; se rechazan muestras con |pitch| >15 deg o |q|,|r| >30 deg/s. Este umbral limita la validez del diagnóstico, no la actuación. La derivada se filtra con constante de tiempo 0.12 s. Se requieren tres muestras consecutivas válidas para la primera estimación; se reinicia al detectar discontinuidades de tiempo (intervalo fuera de 1–150 ms), AHRS inválido, datos no finitos o pérdida de muestras >250 ms. La fuente de tiempo es el timestamp ROS de la IMU primaria (en el hardware interface actual suele ser recepción del host; puede contener jitter). El resultado incluye fuerzas externas y error del modelo; no mide directamente el torque de los motores ni permite inferir su capacidad máxima.

## CSV
- `roll_torque_estimate_valid`: 1 o 0.
- `roll_torque_gravity_Nm`, `roll_torque_damping_Nm`, `roll_torque_external_est_Nm`, `roll_torque_dp_rad_s2`: estimaciones en unidades SI, en blanco cuando no válidas, incluida simulación. Si la identificación sincronizada invalida o contradice la orientación usada por el observador, la estimación de esa fila también se invalida. No modifica ninguna columna anterior. Los metadatos YAML incluyen los coeficientes y los límites de validez.

## Validación
Pruebas C++20 nativas: observador (equilibrio, gravedad, derivada, amortiguamiento, wrap angular, desconexión temporal, AHRS inválido, validez del modelo), modelos, registro, motor observation, parámetros, respuesta al escalón y escala. No se dispone de ROS 2 Jazzy/Qt6 en el entorno de construcción del ZIP; compilación completa y prueba de GUI pendientes. Los dos checks estáticos obsoletos de layout se conservan sin alterar. No hay cambios de firmware ni hardware interface.
