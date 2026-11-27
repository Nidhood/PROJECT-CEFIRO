# CEFIRO GUI 2.4.16 — Predictive Roll Lab

**Exclusivamente offline.** No configura, arma ni publica ganancias al STM32. No reemplaza el control de simulación Gazebo ni el firmware actual. Mantener las protecciones y no aplicar los candidatos a motores.

## Uso
1. Conservar el STM32 Power Lab, cefiro_msgs y cefiro_hardware_interface actuales. Reemplazar solamente `cefiro_gui`.
2. `colcon build --packages-select cefiro_gui --cmake-clean-cache` (con las dependencias ya instaladas y el entorno ROS 2 Jazzy cargado).
3. Abrir `CONTROL ARCHITECTURE` → `AUTO-TUNING LAB / OFFLINE`. Desmarcar la hipótesis de voltaje. Importar simultáneamente los dos CSV CW/CCW. Si hay rechazo, se muestran máximos de actuación aceptada por grupo y recuento de muestras por archivo. No cambiar los filtros ni la fuente de actuación para ocultar datos insuficientes.
4. Presionar `PREDICTIVE SWING-UP / LQR / TRACKING (OFFLINE)`. El botón anterior conserva su función original. Los resultados predictivos se muestran en el texto de estado y se guardan en `Documents/cefiro_data/autotuning/<UTC>_predictive/`.

## Resultados
- `predictive_candidates.csv`: 234 combinaciones, incluidos rechazos, captura, regulación y seguimiento, con prueba independiente comenzando arriba.
- `predictive_report.json`: procedencia, modelo y resumen; `physical_validated=false`, `deployable=false`.
- `nominal_hybrid_trace.csv`: secuencia nominal desde el equilibrio inferior.
- `nominal_upper_trace.csv`: ensayo nominal independiente desde el equilibrio superior.

Fases del CSV: 0 Swing-Up, 1 regulación, 2 seguimiento (0° +5° -5° 0°, expresados alrededor de la rama angular seleccionada). El modelo integra cada 5 ms con control simulado cada 20 ms. Los candidatos comparan un predictor no lineal con historial de torque, Swing-Up energético o quíntico, Riccati discreta y feedforward de gravedad. Captura exige estado medido y predicho en región durante 0.2 s; regulación exige 1.5 s continuos al final de 4 s; cada uno de los tres escalones debe terminar con 1 s continuo dentro de 3° y 10°/s. Estas tolerancias son criterios de simulación elegidos para cribado, no límites mecánicos certificados.

Prueba lineal suficiente en una rejilla finita: norma infinito de M^256 < 0.99, no certificado de robustez continua. Para datos HOST se evalúa un intervalo de retardo estimado −120/+40 ms (limitado a no negativos); para FMU −40/+40 ms. Incertidumbres lineales: gravedad ±10%, amortiguamiento ±20%, actuación ±10%. Escenarios no lineales: 3 retardos × 2 gravedades × 2 factores conjuntos de amortiguamiento/actuación × 2 perturbaciones iniciales. No se aumenta el torque más allá de los máximos observados y el techo del modelo anterior. No se modelan todas las dinámicas ni el retardo variable, el banco, la electrónica ni el mixer multivariable con detalle.

Con los CSV históricos el modelo usa HOST REQUEST y tiempo ROS, no DShot efectivo. El retardo identificado de 280 ms no es una medición del actuador local. El modelo es de Roll únicamente y no valida Pitch, Yaw, ni hover libre. El AHRS embebido debe estar válido antes del control físico. El reporte predictivo tiene un esquema distinto y no puede ser cargado por el importador de Shadow PD. No cambiar los interlocks de control automático.

## Pruebas
`g++ -O2 -std=c++20 -Wall -Wextra -Wpedantic -Werror=misleading-indentation -Iinclude test/test_predictive_roll.cpp -o /tmp/test_predictive_roll` desde este paquete. Ejecutar con dos CSV como argumentos para verificar el modelo histórico. También se ejecutaron `test_autotune` y las pruebas nativas de modelos, sensores de visualización y registro. Falta compilación integral Qt6/ROS 2 Jazzy y validación de hardware.
