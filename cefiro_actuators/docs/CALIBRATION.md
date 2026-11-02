# CEFIRO — plan de caracterización ESC + motor + hélice

**Objetivo:** sustituir las hipótesis de `config/actuators.yaml` con parámetros medidos, evitando identificar de manera falsa RPM como una función fija de DShot.

## 1. Confirmar la cadena de hardware

Documentar el ESC exacto y su firmware, DShot300 verificado o un protocolo alternativo, número de polos del motor, Kv de los cuatro motores, diámetro/paso/tipo de hélice CW y CCW, alimentación bajo carga y dirección de cada canal. La ficha del ESC B0DQQ1B7DM indica DShot, pero **no** demuestra por sí sola qué versión, dirección o telemetría opera en tu unidad. Los motores 1/3 son CW y 2/4 CCW según el CAD, pero verificarlos físicamente.

**Seguridad:** no ejecutar calibraciones de empuje con hélices descubiertas o con el dron sin inmovilizar. Usar soporte adecuado, pantalla y procedimiento de apagado físico. Las pruebas iniciales de DShot deben realizarse sin hélices.

## 2. Ensayos experimentales recomendados

1. **Sin hélice, sin carga:** barridos crecientes/decrecientes de DShot, midiendo RPM, corriente y Vbus. Determinar umbral de arranque, Kv efectivo, corriente en vacío y comportamiento de saturación.
2. **Con hélice y dinamómetro:** para varios valores de DShot, registrar RPM estable, empuje [N], par de reacción [N·m], corriente [A] y tensión [V]. Separar motores CW/CCW si difieren.
3. **Respuesta transitoria:** aplicar escalones alrededor de RPM de operación y registrar RPM a frecuencia suficiente, en aceleración y desaceleración. Ajustar `esc.time_constant_s`, `motor.rotor_prop_inertia_kg_m2` y pérdidas. No confundir latencia de DShot (~53 µs de trama) con tiempo de respuesta electromecánica (típicamente mucho mayor).
4. **Carga compartida:** repetir cuatro motores simultáneamente; identificar caída de tensión en distribución/ESC y `battery.internal_resistance_ohm`.
5. **Validación:** usar ensayos **diferentes** de los usados para ajustar, comparar RMSE de RPM, tiempo de subida y establecimiento, error de empuje y error de corriente. Registrar unidades y temperatura.

## 3. CT y CQ a partir de mediciones

Para hélice de diámetro D, velocidad en rad/s y densidad del aire rho:

```
F = [rho * Ct * D^4 / (4*pi^2)] * omega^2
Q = [rho * Cq * D^5 / (4*pi^2)] * omega^2
```

CSV de banco con cabecera `rpm,thrust_n,torque_nm`; `torque_nm` puede omitirse si no hay torquímetro. El ajuste de las constantes se realiza por regresión lineal por el origen:

```bash
python3 scripts/fit_propeller_coefficients.py bench.csv \
  --diameter-m 0.254 --air-density 1.225
```

El valor de diámetro del comando debe coincidir con la hélice **real**. Esto no sirve para modelar por sí solo la respuesta del ESC.

## 4. Parámetros todavía no identificados

`R` y `L` equivalentes, `J` conjunto rotor-hélice, Kv efectivo, Ct, Cq, umbral de arranque, dinámica de conmutación del ESC, corriente máxima real, retardo de comandos, capacidad de frenado activo, impedancia de alimentación y temperatura. Un dato de potencia comercial no basta para identificarlos.

## 5. Elección de modelo futuro

Este paquete implementa un modelo físico continuo con integración numérica y límites. La caracterización previa del Propeller Arm contiene un modelo discreto de velocidad con retardo y coeficientes ARX. Ese modelo puede ser útil como **modelo alternativo** después de volver a identificarlo con entradas DShot, RPM medidas y la hélice final de CEFIRO; copiar sus parámetros de PWM 1000–2000 µs no sería físicamente consistente.
