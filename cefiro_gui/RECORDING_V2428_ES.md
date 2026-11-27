# CEFIRO GUI 2.4.28 — adquisición completa y duración configurable

## Uso

1. Seleccione SIMULATION o REAL HARDWARE y abra **DATA RECORDER**.
2. Escoja carpeta y prefijo. Todos los grupos de columnas están marcados inicialmente.
3. Para captura limitada seleccione **Time window**, escriba **Duration** y elija la unidad **ms / s / min / h**. El mismo selector afecta el retardo inicial. Al cambiar unidades se convierte el valor anterior (redondeo según precisión del campo). Máximo 24 h por ventana; modo continuo sin límite fijo. El modo continuo ignora un retardo oculto de una configuración anterior.
4. Mantenga **Save every received IMU sample (recommended)** marcado. No hay decimación de la GUI. Opcionalmente desmárquelo y configure **Optional save interval** en ms/s/min/h; este intervalo selecciona muestras recibidas del CSV principal, no cambia la frecuencia del STM32 ni garantiza muestreo periódico exacto. El registro de eventos no se decima.
5. **Display time as** solo cambia la unidad del indicador, no la captura. START RECORDING inicia la sesión y STOP RECORDING vacía las colas y cierra los archivos. Time window termina automáticamente. La ventana temporal se aplica a la recepción monotónica, no al momento de dibujar o escribir. Una captura independiente puede continuar al detener la salida; Auto record conserva la vinculación a su propia sesión de salida. Los ensayos experimentales integrados del STM32 conservan sus reglas originales: registro continuo a tasa completa durante el ensayo, con parada por sus interlocks, no por la duración libre del diálogo.

Ejemplos: `500 ms` captura medio segundo; `30 s` captura 30 segundos; `2 min` captura 120 segundos; `1.5 h` captura 90 minutos. Una ventana más corta que el periodo de publicación puede contener cero muestras; no se generan datos ficticios.

## Archivos

- `<prefix>_<timestamp>.csv`: una fila por IMU primario recibido dentro de la ventana (salvo decimación seleccionada); conserva las columnas anteriores de actitud, referencias, errores, acelerómetro, giroscopio, magnetómetro/barómetro, DShot, potencia, cuaternión, validez, RPM equivalente, torque estimado y añade revisión del mapa, `capture_elapsed_s`, `capture_elapsed_ms`, `ros_stamp_ns`. `capture_elapsed` se mide desde el inicio de sesión e incluye el retardo si se configura. El timestamp ROS original se conserva; el reloj monotónico se utiliza para filtros y tiempo de captura. MAG/BARO son snapshots recientes, no necesariamente simultáneos con la IMU. Las columnas FMU/identificación se unen por timestamp ROS idéntico dentro de 120 ms; si no hay pareja, se conserva la IMU y se dejan sin confirmar los valores FMU. RPM equivalente no significa RPM medida. Se dejan vacíos los valores inválidos de orientación, magnetómetro, barómetro y DShot.
- `<prefix>_<timestamp>.csv.events.csv`: flujo independiente de eventos recibidos (sin decimación) con `capture_elapsed_s,event,payload`. `payload` está escapado como campo CSV; leer con un parser CSV, no dividir la línea entera por comas. El mismo reloj permite relacionar ambos archivos.
- `<prefix>_<timestamp>.csv.swing.csv`: para el Swing-up integrado, se escribe cada mensaje SWING_REAL de 13 campos recibido durante la sesión, incluyendo `capture_elapsed_s` adicional. El panel modal anterior conserva su log de visualización; la fuente de todos los mensajes recibidos sigue siendo events.csv. Los ACK, FAULT y STOP recibidos durante la sesión se registran en events.csv; el motivo final del ensayo integrado se escribe también en `.swing_status.txt`. No se promete capturar ACK que lleguen después de cerrar la sesión.
- `_metadata.yaml` / `_summary.txt`: si se habilitan, documentan configuración, cantidad de filas, eventos, decimación, faltantes de unión, desbordamientos y errores de escritura.

### Tipos del archivo events.csv

| event | payload |
|---|---|
| mag_uT | ROS stamp [s], Bx, By, Bz [uT] |
| baro_hPa_alt_m | ROS stamp [s], presión [hPa], altitud relativa [m] |
| motor | kind, DShot M1..M4, RPM solicitada M1..M4, RPM medida M1..M4, command_source, host_origin; grupos no aplicables vacíos |
| identification | ros_stamp_ns, fmu_dshot_valid, fmu_sample_ms, fmu_armed, fmu_state, fmu_dshot[4], host_dshot_valid, host_dshot_age_s, host_dshot[4], battery_valid, battery_voltage_v, power_tick_valid, power_sample_ms, power_age_s, current_valid, esc_current_a, esc_sense_valid, esc_current_sense_v, adc_voltage_raw, adc_current_raw, orientation_valid, qw,qx,qy,qz, wx,wy,wz [rad/s], ax,ay,az [m/s²]. Interpretar campos según sus flags; el contenido original no implica que un valor inválido sea una medición. |
| swing_status | Texto original de /cefiro/control/swing_tuning_status (o /real/...); incluye fase, ACK o error según mensaje |
| hardware_status | JSON de estado del nodo hardware |
| control_status | Texto original de estado del controlador |

Motor kind: 0 target RPM, 1 DShot directo, 2 RPM medida, 3 telemetría de motor, 4 DShot host confirmado, 5 snapshot DShot de FMU. Los valores de DShot son setpoints, no una medición eléctrica. Eventos de tipo motor pueden proceder de tópicos o de snapshots/estado y no son necesariamente órdenes únicas.

## Implementación y límites

- Cola de registro separada de la cola de visualización, tanto SIM como REAL. Cola de 32768 IMU y 32768 eventos; no se descartan filas silenciosamente cuando se llena: se contabiliza y se marca INCOMPLETE. El registro de IMU no depende de la coincidencia de identificación para guardar la fila.
- Suscripciones de IMU, identificación, MAG y BARO con historial ampliado a 512, conservando QoS compatible. No se puede recuperar una trama que nunca llegó a ROS/DDS. El logger no aumenta la frecuencia de publicación de la FMU. Los registros nativos no representan una medición a 200 Hz si el hardware publica a 25 Hz. La salida se vacía por lote y al cerrar; no se garantiza persistencia ante corte de alimentación abrupto o disco defectuoso.
- Captura temporal independiente con hasta 120 ms de gracia para resolver uniones pendientes (no prolonga la ventana de IMU; no prolonga ensayos de motores). Una unión tardía puede quedar disponible como evento independiente si llegó antes de cerrar y dentro de la ventana.
- Se conservan controles, ganancias, límites DShot, mapeo y protecciones del paquete base. También se incorporan los archivos de Auto-Tuning Rev.3 entregados previamente, para evitar regresar a las parejas antiguas. No se modifica el firmware ni la interfaz de hardware.
- No se dispone de ROS 2 Jazzy ni Qt6 en el entorno de preparación: no se ha ejecutado compilación Qt/ROS ni validación física. Las pruebas nativas y los checks estáticos ejecutados se adjuntan en `RECORDING_TEST_RESULTS.txt`.

## Compilar en ASUS

Sustituya la carpeta `cefiro_gui` anterior (no deje dos paquetes en src) por la del ZIP.

```bash
cd ~/cefiro_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to cefiro_gui --cmake-clean-cache --cmake-clean-first
source install/setup.bash
ros2 launch cefiro_gui cefiro_gui.launch.py use_sim_time:=false
```
