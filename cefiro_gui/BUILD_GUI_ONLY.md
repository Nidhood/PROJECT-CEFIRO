# Compilar únicamente `cefiro_gui`

El error que menciona `cefiro_actuators/test/test_actuator_model.cpp` viene de
**cefiro_actuators**, no de la GUI.

## Dependencias

- ROS 2 Jazzy
- `cefiro_msgs`
- Qt6 Widgets y Charts

## Comandos recomendados

```bash
cd ~/cefiro_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select cefiro_msgs
source install/setup.bash
rm -rf build/cefiro_gui install/cefiro_gui
colcon build --symlink-install --packages-select cefiro_gui
source install/setup.bash
ros2 launch cefiro_gui cefiro_gui.launch.py
```

**Importante:** no uses `colcon build` sin `--packages-select` si solo deseas
construir la interfaz. No uses `--packages-up-to cefiro_gz` en esa prueba.

Si posteriormente deseas compilar también `cefiro_actuators`, elimina los
artefactos de compilación antiguos y asegúrate de que su `CMakeLists.txt` ya
no contenga el ejecutable `cefiro_motor_model_test`:

```bash
cd ~/cefiro_ws
rm -rf build/cefiro_actuators install/cefiro_actuators
grep -n 'cefiro_motor_model_test' src/cefiro_actuators/CMakeLists.txt
```

Si `grep` produce líneas, sigue siendo necesario corregir ese otro paquete.

## Integración de motores

La GUI publica `/cefiro_actuators/target_rpm` con
`cefiro_msgs/msg/MotorRpm`. Publica `/cefiro_actuators/motor_direction` con
`std_msgs/msg/Int8MultiArray`. Esta segunda orden solo surte efecto cuando
el nodo de actuadores implementa su recepción.
