# CEFIRO GUI 2.4.27 — corrección del identificador de roll Rev3

Base: cefiro_gui(6).zip. La GUI no impone un máximo manual inferior a 2047; la conversión de 12000 RPM nominales a 2047 fue verificada con el código real del hardware interface. No se modifican publicaciones de motores ni el hardware interface.

Cambios: importador de identificación utiliza M1/M4 y M2/M3 (orden lógico del firmware Rev3). Los nuevos CSV registran motor_map_revision desde hardware/status cuando está fresco; los antiguos sin ese campo no se importan al identificador. Para ajustar un modelo se requieren setpoints de FMU y AHRS de FMU, no solo solicitudes del host o fusión diagnóstica. Los registros REAL se guardan a la frecuencia recibida, sin decimación configurable. Los informes indican la revisión del modelo importado, y el modelo histórico sigue marcado como provisional. El signo físico de las parejas debe validarse de forma independiente. La aplicación completa Qt/ROS no fue compilada aquí.

El CSV manual suministrado tiene 13 filas durante 70.99 s, picos de 730 solicitados y 723 reportados por FMU; no prueba la capacidad física a 2047. La FMU reporta consignas aceptadas, no RPM ni torque medido. No se cambiaron límites de potencia o protecciones del firmware. No se ha validado físicamente una curva de torque a 2047.

Instalación: reemplazar únicamente cefiro_gui en el workspace del ASUS; colcon build --packages-up-to cefiro_gui --cmake-clean-cache; source install/setup.bash. Para la identificación use registros nuevos a frecuencia completa y el banco instrumentado con corte físico independiente. No reutilice modelos identificados con parejas anteriores.
