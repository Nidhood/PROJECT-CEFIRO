# Manual DShot full range — corrective release

Manual protocol ceiling: 2047 (11 bits, 2048 possible codes including zero). 0 stops, 1..47 reserved, 48..2047 throttle. The host defaults to 2047 but a lower startup cap remains selectable. The range is not a physically validated power limit. Explicit enable and arm are still required. Initial checks: ESC power disconnected; no propellers and restrained fixture for subsequent motor checks, with independent power cutoff. Never interpret nominal RPM as measured RPM.

Firmware: previous manual ramp (600 codes/s), immediate zero, motion and IMU checks, power checks, watchdog and fault latch unchanged. Generic AUTO keeps its original independent 300 cap; real pilot profile and enable gates unchanged. IMU defaults to eight fields. No change to AHRS, sensor drivers, motor map, identified model or controller gains. The powered Swing-Up profile remains inhibited pending calibration. Historical documents describing a manual hard limit of 300 refer to earlier builds and are superseded by this note.

ROS: hardware interface v0.8.8 restores the full range in constructor, YAML, both launch files and mapping defaults. Swing-Up subscription and CF1/IMU parsing unchanged. GUI does not need replacement.

Offline tests passed: full manual ramp to 2047, rejection of 2048, zero immediate, watchdog, synthetic Swing-Up/LQR, AHRS, sensor selection, host mapping, extracted swing callback and IMU parser, manual keepalive and power gate. ARM build, ROS 2 build and real device tests not performed here.
