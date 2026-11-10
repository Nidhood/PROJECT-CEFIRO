# CEFIRO GUI v1.7 - independent monitoring and control routing

- The ROS subscriptions listen to SIM and REAL concurrently. Neither depends on the other to render.
- IMU, magnetometer and barometer callbacks produce independent, tagged telemetry events; the latest event per topic and source is drawn each GUI cycle.
- When a sensor topic times out, its series is deleted automatically without clearing the other source. The source disconnects if all three topics are stale. Manual **CLEAR CHARTS** still clears everything.
- The status bar at lower left shows **SIM CONNECTED/DISCONNECTED** and **REAL CONNECTED/DISCONNECTED** based on fresh sensor messages, updated continuously. A green badge does not certify the motors are armed or flight-ready.
- The top bar has a **CONTROL TARGET** selector (SIMULATION / REAL HARDWARE), disabled while output is active. Both telemetry sources remain graphed regardless of selection.
- The dashboard has 9 panels. The barometer panel has two scales: hPa and relative altitude in m, on *separate* plots; magnetic components (microtesla) and magnetic norm are separate panels; DShot is host-commanded 0..2047 (never measured ESC throttle).
- Physical units: IMU angular rate rad/s -> deg/s, magnetic field Tesla -> microtesla, fluid pressure Pascal -> hPa, relative altitude derived from a fresh baseline. IMU quaternions use the configured SIM-only reference yaw; REAL orientations are not offset.
- Time-series history is bounded (600 s / 20,000 points per series) but never reset on page navigation or axis changes. Source disconnects reset only their own history.
- The LQR swing-up labeled SIM is blocked from being activated on REAL hardware. Before motor tests, **remove the propellers**.
- Build on ROS 2 Jazzy with Qt6 Charts and cefiro_msgs. **Compilation and physical-hardware tests have not been performed in this environment**.
