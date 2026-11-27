# CEFIRO GUI 2.4.11 — Original AHRS display

Base: 2.4.9 (observer integrated into cefiro_gui). Removed all 2.4.10 accelerometer-only attitude substitution by restoring the original plotting functions. The original dashboard and onSample are identical to the user-supplied cefiro_gui(4).zip. The observer is read-only and requires a valid orientation. No change to control commands, hardware interface or firmware.

SYSTEM now distinguishes raw IMU online from AHRS valid and displays acceleration magnitude. No angle is synthesized when orientation_covariance[0] is -1.

With ESC power disconnected, verify `ros2 topic echo /real/imu/data --field orientation_covariance --qos-reliability best_effort --once`. If first element is -1, investigate the FMU AHRS, not GUI. The original firmware and current firmware use the same AHRS initialization gate: stationary acceleration norm within 0.7 m/s² of 9.80665. Previous raw samples had magnitude about 8.84 m/s². This is below the initialization threshold, but live FMU status/ATT is needed to confirm the current failure. A stationary restart with ESCs disconnected and a known safe orientation can help diagnose. Do not bypass AHRS validity or power protections.

Build: `colcon build --packages-select cefiro_gui --cmake-clean-cache` then source install/setup.bash and launch as usual.
