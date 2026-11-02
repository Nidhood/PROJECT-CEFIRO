# DShot throttle-field command, NOT motor RPM.
# 0 = motor stop, 1..47 = reserved and rejected, 48..2047 = throttle.
# Ordering: [motor_1, motor_2, motor_3, motor_4].
uint16[4] data
