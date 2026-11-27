# IMU and logical FMU throttle snapshot taken while processing the IMU sample.
# A setpoint is not proof of electrical waveform delivery or measured RPM.
sensor_msgs/Imu imu
bool fmu_dshot_valid
uint32 fmu_sample_ms
uint16[4] fmu_dshot
bool fmu_armed
uint8 fmu_state
bool host_dshot_valid
uint16[4] host_dshot
float64 host_dshot_age_s
bool battery_valid
float64 battery_voltage_v
# ESC-side BAT/PWR metadata; asynchronous, matched by MCU tick, not co-sampled IMU.
bool power_tick_valid
uint32 power_sample_ms
float64 power_age_s
bool current_valid
float64 esc_current_a
bool esc_sense_valid
float64 esc_current_sense_v
uint16 adc_voltage_raw
uint16 adc_current_raw
