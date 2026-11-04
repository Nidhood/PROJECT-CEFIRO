# CEFIRO FMU V3 CF1 firmware protocol — actual modular sources

Source of truth: `CEFIRO_FMU_V3_Modular_Main_Explicito.zip`,
`MDK-ARM/Core/Src/cf_protocol.c`, `cf_fmu.c`, `cf_link.c`, `cf_config.h`.
This document describes what the uploaded firmware **actually** accepts.

## Framing

`@CF1|<seq:uint16 decimal>|<KIND>|<csv>*<CRC16:hex4>\n`

CRC16-CCITT-FALSE polynomial `0x1021`, init `0xFFFF`, over the bytes
`CF1|seq|KIND|csv` (no `@`, `*`, CRC or newline).
Maximum line `CF_FRAME_MAX=900`; USART2 at 115200 baud 8N1.

Valid example (generate an accurate CRC using the C++ encoder):
`encodeFrame(100, "CMD_DSHOT", {"0","48","100","300"})`.

Host and STM32 C encoder were cross-compiled and tested to generate
**byte-for-byte identical** frames.

## Supported incoming FMU commands

| Kind | CSV payload | Status / limitations |
|---|---|---|
| `CMD_STOP` | empty | Normal stop, clears manual output, no fault latch |
| `CMD_ARM` | `0` or `1` | MCU checks battery, ESC readiness, emergency latch; `ARM=1` needs ACK `OK` |
| `CMD_MODE` | `MANUAL` or `AUTO` | Only accepted while disarmed; bridge requests MANUAL |
| `CMD_DSHOT` | `d1,d2,d3,d4` | 0 stop or 48..300 inclusive; only while armed in MANUAL, active host watchdog |
| `CMD_ESTOP` | empty | Stop and latch MCU FMU FAULT |
| `CMD_RESET_FAULT` | empty | Only when disarmed and battery safe; use explicit reset service |
| `CMD_LQR` | 18 floating-point values | Configuration only, disarmed |
| `CMD_L` | 36 floating-point values | Observer gains; disarmed |
| `CMD_MIX` | 12 floating-point values | Mix matrix; disarmed |
| `CMD_BASE` | 1 DShot 48..300 | Base throttle; disarmed |
| `CMD_STRATEGY` | `LQR` or `STATE_FEEDBACK` | Supported selection, but no automatic actuation currently |
| `CMD_ATT` | 3 radians | Auto must already be running, thus inhibited currently |
| `CMD_ENABLE` | `0` or `1` | `1` returns `INHIBITED` while `CF_ENABLE_AUTOMATIC_OUTPUT=0` |
| `CMD_RPM` | any | `UNSUPPORTED` (no RPM feedback) |
| `CMD_DIR` | any | `UNSUPPORTED` (no reversible drive support) |
| `CMD_PID` | any | `UNSUPPORTED` |

ACK: `ACK,<command_sequence>,<CMD_KIND>,<STATUS>`. E.g. the payload
`101,CMD_DSHOT,OK` with its own surrounding CF1 frame and CRC.
Statuses include `OK`, `INVALID`, `LIMIT`, `DENIED`, `INHIBITED`,
`UNSUPPORTED`.

## Telemetry from current modular FMU

* `IMU`: `ax,ay,az,gx_dps,gy_dps,gz_dps,temp_c,uptime_ms` (7 numerical values + timestamp).
* `ATT`: `qw,qx,qy,qz,uptime_ms` (4 quaternion values + timestamp).
* `MAG`: `mx_uT,my_uT,mz_uT,uptime_ms`.
* `BARO`: `pressure_hPa,temp_c,alt_isa_m,raw_pressure_hPa,z_rel_m,uptime_ms`.
* `STAT`: `FMU_V3,state,calibrated,k_valid,auto_enabled,bad_packets,fault_count,imu_dma_errors,link_drops`.
* `ACK`: as above.

States: 0=UNCONFIGURED, 1=INACTIVE, 2=MANUAL, 3=AUTO, 4=FAULT.

Telemetry is emitted using the MCU-side `CfLink_Frame` and may lag behind
other events. Use ACK for confirmation of ARM/DShot; status alone does not
constitute proof of spinning motors.

## GUI integration and safeguarding

GUI sends `/real/cefiro/control/enable=true` followed by
`/real/cefiro_actuators/armed=true` when manual output is activated.
The bridge treats `enable` as a **software permission gate** and waits for
manual-mode confirmation and MCU `STAT` before sending `CMD_ARM,1`.
After matching ARM ACK OK, the bridge can translate a new GUI RPM setpoint to
`CMD_DSHOT` **if and only if** `rpm_to_dshot_enabled=true` and command TX enabled.

This translation is open-loop and not a measurement. The GUI continues to
publish nominal RPM while manual output is active; the hardware node enforces
a shorter watchdog (200 ms) than firmware's 300 ms.

GUI `SAFE STOP` -> `CMD_STOP` + `CMD_ARM,0`.
GUI `EMERGENCY ZERO` -> `CMD_ESTOP` (fault latch).
Remote reset requires explicit `std_srvs/srv/Trigger` call to
`/real/cefiro/hardware/reset_fault`; service acceptance != MCU ACK.

No real measured RPM or detailed motor telemetry exists in this firmware.
Do not publish simulated RPM as real feedback. Do not bypass firmware AUTO
inhibition or accept reversed motor direction requests.
