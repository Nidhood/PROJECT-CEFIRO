from __future__ import annotations
from collections import deque
import json
import math
import os
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import FluidPressure, Imu, MagneticField, Temperature
from std_msgs.msg import Bool, Float64, Float64MultiArray, Int8MultiArray, String
from cefiro_msgs.msg import MotorRpm, MotorDshot, MotorTelemetry

from .legacy import ImuSample, MagSample, BaroSample, SensorError, parse_legacy_line
from .protocol import (LineBuffer, ProtocolError, decode_frame, encode_frame,
                       floats, integers)

try:
    import serial
except ImportError:
    serial = None

RAD_PER_DEG = math.pi / 180.0
MAX_TX_QUEUE = 64


class HardwareInterface(Node):
    def __init__(self):
        super().__init__('cefiro_hardware_interface')
        self.port = self.declare_parameter('port', '/dev/ttyUSB0').value
        self.baudrate = int(self.declare_parameter('baudrate', 115200).value)
        self.protocol_mode = self.declare_parameter('protocol_mode', 'legacy_text').value
        self.enable_command_tx = bool(self.declare_parameter('enable_command_tx', False).value)
        self.allow_direction_commands = bool(self.declare_parameter('allow_direction_commands', False).value)
        self.retry_s = float(self.declare_parameter('serial_reconnect_s', 2.0).value)
        self.rx_timeout_s = float(self.declare_parameter('sensor_stale_s', 1.5).value)
        self.command_timeout_s = float(self.declare_parameter('command_timeout_s', 0.5).value)
        self.imu_frame = self.declare_parameter('imu_frame_id', 'cefiro_fmu_imu').value
        self.mag_frame = self.declare_parameter('mag_frame_id', 'cefiro_fmu_mag').value
        self.baro_frame = self.declare_parameter('baro_frame_id', 'cefiro_fmu_baro').value
        self.body_frame = self.declare_parameter('body_frame_id', 'drone_body_link').value
        self.max_rpm = float(self.declare_parameter('max_command_rpm', 12000.0).value)
        self.orientation_timeout = float(self.declare_parameter('orientation_timeout_s', 0.2).value)
        if self.protocol_mode not in ('legacy_text', 'cf1'):
            raise ValueError('protocol_mode must be legacy_text or cf1')
        if self.baudrate <= 0 or self.retry_s <= 0 or self.command_timeout_s <= 0 or self.rx_timeout_s <= 0:
            raise ValueError('invalid serial/timing parameter')
        self.can_tx = self.protocol_mode == 'cf1' and self.enable_command_tx
        self.connected = False
        self.ser = None
        self.next_reconnect = 0.0
        self.codec_buffer = LineBuffer()
        self.tx_queue = deque(maxlen=MAX_TX_QUEUE)
        self.tx_pending = b''
        self.tx_offset = 0
        self.tx_seq = 0
        self.arm_requested = False
        self.arm_confirmed = False
        self.arm_ack_seq = None
        self.controller_enabled = False
        self.requested_rpm = [0.0] * 4
        self.last_command = None
        self.command_kind = None
        self.command_payload = None
        self.last_rx = None
        self.last_imu = None
        self.last_mag = None
        self.last_baro = None
        self.last_motor = None
        self.orientation = None
        self.last_orientation = None
        self.rx_frames = 0
        self.bad_frames = 0
        self.serial_errors = 0
        self.sensor_errors = {}
        self.mcu_status = 'NO MCU STATUS'
        self.mcu_boot_ms = None
        self._last_status_warn = 0.0

        # Relative names guarantee /real/... when launched namespace='real'.
        self.pub_imu = self.create_publisher(Imu, 'imu/data', qos_profile_sensor_data)
        self.pub_mag = self.create_publisher(MagneticField, 'magnetometer/data', qos_profile_sensor_data)
        self.pub_baro = self.create_publisher(FluidPressure, 'bmp581/air_pressure', qos_profile_sensor_data)
        self.pub_imu_temp = self.create_publisher(Temperature, 'cefiro/sensors/imu_temperature', 10)
        self.pub_baro_temp = self.create_publisher(Temperature, 'cefiro/sensors/baro_temperature', 10)
        self.pub_isa = self.create_publisher(Float64, 'cefiro/sensors/baro/isa_altitude', 10)
        self.pub_relative = self.create_publisher(Float64, 'cefiro/sensors/baro/relative_altitude', 10)
        self.pub_measured_rpm = self.create_publisher(MotorRpm, 'cefiro_actuators/measured_rpm', 10)
        self.pub_motor_state = self.create_publisher(MotorTelemetry, 'cefiro_actuators/telemetry', 10)
        self.pub_hardware_status = self.create_publisher(String, 'cefiro/hardware/status', 10)
        self.pub_controller_status = self.create_publisher(String, 'cefiro/control/status', 10)

        self.create_subscription(MotorRpm, 'cefiro_actuators/target_rpm', self.on_rpm, 10)
        self.create_subscription(MotorDshot, 'cefiro_actuators/dshot_throttle', self.on_dshot, 10)
        self.create_subscription(Bool, 'cefiro_actuators/armed', self.on_arm, 10)
        self.create_subscription(Int8MultiArray, 'cefiro_actuators/motor_direction', self.on_direction, 10)
        self.create_subscription(Bool, 'cefiro/control/enable', self.on_enable, 10)
        self.create_subscription(Bool, 'cefiro/control/emergency_stop', self.on_estop, 10)
        self.create_subscription(Vector3Stamped, 'cefiro/control/attitude_reference', self.on_attitude, 10)
        self.create_subscription(String, 'cefiro/control/mode', lambda m: self.on_string('CMD_MODE', m), 10)
        self.create_subscription(String, 'cefiro/control/strategy', lambda m: self.on_string('CMD_STRATEGY', m), 10)
        self.create_subscription(Float64MultiArray, 'cefiro/control/pid_gains',
                                 lambda m: self.on_gains('CMD_PID', m, 9), 10)
        self.create_subscription(Float64MultiArray, 'cefiro/control/lqr_gain',
                                 lambda m: self.on_gains('CMD_LQR', m, 18), 10)

        self.create_timer(0.01, self.io_tick)
        self.create_timer(0.50, self.publish_status)
        if serial is None:
            self.get_logger().error('pyserial unavailable: sudo apt install python3-serial')
        self.get_logger().info(
            f'CEFIRO hardware namespace /real | {self.port} {self.baudrate} 8N1 | '
            f'protocol={self.protocol_mode} | TX={"ENABLED" if self.can_tx else "DISABLED"}')
        if self.protocol_mode == 'legacy_text':
            self.get_logger().warn('Legacy main.c mode: IMU/MAG/BARO RX only; MCU has no motor command parser or attitude quaternion')

    def close_serial(self, detail=''):
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
        was_connected = self.connected
        self.ser = None
        self.connected = False
        self.next_reconnect = time.monotonic() + self.retry_s
        self.codec_buffer.clear()
        self.tx_queue.clear()
        self.tx_pending = b''
        self.tx_offset = 0
        self.arm_requested = False
        self.arm_confirmed = False
        self.arm_ack_seq = None
        self.last_command = None
        self.command_payload = None
        self.command_kind = None
        self.controller_enabled = False
        self.orientation = None
        self.last_orientation = None
        if detail and was_connected:
            self.get_logger().error('Serial disconnected; state reset, re-arm required: ' + detail)

    def connect_serial(self):
        if serial is None or time.monotonic() < self.next_reconnect:
            return
        self.next_reconnect = time.monotonic() + self.retry_s
        try:
            self.ser = serial.Serial(port=self.port, baudrate=self.baudrate,
                                     bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                                     stopbits=serial.STOPBITS_ONE, timeout=0, write_timeout=0)
            self.ser.reset_input_buffer()
            self.connected = True
            self.codec_buffer.clear()
            self.arm_requested = False
            self.arm_confirmed = False
            self.last_command = None
            self.get_logger().info('Serial connected: ' + self.port)
            if self.can_tx:
                self.queue_command('CMD_ESTOP', ())
                self.queue_command('CMD_ARM', ('0',))
        except (OSError, ValueError, serial.SerialException) as exc:
            self.ser = None
            self.connected = False
            # Avoid spamming warnings every 10ms.
            now = time.monotonic()
            if now - self._last_status_warn >= 5.0:
                self.get_logger().warn(f'Waiting for serial device {self.port}: {exc}')
                self._last_status_warn = now

    def queue_command(self, kind, fields):
        if not self.can_tx or not self.connected:
            return None
        self.tx_seq = (self.tx_seq + 1) & 0xFFFF
        try:
            packed = encode_frame(self.tx_seq, kind, fields)
        except (ProtocolError, UnicodeEncodeError) as exc:
            self.get_logger().warn(f'Rejected outgoing {kind}: {exc}')
            return None
        if kind in ('CMD_RPM', 'CMD_DSHOT'):
            # Keep only the newest motor command; never replay an old throttle
            # after a transient backlog on a busy 115200 baud UART.
            self.tx_queue = deque((item for item in self.tx_queue
                                   if b'|CMD_RPM|' not in item and b'|CMD_DSHOT|' not in item),
                                  maxlen=MAX_TX_QUEUE)
        if len(self.tx_queue) == MAX_TX_QUEUE:
            # Overflow must not allow stale actuation to play back later.
            self.tx_queue.clear()
            self.get_logger().error('TX queue overflow; clearing pending commands')
            self.arm_requested = False
            self.arm_confirmed = False
            self.tx_queue.append(encode_frame((self.tx_seq+1)&0xFFFF, 'CMD_ESTOP', ()))
            return None
        self.tx_queue.append(packed)
        return self.tx_seq

    def io_tick(self):
        if not self.connected:
            self.connect_serial()
        if not self.connected:
            return
        try:
            count = min(self.ser.in_waiting, 8192)
            if count:
                chunk = self.ser.read(count)
                for record in self.codec_buffer.feed(chunk):
                    self.handle_record(record)
            self.enforce_command_watchdog()
            # Nonblocking partial-write continuation.
            if not self.tx_pending and self.tx_queue:
                self.tx_pending = self.tx_queue.popleft()
                self.tx_offset = 0
            if self.tx_pending:
                wrote = self.ser.write(self.tx_pending[self.tx_offset:])
                self.tx_offset += int(wrote or 0)
                if self.tx_offset >= len(self.tx_pending):
                    self.tx_pending = b''
                    self.tx_offset = 0
        except (OSError, serial.SerialException, serial.SerialTimeoutException) as exc:
            self.serial_errors += 1
            self.close_serial(str(exc))

    def handle_record(self, record):
        if self.protocol_mode == 'legacy_text':
            try:
                decoded = record.decode('ascii', errors='replace')
                reading = parse_legacy_line(decoded)
                if isinstance(reading, SensorError):
                    self.sensor_errors[reading.sensor] = reading.detail
                elif isinstance(reading, ImuSample):
                    self.publish_imu(reading)
                elif isinstance(reading, MagSample):
                    self.publish_mag(reading)
                elif isinstance(reading, BaroSample):
                    self.publish_baro(reading)
            except (ValueError, OverflowError) as exc:
                self.bad_frames += 1
                self.get_logger().warn('Legacy parse error: ' + str(exc))
            return
        try:
            frame = decode_frame(record)
            self.handle_frame(frame)
            self.rx_frames += 1
        except (ProtocolError, ValueError, OverflowError) as exc:
            self.bad_frames += 1
            if self.bad_frames % 50 == 1:
                self.get_logger().warn(f'Dropped invalid CF1 frame: {exc}')

    def handle_frame(self, frame):
        data = frame.payload
        if frame.kind == 'IMU':
            # Values from firmware remain m/s^2, deg/s, Celsius.
            numbers = floats(data[:7], 7)
            if len(data) not in (7, 8):
                raise ProtocolError('IMU needs 7 or 8 fields')
            self.publish_imu(ImuSample(numbers[:3], numbers[3:6], numbers[6]))
        elif frame.kind == 'MAG':
            if len(data) not in (3, 4):
                raise ProtocolError('MAG needs 3 or 4 fields')
            self.publish_mag(MagSample(floats(data[:3], 3)))
        elif frame.kind == 'BARO':
            if len(data) not in (5, 6):
                raise ProtocolError('BARO needs 5 or 6 fields')
            b = floats(data[:5], 5)
            if b[0] <= 0 or b[3] <= 0:
                raise ProtocolError('invalid barometric pressure')
            self.publish_baro(BaroSample(*b))
        elif frame.kind == 'ATT':
            if len(data) not in (4, 5):
                raise ProtocolError('ATT needs 4 or 5 fields')
            q = floats(data[:4], 4)
            norm = math.sqrt(sum(x*x for x in q))
            if not 0.5 < norm < 1.5:
                raise ProtocolError('invalid attitude quaternion')
            self.orientation = tuple(x/norm for x in q)
            self.last_orientation = time.monotonic()
        elif frame.kind == 'MOTOR':
            # Summary is not sufficient to fill MotorTelemetry honestly.
            if len(data) != 11:
                raise ProtocolError('MOTOR summary needs 11 fields')
            speed = floats(data[:4], 4, minimum=0.0)
            currents = floats(data[4:8], 4)
            bus_voltage = floats(data[8:9], 1, minimum=0.0)[0]
            flags = integers(data[9:11], 2, 0, 1)
            msg = MotorRpm()
            msg.data = list(speed)
            self.pub_measured_rpm.publish(msg)
            self.last_motor = time.monotonic()
            if not flags[0]:
                self.arm_confirmed = False
            self.mcu_status = f'MOTOR rpm={list(map(round, speed))}, bus={bus_voltage:.2f}V, armed={flags[0]}, watchdog={flags[1]}'
        elif frame.kind == 'MOTOR_FULL':
            self.publish_full_motor(frame)
        elif frame.kind == 'ACK':
            if len(data) != 3:
                raise ProtocolError('ACK requires seq, command, status')
            seq = integers(data[:1], 1)[0]
            if seq == self.arm_ack_seq and data[1] == 'CMD_ARM':
                self.arm_confirmed = data[2] == 'OK' and self.arm_requested
                if not self.arm_confirmed:
                    self.arm_requested = False
                self.arm_ack_seq = None
        elif frame.kind == 'STAT':
            self.mcu_status = ','.join(data)
            message = String()
            message.data = self.mcu_status
            self.pub_controller_status.publish(message)
        elif frame.kind == 'ERROR':
            self.mcu_status = 'MCU ERROR: ' + ','.join(data)
            self.get_logger().warn(self.mcu_status)
        else:
            raise ProtocolError('unsupported CF1 message type: ' + frame.kind)

    def publish_full_motor(self, frame):
        data = frame.payload
        if len(data) != 50:
            raise ProtocolError('MOTOR_FULL requires exactly 50 fields')
        msg = MotorTelemetry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.body_frame
        names = ('requested_rpm','measured_rpm','dshot_value','dshot_packet',
                 'throttle_fraction','thrust_n','aerodynamic_torque_nm',
                 'reaction_torque_nm','electromagnetic_torque_nm','current_a','applied_voltage_v')
        for idx, name in enumerate(names):
            four = data[idx*4:(idx+1)*4]
            if name in ('dshot_value', 'dshot_packet'):
                setattr(msg, name, list(integers(four, 4)))
            else:
                setattr(msg, name, list(floats(four, 4)))
        msg.bus_voltage_v, msg.total_thrust_n, msg.total_reaction_yaw_nm = floats(data[44:47], 3)
        flags = integers(data[47:50], 3, 0, 255)
        if any(flag not in (0, 1) for flag in flags[:2]) or flags[2] not in (0, 1, 2):
            raise ProtocolError('invalid motor flags')
        msg.armed = bool(flags[0])
        msg.watchdog_ok = bool(flags[1])
        msg.command_source = flags[2]
        rpm = MotorRpm()
        rpm.data = list(msg.measured_rpm)
        self.pub_measured_rpm.publish(rpm)
        self.pub_motor_state.publish(msg)
        self.last_motor = time.monotonic()
        # Observed MCU status is not itself a substitute for ARM acknowledgement.
        if not msg.armed:
            self.arm_confirmed = False

    def publish_imu(self, s: ImuSample):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.imu_frame
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = s.acceleration
        gyro = [g*RAD_PER_DEG for g in s.gyro_dps]
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = gyro
        if self.last_orientation is not None and time.monotonic() - self.last_orientation <= self.orientation_timeout:
            w,x,y,z = self.orientation
            msg.orientation.w = w
            msg.orientation.x = x
            msg.orientation.y = y
            msg.orientation.z = z
            # Unknown orientation covariance: zero means not characterized.
        else:
            msg.orientation.w = 1.0
            msg.orientation_covariance[0] = -1.0
        self.pub_imu.publish(msg)
        temp = Temperature()
        temp.header = msg.header
        temp.temperature = s.temperature_c
        self.pub_imu_temp.publish(temp)
        self.last_imu = time.monotonic()
        self.last_rx = self.last_imu
        self.sensor_errors.pop('IMU', None)

    def publish_mag(self, s: MagSample):
        msg = MagneticField()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.mag_frame
        msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z = [v*1e-6 for v in s.magnetic_ut]
        self.pub_mag.publish(msg)
        self.last_mag = time.monotonic()
        self.last_rx = self.last_mag
        self.sensor_errors.pop('MAG', None)

    def publish_baro(self, s: BaroSample):
        msg = FluidPressure()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.baro_frame
        msg.fluid_pressure = s.pressure_hpa*100.0
        self.pub_baro.publish(msg)
        t = Temperature()
        t.header = msg.header
        t.temperature = s.temperature_c
        self.pub_baro_temp.publish(t)
        rel = Float64()
        rel.data = s.relative_altitude_m
        self.pub_relative.publish(rel)
        isa = Float64()
        isa.data = s.altitude_isa_m
        self.pub_isa.publish(isa)
        self.last_baro = time.monotonic()
        self.last_rx = self.last_baro
        self.sensor_errors.pop('BARO', None)

    def on_arm(self, msg: Bool):
        if not self.can_tx or not self.connected:
            self.arm_requested = False
            self.arm_confirmed = False
            return
        if not msg.data:
            self.disarm('ROS disarm request')
        else:
            self.arm_requested = True
            self.arm_confirmed = False
            self.command_payload = None
            self.last_command = None
            self.arm_ack_seq = self.queue_command('CMD_ARM', ('1',))
            self.get_logger().info('ARM requested; awaiting explicit MCU ACK')

    def disarm(self, reason=''):
        self.arm_requested = False
        self.arm_confirmed = False
        self.arm_ack_seq = None
        self.command_payload = None
        self.command_kind = None
        self.last_command = None
        self.requested_rpm = [0.0] * 4
        self.tx_queue.clear()
        self.tx_pending = b''
        self.tx_offset = 0
        self.queue_command('CMD_STOP', ())
        self.queue_command('CMD_ARM', ('0',))
        if reason:
            self.get_logger().info('Disarmed: ' + reason)

    def on_rpm(self, msg: MotorRpm):
        values = list(msg.data)
        if len(values) != 4 or not all(math.isfinite(v) and 0 <= v <= self.max_rpm for v in values):
            self.get_logger().warn('Invalid four-motor RPM command rejected')
            return
        self.requested_rpm = values
        self.motor_command('CMD_RPM', tuple(f'{x:.2f}' for x in values))

    def on_dshot(self, msg: MotorDshot):
        values = list(msg.data)
        if len(values) != 4 or not all(v == 0 or 48 <= v <= 2047 for v in values):
            self.get_logger().warn('Invalid DShot values; allowed 0 or 48..2047')
            return
        self.motor_command('CMD_DSHOT', tuple(str(v) for v in values))

    def motor_command(self, kind, payload):
        if not self.can_tx or not self.connected or not self.arm_requested:
            return
        self.last_command = time.monotonic()
        self.command_kind = kind
        self.command_payload = payload
        if self.arm_confirmed:
            self.queue_command(kind, payload)

    def on_direction(self, msg: Int8MultiArray):
        directions = list(msg.data)
        if not self.can_tx or not self.allow_direction_commands or not self.connected:
            return
        if len(directions) != 4 or any(v not in (-1, 1) for v in directions):
            self.get_logger().warn('Invalid direction request')
            return
        if self.arm_requested or self.arm_confirmed or any(r > 0.0 for r in self.requested_rpm):
            self.get_logger().warn('Direction change rejected while armed or nonzero RPM was requested')
            return
        self.queue_command('CMD_DIR', tuple(str(v) for v in directions))

    def on_enable(self, msg: Bool):
        self.controller_enabled = bool(msg.data)
        if self.can_tx:
            self.queue_command('CMD_ENABLE', ('1' if msg.data else '0',))

    def on_estop(self, msg: Bool):
        if not msg.data:
            return
        self.disarm('emergency stop')
        if self.can_tx:
            self.tx_queue.clear()
            self.tx_pending = b''
            self.tx_offset = 0
            self.queue_command('CMD_ESTOP', ())
            self.queue_command('CMD_ARM', ('0',))
        self.controller_enabled = False

    def on_attitude(self, msg: Vector3Stamped):
        if not self.can_tx or not self.controller_enabled or not self.arm_confirmed:
            return
        values = (msg.vector.x, msg.vector.y, msg.vector.z)
        if any(not math.isfinite(v) or abs(v) > math.pi + 0.01 for v in values):
            return
        self.queue_command('CMD_ATT', tuple(f'{v:.7f}' for v in values))

    def on_string(self, kind, msg: String):
        value = msg.data.strip().upper()
        if self.can_tx and value and len(value) <= 32 and all(c.isalnum() or c == '_' for c in value):
            self.queue_command(kind, (value,))

    def on_gains(self, kind, msg: Float64MultiArray, count: int):
        if not self.can_tx or not self.connected or len(msg.data) != count:
            return
        if not all(math.isfinite(v) and abs(v) < 1e7 for v in msg.data):
            return
        self.queue_command(kind, tuple(f'{v:.6g}' for v in msg.data))

    def enforce_command_watchdog(self):
        if not self.can_tx or not self.arm_requested:
            return
        now = time.monotonic()
        if self.last_command is not None and now - self.last_command > self.command_timeout_s:
            self.disarm('host command watchdog expired')

    def publish_status(self):
        now = time.monotonic()
        def age(stamp):
            return None if stamp is None else round(now - stamp, 3)
        status = {
            'serial_connected': self.connected,
            'device': self.port,
            'protocol': self.protocol_mode,
            'command_tx_enabled': self.can_tx,
            'armed_requested': self.arm_requested,
            'armed_acknowledged': self.arm_confirmed,
            'orientation_available': self.last_orientation is not None and age(self.last_orientation) <= self.orientation_timeout,
            'imu_online': self.last_imu is not None and age(self.last_imu) <= self.rx_timeout_s,
            'last_imu_age_s': age(self.last_imu),
            'mag_online': self.last_mag is not None and age(self.last_mag) <= self.rx_timeout_s,
            'last_mag_age_s': age(self.last_mag),
            'baro_online': self.last_baro is not None and age(self.last_baro) <= self.rx_timeout_s,
            'last_baro_age_s': age(self.last_baro),
            'last_motor_age_s': age(self.last_motor),
            'rx_valid_frames': self.rx_frames,
            'rx_invalid_frames': self.bad_frames + self.codec_buffer.dropped,
            'serial_errors': self.serial_errors,
            'sensor_errors': self.sensor_errors,
            'mcu_status': self.mcu_status,
            'timestamp_source': 'Raspberry Pi ROS clock at reception',
        }
        m = String()
        m.data = json.dumps(status, separators=(',', ':'))
        self.pub_hardware_status.publish(m)

    def destroy_node(self):
        if self.connected and self.can_tx:
            try:
                self.ser.write(encode_frame((self.tx_seq+1)&0xFFFF, 'CMD_ESTOP', ()))
                self.ser.write(encode_frame((self.tx_seq+2)&0xFFFF, 'CMD_ARM', ('0',)))
                self.ser.flush()
            except Exception:
                pass
        self.close_serial()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HardwareInterface()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
