#!/usr/bin/env python3
"""Standalone four-motor RPM step test. No Gazebo required."""

import argparse
import csv
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from cefiro_msgs.msg import MotorRpm, MotorTelemetry


class StepTest(Node):
    def __init__(self, logfile):
        super().__init__('cefiro_actuators_step_test')
        self.arm = self.create_publisher(Bool, '/cefiro_actuators/armed', 10)
        self.command = self.create_publisher(MotorRpm, '/cefiro_actuators/target_rpm', 10)
        self.out = logfile
        self.writer = csv.writer(self.out)
        self.writer.writerow(['time_s', 'rpm_ref_1', 'rpm_ref_2', 'rpm_ref_3', 'rpm_ref_4',
                              'rpm_1', 'rpm_2', 'rpm_3', 'rpm_4', 'thrust_n', 'bus_voltage_v'])
        self.t0 = time.monotonic()
        self.create_subscription(MotorTelemetry, '/cefiro_actuators/telemetry', self.on_state, 10)

    def on_state(self, msg):
        self.writer.writerow([round(time.monotonic() - self.t0, 6), *msg.requested_rpm,
                              *msg.measured_rpm, msg.total_thrust_n, msg.bus_voltage_v])

    def publish_command(self, data):
        msg = MotorRpm()
        msg.data = [float(x) for x in data]
        self.command.publish(msg)

    def set_arm(self, armed):
        msg = Bool()
        msg.data = armed
        self.arm.publish(msg)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--rpm', type=float, default=3000.0)
    parser.add_argument('--seconds', type=float, default=4.0)
    parser.add_argument('--output', type=Path, default=Path('cefiro_actuator_step.csv'))
    args = parser.parse_args()
    if not 0 <= args.rpm <= 20000 or args.seconds <= 0:
        parser.error('Invalid RPM or duration')
    rclpy.init()
    with args.output.open('w', newline='') as output:
        node = StepTest(output)
        try:
            # Run publisher at 50 Hz to keep the command watchdog fresh.
            node.set_arm(True)
            begin = time.monotonic()
            while time.monotonic() - begin < args.seconds:
                node.publish_command([args.rpm] * 4)
                rclpy.spin_once(node, timeout_sec=0.02)
        finally:
            node.publish_command([0.0] * 4)
            node.set_arm(False)
            for _ in range(10):
                rclpy.spin_once(node, timeout_sec=0.01)
            node.destroy_node()
    rclpy.shutdown()
    print(f'Saved simulated motor response to {args.output}')


if __name__ == '__main__':
    main()
