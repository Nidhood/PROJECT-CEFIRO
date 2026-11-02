#!/usr/bin/env python3

import os
import pty
import signal
import sys
import time


def main():
    master, slave = pty.openpty()
    tty = os.ttyname(slave)
    print('SIMULATED serial source (NOT real STM32).', flush=True)
    print('Use with: ros2 launch cefiro_hardware_interface hardware.launch.py port:=' + tty, flush=True)
    print('Ctrl+C to stop.', flush=True)
    signal.signal(signal.SIGINT, lambda *_: sys.exit(0))
    while True:
        data = (
            'IMU  | A[m/s2] X=-0.196 Y=-0.402 Z=9.814 | G[deg/s] X=-0.107 Y=-0.214 Z=0.015 | T=28.306 C\r\n'
            'MAG  | B[uT] X=21.100 Y=-5.200 Z=37.900\r\n'
            'BARO | P=742.054 hPa | T=27.650 C | Alt_ISA=2551.345 m | P_RAW=742.053 hPa | Z_REL=-1.083 m\r\n'
        )
        try:
            os.write(master, data.encode('ascii'))
        except OSError:
            break
        time.sleep(0.1)


if __name__ == '__main__':
    main()
