#!/usr/bin/env python3
"""Select STM32 link via idle USART2 maintenance console; never sends ARM."""
import argparse
import time
import serial
p=argparse.ArgumentParser();p.add_argument('--port',default='/dev/ttyUSB0');p.add_argument('--baud',type=int,default=115200);p.add_argument('mode',choices=['usb','wifi','auto','status']);a=p.parse_args()
with serial.Serial(a.port,a.baud,timeout=.25,write_timeout=1,rtscts=False,dsrdtr=False,xonxoff=False,exclusive=True) as s:
    # Existing wired session expires after 750 ms with its ROS node stopped.
    time.sleep(1.0);s.reset_input_buffer();s.write(('\nLINK '+a.mode.upper()+'\n').encode('ascii'));s.flush()
    deadline=time.monotonic()+3
    while time.monotonic()<deadline:
        line=s.readline().decode('ascii',errors='replace').strip()
        if line.startswith(('LINK ','DENIED ')):
            print(line)
            if line.startswith('DENIED '):raise SystemExit(1)
            if ' SELECTED' in line or line.startswith('LINK MODE='):break
    else: raise SystemExit('No selection acknowledgement. Stop all hardware nodes, disarm, check USART2 and firmware version.')
