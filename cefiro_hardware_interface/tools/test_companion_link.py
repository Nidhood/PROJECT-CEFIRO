#!/usr/bin/env python3
"""Read-only CF1 test over the STM32 USART2 USB-UART adapter (not ESP native USB)."""
import argparse
import time
from collections import Counter
import binascii
def frame(seq):
    body = f'CF1|{seq}|CMD_PING|'.encode('ascii')
    return b'@' + body + f'*{binascii.crc_hqx(body, 0xffff):04X}\n'.encode('ascii')


def decode(raw):
    if not raw.startswith(b'@CF1|'):
        return None
    try:
        body, crc = raw[1:].rsplit(b'*', 1)
        if len(crc) != 4 or binascii.crc_hqx(body, 0xffff) != int(crc, 16):
            return None
        _, seq, kind, payload = body.decode('ascii').split('|', 3)
        return int(seq), kind, payload
    except (ValueError, UnicodeError):
        return None



def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port', required=True, help='/dev/ttyUSB0, /dev/serial/by-id/... or COMx')
    p.add_argument('--baud', type=int, default=115200)
    p.add_argument('--seconds', type=int, default=10, choices=range(1,61))
    p.add_argument('--hz', type=int, default=5, choices=range(2,21))
    p.add_argument("--require-sensors", action="store_true", help="Fail unless IMU, MAG and BARO frames are received")
    a=p.parse_args()
    try:
        import serial
    except ImportError:
        p.exit(2, 'Install pyserial: python -m pip install pyserial\n')
    pending=set(); counts=Counter(); seq=0; buf=b''; first_samples={}; received_bytes=0
    with serial.Serial(a.port, a.baud, timeout=0.01, write_timeout=0.5, xonxoff=False, rtscts=False, dsrdtr=False, exclusive=True) as link:
        link.reset_input_buffer(); link.reset_output_buffer()
        # Synchronize the MCU line parser before the first CRC-valid PING.
        link.write(b'\n')
        start=time.monotonic(); end=start+a.seconds; next_tx=start
        while time.monotonic()<end+0.4:
            now=time.monotonic()
            if now>=next_tx and now<end:
                seq+=1; link.write(frame(seq)); pending.add(seq); next_tx=now+1/a.hz
            chunk=link.read(4096); received_bytes+=len(chunk); buf+=chunk
            while b'\n' in buf:
                line,buf=buf.split(b'\n',1); line=line.rstrip(b'\r')
                if not line: continue
                decoded=decode(line)
                if not decoded: counts['INVALID']+=1; continue
                _,kind,payload=decoded; counts[kind]+=1
                if kind in ('IMU','ATT','MAG','BARO','STAT','BAT','PWR'):
                    first_samples.setdefault(kind,payload)
                if kind=='ACK':
                    f=payload.split(',')
                    if len(f)==3 and f[1:]==['CMD_PING','OK']:
                        try: pending.discard(int(f[0]))
                        except ValueError: pass
            if len(buf)>4096: raise RuntimeError('Unterminated CF1 input')
    print(f'PING sent={seq} ACK OK={seq-len(pending)} missing={len(pending)}')
    print('RX bytes:',received_bytes)
    print('Frame counts:',dict(counts))
    for kind,payload in sorted(first_samples.items()):
        print(f'{kind}: {payload}')
    missing_sensors = [kind for kind in ("IMU", "MAG", "BARO") if not counts[kind]]
    if a.require_sensors and missing_sensors:
        print("Missing required sensor frames:", ", ".join(missing_sensors))
    return 0 if seq and not pending and not counts['INVALID'] and (not a.require_sensors or not missing_sensors) else 1

if __name__=='__main__': raise SystemExit(main())
