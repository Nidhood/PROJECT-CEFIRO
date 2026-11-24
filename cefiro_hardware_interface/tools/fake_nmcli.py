#!/usr/bin/env python3
"""NetworkManager command double; never touches a real network."""
import os
import sys
from pathlib import Path

root = Path(os.environ['CF_WIFI_TEST_DIR'])
a = sys.argv[1:]
if a[:1] == ['--wait']:
    a = a[2:]

def read(name):
    p = root / name
    return p.read_text() if p.exists() else ''

def write(name, value):
    (root / name).write_text(value)

def delete(name):
    (root / name).unlink(missing_ok=True)

def value(key):
    return a[a.index(key) + 1]

if a[-2:] == ['device', 'status']:
    print('eth0:ethernet:connected\nwlan0:wifi:connected\nwlan1:wifi:' + ('connected' if read('busy') else 'disconnected'))
elif a[-3:] == ['connection', 'show', '--active']:
    print('ethernet-uuid:eth0')
    if read('active'):
        print(read('active') + ':wlan1')
elif 'wifi' in a and 'list' in a:
    assert value('ifname') == 'wlan1'
    print('Other WiFi')
    if read('ap') == 'yes':
        print('CEFIRO_FMU_V3')
elif a[:2] == ['connection', 'add']:
    assert value('save') == 'no' and value('ifname') == 'wlan1'
    assert value('connection.autoconnect') == 'no'
    assert value('ipv4.never-default') == 'yes'
    assert value('ipv4.ignore-auto-dns') == 'yes'
    assert value('ipv4.ignore-auto-routes') == 'yes'
    assert value('ipv6.method') == 'disabled'
    assert value('wifi-sec.psk-flags') == '2'
    assert value('ssid') == 'CEFIRO_FMU_V3'
    if read('deny') == 'yes':
        print('Error: not authorized', file=sys.stderr)
        sys.exit(1)
    write('profile', value('connection.uuid'))
elif a[:2] == ['connection', 'up']:
    assert value('uuid') == read('profile')
    assert value('ifname') == 'wlan1'
    p = Path(value('passwd-file'))
    assert p.stat().st_mode & 0o777 == 0o600
    assert p.read_text() == '802-11-wireless-security.psk:TestPassword123!\n'
    assert 'TestPassword123!' not in a
    write('password_path', str(p))
    write('active', value('uuid'))
elif a[:2] == ['connection', 'delete']:
    # Only the package's own transient UUID may be touched.
    assert value('uuid') != 'ethernet-uuid'
    if read('profile') and read('profile') == value('uuid'):
        delete('profile')
        delete('active')
else:
    print('unexpected nmcli invocation', a, file=sys.stderr)
    sys.exit(99)
