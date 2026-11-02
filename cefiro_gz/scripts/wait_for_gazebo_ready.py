#!/usr/bin/env python3
"""Wait for the local Gazebo world creation service (no fixed launch delays)."""

import argparse
import subprocess
import sys
import time


def parse_services(output):
    return {line.strip() for line in output.splitlines() if line.strip()}


def wait_for_create_service(world, timeout=120.0, period=0.25):
    expected = '/world/' + world + '/create'
    deadline = time.monotonic() + timeout if timeout > 0 else None
    announced = False

    while True:
        try:
            result = subprocess.run(
                ['gz', 'service', '--list'],
                capture_output=True, text=True, timeout=5, check=False,
            )
            available = parse_services(result.stdout) if result.returncode == 0 else set()
        except (OSError, subprocess.TimeoutExpired):
            available = set()

        if expected in available:
            print('[CEFIRO] Local Gazebo create service is ready', flush=True)
            return 0

        if not announced:
            print('[CEFIRO] Waiting for local Gazebo service: ' + expected,
                  flush=True)
            announced = True

        if deadline is not None and time.monotonic() >= deadline:
            print('[CEFIRO] ERROR: Gazebo create service unavailable: ' + expected,
                  file=sys.stderr, flush=True)
            return 1

        # Retry frequency only. This does not govern when the next model starts.
        time.sleep(period)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--world', default='main_world')
    parser.add_argument('--timeout', type=float, default=120.0)
    args = parser.parse_args()
    return wait_for_create_service(args.world, args.timeout)


if __name__ == '__main__':
    sys.exit(main())
