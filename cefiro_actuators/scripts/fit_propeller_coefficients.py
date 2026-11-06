#!/usr/bin/env python3
"""Estimate dimensionless propeller CT/CQ from independently measured bench data.

Input CSV: rpm, thrust_n, torque_nm (optional torque_nm if unavailable).
For torque, use reaction torque magnitude on a dynamometer, not motor current.
"""

import argparse
import csv
import math
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv_file', type=Path)
    parser.add_argument('--diameter-m', type=float, required=True)
    parser.add_argument('--air-density', type=float, default=1.225)
    args = parser.parse_args()
    if args.diameter_m <= 0 or args.air_density <= 0:
        parser.error('Diameter and density must be positive')

    thrust_xy = []
    torque_xy = []
    with args.csv_file.open(newline='') as source:
        for row in csv.DictReader(source):
            try:
                rpm = float(row['rpm'])
                thrust = float(row['thrust_n'])
                if rpm <= 0 or not math.isfinite(rpm) or not math.isfinite(thrust):
                    continue
                omega2 = (rpm * 2.0 * math.pi / 60.0) ** 2
                thrust_xy.append((omega2, thrust))
                if row.get('torque_nm') not in (None, ''):
                    torque = float(row['torque_nm'])
                    if math.isfinite(torque):
                        torque_xy.append((omega2, abs(torque)))
            except (KeyError, ValueError):
                continue

    def slope0(samples):
        denominator = sum(x * x for x, _ in samples)
        if not denominator:
            return None
        return sum(x * y for x, y in samples) / denominator

    k_t = slope0(thrust_xy)
    k_q = slope0(torque_xy)
    if k_t is None:
        parser.error('CSV has no valid rpm/thrust pairs')
    scale = 4.0 * math.pi ** 2 / args.air_density
    ct = k_t * scale / (args.diameter_m ** 4)
    print(f'Valid thrust samples: {len(thrust_xy)}')
    print(f'propeller.ct: {ct:.8f}   # estimated, compare across RPM range')
    if k_q is not None:
        cq = k_q * scale / (args.diameter_m ** 5)
        print(f'propeller.cq: {cq:.8f}   # from measured torque')
    else:
        print('propeller.cq: unknown (torque_nm not provided)')
    print('These fits assume static thrust and constant coefficients across the tested RPMs.')


if __name__ == '__main__':
    main()
