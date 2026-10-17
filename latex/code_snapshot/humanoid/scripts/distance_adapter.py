#!/usr/bin/env python3
"""Hiwonder I2C Sonar -> atomic fresh distance file on the TonyPi.

Source contract: official hiwonder.Sonar.Sonar.getDistance() returns mm,
99999 on read failure, and caps out-of-range distances at 5000.
No servo commands. Unsupported/no-return values are published as invalid.
"""
import argparse
import importlib
import json
import math
import os
from pathlib import Path
import sys
import time


def write_sample(path, sample):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + '.tmp')
    temporary.write_text(json.dumps(sample), encoding='utf-8')
    os.replace(str(temporary), str(path))


def distance_sample(distance_mm):
    valid = isinstance(distance_mm, (int, float)) and not isinstance(distance_mm, bool) and math.isfinite(distance_mm) and 0 < distance_mm < 5000
    return {'timestamp': time.time(), 'distance_m': distance_mm / 1000.0 if valid else 0,
            'source': 'hiwonder_i2c_sonar', 'valid': valid}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--file', default='/tmp/tonypi_distance.json')
    parser.add_argument('--hz', type=float, default=20)
    args = parser.parse_args()
    if not math.isfinite(args.hz) or not 5 <= args.hz <= 50:
        parser.error('--hz must be 5..50')
    for directory in ('/home/pi/TonyPi', '/home/pi/TonyPi/HiwonderSDK', '/home/pi/TonyPi/HiwonderSDK/build/lib'):
        sys.path.insert(0, directory)
    sensor = importlib.import_module('hiwonder.Sonar').Sonar()
    try:
        while True:
            write_sample(args.file, distance_sample(sensor.getDistance()))
            time.sleep(1.0 / args.hz)
    except KeyboardInterrupt:
        pass
    finally:
        write_sample(args.file, distance_sample(None))


if __name__ == '__main__':
    main()
