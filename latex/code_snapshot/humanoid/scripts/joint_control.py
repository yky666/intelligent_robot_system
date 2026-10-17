#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Calibrated joint position/time commands, default dry run. Python >= 3.5.

Speed means desired average speed of a finite position move, not velocity mode.
Positions are LX-824HV 0..1000 units, not PWM microseconds.
"""
from __future__ import print_function
import argparse
import importlib
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]


def number(value, label):
    if isinstance(value, bool):
        raise ValueError(label + ' must be a finite number')
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(label + ' must be finite')
    return result


class JointControl(object):
    def __init__(self, config, execute=False, board=None):
        self.config = config
        self.execute = execute
        self.board = board
        self.backend = config.get('hardware_backend', 'controller')
        if self.backend not in ('legacy', 'controller'):
            raise ValueError('hardware_backend must be legacy or controller')
        self.scale = number(config['degrees_per_unit'], 'degrees_per_unit')
        if self.scale <= 0:
            raise ValueError('degrees_per_unit must be positive')
        ids = [j['id'] for j in config['joints'].values()]
        if len(set(ids)) != len(ids) or any(isinstance(i, bool) or not isinstance(i, int) or not 1 <= i <= 16 for i in ids):
            raise ValueError('joint IDs must be unique integers in 1..16')
        if execute and config.get('hardware_verified') is not True:
            raise ValueError('Confirm hardware mapping and joint limits; then set hardware_verified: true')

    def joint(self, name_or_id):
        if name_or_id in self.config['joints']:
            return self.config['joints'][name_or_id]
        for joint in self.config['joints'].values():
            if str(joint['id']) == str(name_or_id):
                return joint
        raise ValueError('Unknown joint: ' + str(name_or_id))

    def calibration(self, joint):
        if joint.get('zero') is None or isinstance(joint.get('direction'), bool) or joint.get('direction') not in (-1, 1):
            raise ValueError('Set measured zero and direction before angle/speed control')
        zero = number(joint['zero'], 'zero')
        if not 0 <= zero <= 1000:
            raise ValueError('zero outside servo range')
        return zero, joint['direction']

    def position(self, joint, angle_deg):
        zero, direction = self.calibration(joint)
        return int(round(zero + direction * number(angle_deg, 'angle') / self.scale))

    def angle(self, joint, position):
        zero, direction = self.calibration(joint)
        return direction * (number(position, 'position') - zero) * self.scale

    def check_position(self, joint, position):
        position = number(position, 'position')
        if not position.is_integer() or not 0 <= position <= 1000:
            raise ValueError('position must be an integer in 0..1000')
        low, high = joint.get('min'), joint.get('max')
        if low is None or high is None:
            if self.execute:
                raise ValueError('Set measured min/max for every commanded joint')
        else:
            low, high = number(low, 'min'), number(high, 'max')
            if not 0 <= low < high <= 1000 or not low <= position <= high:
                raise ValueError('Target exceeds calibrated joint limits')
        return int(position)

    def connect(self):
        if self.board is None:
            for path in ('/home/pi/TonyPi', '/home/pi/TonyPi/HiwonderSDK', '/home/pi/TonyPi/HiwonderSDK/build/lib'):
                sys.path.insert(0, path)
            if self.backend == 'legacy':
                self.board = importlib.import_module('hiwonder.Board')
            else:
                sdk = importlib.import_module('hiwonder.ros_robot_controller_sdk')
                self.board = sdk.Board()
        return self.board

    def move_joint_group(self, positions, duration_ms):
        duration = number(duration_ms, 'duration_ms')
        if not duration.is_integer() or not 20 <= duration <= 9999:
            raise ValueError('duration_ms must be an integer in 20..9999')
        if not positions:
            raise ValueError('No joints specified')
        targets = []
        for name, value in positions.items():
            joint = self.joint(name)
            targets.append([joint['id'], self.check_position(joint, value)])
        if len(set(p[0] for p in targets)) != len(targets):
            raise ValueError('Same joint supplied twice using name and ID')
        targets.sort()
        command = {'duration_ms': int(duration), 'positions': targets, 'execute': self.execute}
        if self.execute:
            board = self.connect()
            if self.backend == 'legacy':
                for servo, pulse in targets:
                    board.setBusServoPulse(servo, pulse, int(duration))
            else:
                board.bus_servo_set_position(duration / 1000.0, targets)
        return command

    def set_joint_angle(self, name, angle_deg, duration_ms):
        return self.move_joint_group({name: self.position(self.joint(name), angle_deg)}, duration_ms)

    def move_joint_at_speed(self, name, start_deg, target_deg, speed_deg_s):
        speed = number(speed_deg_s, 'speed')
        if speed <= 0:
            raise ValueError('speed must be a positive magnitude')
        joint = self.joint(name)
        self.check_position(joint, self.position(joint, start_deg))
        target = self.position(joint, target_deg)
        # Use quantized commanded travel, not an unrepresentable fractional unit.
        travel = abs(target - self.position(joint, start_deg)) * self.scale
        if travel == 0:
            return {'execute': False, 'reason': 'no position change', 'positions': []}
        duration_ms = int(round(1000.0 * travel / speed))
        return self.move_joint_group({name: target}, duration_ms)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config', default=str(ROOT / 'humanoid/config/joints.yaml'))
    p.add_argument('--joint', required=True, help='configured name or bus servo ID')
    goal = p.add_mutually_exclusive_group(required=True)
    goal.add_argument('--position', type=int)
    goal.add_argument('--angle', type=float)
    p.add_argument('--duration-ms', type=int, default=1000)
    p.add_argument('--speed', type=float, help='positive average deg/s; requires angle and start-angle')
    p.add_argument('--start-angle', type=float, help='actual known starting angle; not a position readback')
    p.add_argument('--execute', action='store_true')
    args = p.parse_args()
    import yaml
    with open(args.config, encoding='utf-8') as stream:
        control = JointControl(yaml.safe_load(stream), execute=args.execute)
    if args.speed is not None:
        if args.angle is None or args.start_angle is None:
            p.error('--speed requires --angle and a known --start-angle')
        result = control.move_joint_at_speed(args.joint, args.start_angle, args.angle, args.speed)
    elif args.angle is not None:
        result = control.set_joint_angle(args.joint, args.angle, args.duration_ms)
    else:
        result = control.move_joint_group({args.joint: args.position}, args.duration_ms)
    print(json.dumps(result, indent=2))
    if not args.execute:
        print('Dry run: no hardware driver imported, no servo command sent.')


if __name__ == '__main__':
    main()
