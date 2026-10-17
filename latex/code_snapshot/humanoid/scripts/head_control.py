#!/usr/bin/env python3
"""Explicit PWM head positioning; dry run unless --execute is supplied."""
import argparse
import importlib
import sys


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--pulse', type=int, required=True, help='PWM microseconds; not body bus-servo units')
    p.add_argument('--servo', type=int, choices=(1, 2), default=1, help='1=pitch, 2=pan on inspected SDK')
    p.add_argument('--duration-ms', type=int, default=500)
    p.add_argument('--backend', choices=('controller', 'legacy'), default='controller')
    p.add_argument('--execute', action='store_true')
    a = p.parse_args()
    if not 500 <= a.pulse <= 2500 or not 20 <= a.duration_ms <= 3000:
        p.error('pulse must be 500..2500 us; duration 20..3000 ms')
    print('PWM servo=%d pulse=%d us duration=%d ms execute=%s' %
          (a.servo, a.pulse, a.duration_ms, a.execute))
    if not a.execute:
        return
    sys.path[:0] = ['/home/pi/TonyPi/HiwonderSDK', '/home/pi/TonyPi']
    if a.backend == 'controller':
        sdk = importlib.import_module('hiwonder.ros_robot_controller_sdk')
        controller = importlib.import_module('hiwonder.Controller')
        board = sdk.Board()
        try:
            controller.Controller(board).set_pwm_servo_pulse(a.servo, a.pulse, a.duration_ms)
        finally:
            board.port.close()
    else:
        importlib.import_module('hiwonder.Board').setPWMServoPulse(a.servo, a.pulse, a.duration_ms)


if __name__ == '__main__':
    main()
