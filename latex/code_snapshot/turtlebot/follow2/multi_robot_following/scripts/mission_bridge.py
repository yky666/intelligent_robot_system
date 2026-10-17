#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""UDP heartbeat from the Pi; one bridge per ROS master, never a motion RPC."""
import json
import socket
import time
import rospy
from std_msgs.msg import String

try:
    string_types = (basestring,)
except NameError:
    string_types = (str,)


def main():
    rospy.init_node('mission_bridge')
    pub = rospy.Publisher('/competition/mission', String, queue_size=1)
    address = rospy.get_param('~bind', '0.0.0.0')
    port = rospy.get_param('~port', 15000)
    allowed_ip = rospy.get_param('~sender_ip', '')
    session = rospy.get_param('~session', 'team1')
    clock = getattr(time, 'monotonic', time.time)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((address, port))
    sock.settimeout(0.1)
    last = None
    state = 'idle'
    run_id = None
    sequence = -1
    valid = ('idle', 'searching', 'grasping', 'verifying_grasp', 'grasp_success',
             'grasp_failed', 'retrying', 'carrying', 'checking_hold',
             'checking_placement', 'releasing', 'verifying_release', 'done', 'fault', 'stopped')
    try:
        while not rospy.is_shutdown():
            try:
                raw, sender = sock.recvfrom(2048)
                data = json.loads(raw.decode('utf-8'))
                if not allowed_ip or sender[0] != allowed_ip or data.get('session') != session:
                    continue
                new_run = data.get('run_id')
                seq = data.get('sequence')
                if not isinstance(new_run, string_types) or not isinstance(seq, int):
                    continue
                # A new run is accepted only after the previous run timed out.
                if run_id is not None and run_id != new_run and last is not None and clock() - last <= 1.0:
                    continue
                if new_run != run_id:
                    run_id, sequence = new_run, -1
                if seq <= sequence or data.get('state') not in valid:
                    continue
                state, last, sequence = data['state'], clock(), seq
            except socket.timeout:
                pass
            except (ValueError, TypeError, UnicodeError, AttributeError):
                pass
            pub.publish(state if last is not None and 0 <= clock() - last <= 1.0 else 'stopped')
    finally:
        pub.publish('stopped')
        sock.close()


if __name__ == '__main__':
    main()
