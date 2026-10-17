#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Ask AMCL for stationary scan updates so freshness checks don't deadlock."""
import rospy
from std_srvs.srv import Empty


def main():
    rospy.init_node('localization_refresh')
    refresh = rospy.ServiceProxy('request_nomotion_update', Empty)
    rate = rospy.Rate(1.5)
    while not rospy.is_shutdown():
        try:
            refresh()
        except rospy.ServiceException as exc:
            rospy.logwarn_throttle(5, 'AMCL stationary update unavailable: %s' % exc)
        rate.sleep()


if __name__ == '__main__':
    main()
