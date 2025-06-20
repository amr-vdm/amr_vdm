#!/usr/bin/env python3

import rospy

from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan

class rep117_filter_laser():
    def __init__(self):
        self.pub = rospy.Publisher('scan_filtered', LaserScan, queue_size=10)
        rospy.Subscriber('enable_rep117_filter', Bool, self.enable_filter_callback)
        rospy.Subscriber('scan', LaserScan, self.callback)

        self.enable_filter = True
    
    def enable_filter_callback(self, msg: Bool):
        self.enable_filter = msg.data

    def callback(self, msg:LaserScan):
        """
        Convert laser scans to REP 117 standard:
        http://www.ros.org/reps/rep-0117.html
        """
        ranges_out = []
        for dist in msg.ranges:
            if not self.enable_filter:
                ranges_out.append(float("inf"))
            elif dist > msg.range_max:
                ranges_out.append(float("inf"))
            elif dist < msg.range_min:
                ranges_out.append(float("-inf"))
            else:
                ranges_out.append(dist)

        msg.ranges = ranges_out
        self.pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('rep117_filter')
    try:
        rep117_filter_oj = rep117_filter_laser()
        rospy.loginfo("%s node is running", rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
