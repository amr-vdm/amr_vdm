#!/usr/bin/env python3
import rospy
import tf2_ros

from std_msgs.msg import Bool
from amr_msgs.msg import SafetyZone


class SafetyZoneType():

    def __init__(self):
        self.is_running_ = False
        self.turn_off_front_depth_safety_ = False

        # Publishers:
        self.safety_zone_type_pub_ = rospy.Publisher("safety_zone_type", SafetyZone, queue_size=5)
        self.turn_off_front_depth_scan_pub_ = rospy.Publisher("turn_off_front_depth_safety", Bool, queue_size=5)

        # Subscribers:
        rospy.Subscriber("state_runonce_nav", Bool, self.runonce_callback)
        rospy.Subscriber("/safety_filter/safety_state", Bool, self.safety_state_callback)
        rospy.Subscriber("turn_off_front_depth_autodock", Bool, self.turn_off_fd_autodock_cb)
    
    def runonce_callback(self, msg:Bool):
        self.is_running_ = msg.data

    def publish_zone_type(self, type):
        msg = SafetyZone()
        msg.zone = type
        self.safety_zone_type_pub_.publish(msg)

    def turn_off_fd_autodock_cb(self, msg: Bool):
        if not self.turn_off_front_depth_safety_:
            self.turn_off_front_depth_scan_pub_.publish(msg.data)

    def safety_state_callback(self, msg: Bool):
        self.turn_off_front_depth_safety_ = msg.data

        # if not self.is_running_:
        #     return

        if msg.data:
            self.publish_zone_type(SafetyZone.SMALL_ZONE)
            self.turn_off_front_depth_scan_pub_.publish(True)
            rospy.loginfo("/safety_zone_type: Published small zone!")
        else:
            self.publish_zone_type(SafetyZone.BIG_ZONE)
            self.turn_off_front_depth_scan_pub_.publish(False)
            rospy.loginfo("/safety_zone_type: Published big zone!")

if __name__== '__main__':
    rospy.init_node("safety_zone_type")
    try:
        safety_zone_type = SafetyZoneType()
        rospy.loginfo("%s node is running!", rospy.get_name())
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
        