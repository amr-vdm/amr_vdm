#!/usr/bin/env python3
import rospy
import tf2_ros

from std_msgs.msg import Bool
from amr_msgs.msg import SafetyZone


class SafetyZoneType():

    def __init__(self):

        # Get params from server
        self.pub_frequency = rospy.get_param("~pub_frequency", 5)

        self.rate = rospy.Rate(self.pub_frequency)

        # Listen to Transfromation
        self.__tfBuffer = tf2_ros.Buffer(cache_time=rospy.Duration(5.0))
        self.__tf_listener = tf2_ros.TransformListener(self.__tfBuffer)

        # Variables:
        self.is_run_once = False
        self.turn_off_front_dept_safety = False

        # Publishers:
        self._pub_safety_zone_type = rospy.Publisher("safety_zone_type", SafetyZone, queue_size=5)
        self._pub_turn_off_front_depth_scan = rospy.Publisher("turn_off_front_depth_safety", Bool, queue_size=5)

        # Subscribers:
        rospy.Subscriber("state_runonce_nav", Bool, self.runOnceStateCb)
        rospy.Subscriber("/safety_filter/safety_state", Bool, self.safety_state_callback)
        rospy.Subscriber("turn_off_front_depth_autodock", Bool, self.turn_off_fd_autodock_cb)

    
    def runOnceStateCb(self, msg:Bool):
        self.is_run_once = msg.data


    def pubSafetyZoneType(self, type):
        msg = SafetyZone()
        msg.zone = type
        self._pub_safety_zone_type.publish(msg)

    def turn_off_fd_autodock_cb(self, msg: Bool):
        if not self.turn_off_front_dept_safety:
            self._pub_turn_off_front_depth_scan.publish(msg.data)


    def safety_state_callback(self, msg: Bool):
        self.turn_off_front_dept_safety = msg.data

        # if not self.is_run_once:
        #     return

        if msg.data:
            self.pubSafetyZoneType(SafetyZone.SMALL_ZONE)
            self._pub_turn_off_front_depth_scan.publish(True)
        else:
            self.pubSafetyZoneType(SafetyZone.BIG_ZONE)
            self._pub_turn_off_front_depth_scan.publish(False)


if __name__== '__main__':
    rospy.init_node("safety_zone_type")
    try:
        safety_zone_type = SafetyZoneType()
        rospy.loginfo("%s node is running!", rospy.get_name())
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
        