#!/usr/bin/env python3
import rospy

from std_msgs.msg import Int16, Bool
from amr_msgs.msg import SafetyStatusStamped, SafetyStatus

NO_OBSTACLE = 0
OBSTACLE = 1

class FrontDepthScannerSafety():

    def __init__(self):

        self.front_depth_camera_frame_id_ = rospy.get_param("~camera_frame_id", "front_camera_depth_frame")

        # Publishers:
        self.pub_front_depth_scanner_status = rospy.Publisher("front_depth_scanner_status",
                                                              SafetyStatusStamped, queue_size=5)

        # Variables:
        self.prev_obstacle_state_ = SafetyStatus.NORMAL
        self.is_turn_off_front_depth_scanner = False

        # Subscribers:
        rospy.Subscriber("/front_camera/depth_scan/status", Int16, self.front_depth_scan_callback)
        rospy.Subscriber("turn_off_front_depth_safety", Bool, self.turn_off_front_depth_scanner)


    def turn_off_front_depth_scanner(self, msg:Bool):
        self.is_turn_off_front_depth_scanner = msg.data

    def front_depth_scan_callback(self, msg: Int16):

        if self.is_turn_off_front_depth_scanner:
            return
        
        if not msg.data:
            obstacle_state = SafetyStatus.NORMAL
        else:
            obstacle_state = SafetyStatus.PROTECTED

        if obstacle_state != self.prev_obstacle_state_:
            safety = SafetyStatusStamped()
            safety.header.frame_id = self.front_depth_camera_frame_id_
            safety.header.stamp = rospy.Time.now()
            safety.safety_status.status = obstacle_state
            self.pub_front_depth_scanner_status.publish(safety)
            self.prev_obstacle_state_ = obstacle_state


if __name__== '__main__':
    rospy.init_node("front_depth_scanner_safety_status")
    try:
        front_depth_scanner_safety = FrontDepthScannerSafety()
        rospy.loginfo("%s node is running!", rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass